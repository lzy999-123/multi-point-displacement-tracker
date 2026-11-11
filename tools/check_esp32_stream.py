"""Measure real MJPEG delivery, including empty seconds and the final stall.

Requires opencv-python, numpy; pyserial only when --serial is supplied.
Example: python tools/check_esp32_stream.py http://192.168.1.10:81/stream
         --serial COM5 --duration 120 --output tmp/camera-check
"""
import argparse
import hashlib
import http.client
import json
import re
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit

import cv2
import numpy as np
from stream_metrics import summarize_frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("url")
    parser.add_argument("--serial")
    parser.add_argument("--command", default="", help="Optional test-firmware serial command")
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument("--timeout", type=float, default=2)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--min-fps", type=int, default=20)
    parser.add_argument("--max-gap-ms", type=float, default=150)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--output", type=Path, default=Path("tmp/camera-check"))
    args = parser.parse_args()
    url = urlsplit(args.url)
    if url.scheme != "http" or not url.hostname or args.duration <= 0 or args.timeout <= 0:
        parser.error("Use an HTTP URL and positive duration/timeout")
    if (args.warmup < 0 or args.duration < args.warmup + 1 or args.min_fps < 1 or
            args.max_gap_ms <= 0 or args.width < 1 or args.height < 1):
        parser.error("Need a full second after warmup, and positive fps/gap/dimensions")
    args.output.mkdir(parents=True, exist_ok=True)
    serial_data = bytearray()
    stop = threading.Event()
    port = None
    thread = None
    if args.serial:
        import serial
        port = serial.Serial(port=None, baudrate=115200, timeout=.1)
        port.dtr = False
        port.rts = False
        port.port = args.serial
        port.open()

        def read_serial():
            while not stop.is_set():
                serial_data.extend(port.read(port.in_waiting or 1))

        thread = threading.Thread(target=read_serial, daemon=True)
        thread.start()
        if args.command:
            port.write(args.command.encode("ascii"))
            port.flush()
            time.sleep(2)  # allow a protocol change to reassociate before measurement

    received = []
    errors = []
    failures = 0
    connections = 0
    attempted = 0
    last_jpeg = None
    start = time.monotonic()
    deadline = start + args.duration
    try:
        while time.monotonic() < deadline:
            conn = http.client.HTTPConnection(url.hostname, url.port or 80, timeout=args.timeout)
            attempted += 1
            try:
                target = (url.path or "/") + ("?" + url.query if url.query else "")
                conn.request("GET", target, headers={"Cache-Control": "no-cache"})
                response = conn.getresponse()
                if response.status != 200:
                    raise ValueError(f"HTTP {response.status}")
                connections += 1
                buffer = b""
                while time.monotonic() < deadline:
                    data = response.read1(65536)
                    if not data:
                        raise ConnectionError("Stream closed")
                    buffer += data
                    while True:
                        end = buffer.find(b"\r\n\r\n")
                        if end < 0:
                            break
                        header = buffer[:end]
                        match = re.search(rb"content-length:\s*(\d+)", header, re.I)
                        if not match:
                            raise ValueError("Missing MJPEG Content-Length")
                        size = int(match.group(1))
                        if size < 4 or size > 1024 * 1024:
                            raise ValueError(f"Invalid frame size {size}")
                        if len(buffer) < end + 4 + size:
                            break
                        jpeg = buffer[end + 4:end + 4 + size]
                        buffer = buffer[end + 4 + size:]
                        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                        if decoded is None:
                            failures += 1
                            continue
                        seq = re.search(rb"x-frame-seq:\s*(\d+)", header, re.I)
                        age = re.search(rb"x-frame-age-ms:\s*(\d+)", header, re.I)
                        capture = re.search(rb"x-capture-ms:\s*(\d+)", header, re.I)
                        received.append({
                            "seconds": time.monotonic() - start,
                            "bytes": size,
                            "sha256": hashlib.sha256(jpeg).hexdigest(),
                            "seq": int(seq.group(1)) if seq else None,
                            "source_age_ms": int(age.group(1)) if age else None,
                            "capture_ms": int(capture.group(1)) if capture else None,
                            "width": decoded.shape[1], "height": decoded.shape[0],
                        })
                        last_jpeg = jpeg
                    if len(buffer) > 2 * 1024 * 1024:
                        raise ValueError("MJPEG buffer exceeded 2 MB")
            except (OSError, ValueError, http.client.HTTPException) as error:
                errors.append({"seconds": round(time.monotonic() - start, 3),
                               "reason": f"{type(error).__name__}: {error}"})
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(min(.1, remaining))
            finally:
                conn.close()
    finally:
        end = time.monotonic()
        stop.set()
        if thread:
            thread.join(2)
        if port:
            port.close()

    metrics = summarize_frames(received, args.duration, args.warmup, args.width, args.height)
    times = [f["seconds"] for f in received if 0 <= f["seconds"] < args.duration]
    gaps = [b - a for a, b in zip(times, times[1:])]
    try:
        text = serial_data.decode("utf-8")
    except UnicodeDecodeError:
        text = serial_data.decode("gb18030", errors="replace")
    text = re.sub(r"(\[wifi\] 连接 )[^\r\n.]*", r"\1<SSID hidden>", text)
    report = {
        "url": args.url, "requested_s": args.duration, "observed_s": round(end - start, 3),
        "attempted_connections": attempted, "http_200_connections": connections,
        "received_frames": len(received), "frames_within_requested_window": len(times),
        "unique_jpegs": len({f['sha256'] for f in received}), "decode_failures": failures,
        "resolution": [received[-1]["width"], received[-1]["height"]] if received else None,
        "first_frame_ms": round(times[0] * 1000, 2) if times else None,
        "gap_ms": {"median": round(float(np.median(gaps)) * 1000, 2),
                   "p95": round(float(np.percentile(gaps, 95)) * 1000, 2),
                   "max": round(max(gaps) * 1000, 2)} if gaps else None,
        "errors": errors, "observed_boots": text.count("ESP-ROM:"),
        "faults": {word: text.count(word) for word in
                   ["Brownout", "Guru Meditation", "Backtrace", "panic", "watchdog", "assert"] if word in text},
        "criteria": {"warmup_s": args.warmup, "min_fps_each_second": args.min_fps,
                     "max_no_frame_ms": args.max_gap_ms},
        "serial_log": text,
    }
    report.update(metrics)
    report["criteria"].update(required_width=args.width, required_height=args.height,
                              min_source_fps_in_any_second=args.min_fps,
                              source_sequence_required=True)
    report["serial_observed"] = port is not None
    report["passed"] = bool(report["steady_min_source_fps"] >= args.min_fps and
                            report["steady_min_rolling_source_fps"] >= args.min_fps and
                            report["steady_max_no_frame_ms"] <= args.max_gap_ms and
                            report["wrong_resolution_frames"] == 0 and
                            report["missing_sequences"] == 0 and
                            report["duplicate_sequences"] == 0 and
                            report["out_of_order_sequences"] == 0 and
                            failures == 0 and not errors and report["observed_boots"] == 0 and not report["faults"])
    (args.output / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (args.output / "frames.json").write_text(json.dumps(received, indent=2) + "\n", encoding="utf-8")
    (args.output / "serial.log").write_text(text, encoding="utf-8")
    if last_jpeg:
        (args.output / "last.jpg").write_bytes(last_jpeg)
    print(json.dumps(report, ensure_ascii=True, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

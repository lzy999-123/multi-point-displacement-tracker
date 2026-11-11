"""Use the connected CH340 for real video; WiFi performance is measured separately.

Owns the serial port, localhost MJPEG proxy and Qt process. Closing the Qt window
stops the proxy and restores the board's normal 115200-baud WiFi mode.
Requires pyserial, numpy, opencv-python (already installed on this workstation).
"""
import argparse
from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time

import cv2
import numpy as np
import serial

MAGIC = b"ESPCAM1\n"


def frame_hash(data):
    result = 2166136261
    for value in data:
        result = ((result ^ value) * 16777619) & 0xFFFFFFFF
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--http-port", type=int, default=8181)
    parser.add_argument("--test-seconds", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path("tmp/esp32-fix-20261106/usb"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    runtime = root / "host/build/run"
    executable = runtime / ("live_viewer.exe" if args.test_seconds else "DisplacementTracker.exe")
    if not executable.is_file():
        parser.error(f"Missing viewer: {executable}")

    condition = threading.Condition()
    stop = threading.Event()
    latest = {"seq": -1, "jpeg": None}
    # 验收保留完整窗口；日常预览仅保留最近约5分钟，避免长期运行积累内存。
    records = deque(maxlen=None if args.test_seconds else 6000)
    errors = []
    checksum_errors = decode_errors = 0
    total_frames = 0
    started = time.monotonic()
    port = serial.Serial(port=None, baudrate=921600, timeout=.1)
    port.dtr = False
    port.rts = False
    port.port = args.port
    server = None
    reader = None
    server_thread = None
    process = None

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *unused):
            pass

        def do_GET(self):
            if self.path != "/stream":
                self.send_error(404)
                return
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace;boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            previous = -1
            try:
                while not stop.is_set():
                    with condition:
                        condition.wait_for(lambda: stop.is_set() or latest["seq"] != previous, timeout=1)
                        if stop.is_set():
                            break
                        if latest["seq"] == previous or latest["jpeg"] is None:
                            continue
                        previous, jpeg = latest["seq"], latest["jpeg"]
                    header = (f"\r\n--frame\r\nContent-Type: image/jpeg\r\n"
                              f"Content-Length: {len(jpeg)}\r\nX-Frame-Seq: {previous}\r\n\r\n").encode()
                    self.wfile.write(header + jpeg)
                    self.wfile.flush()
            except (OSError, ConnectionError):
                pass
            self.close_connection = True

    def read_frames():
        nonlocal checksum_errors, decode_errors, total_frames
        buffer = bytearray()
        last_status = started
        try:
            while not stop.is_set():
                buffer.extend(port.read(port.in_waiting or 1))
                while True:
                    index = buffer.find(MAGIC)
                    if index < 0:
                        del buffer[:-len(MAGIC)]
                        break
                    if index:
                        del buffer[:index]
                    if len(buffer) < 20:
                        break
                    size, seq, checksum = struct.unpack_from("<III", buffer, 8)
                    if size < 4 or size > 256 * 1024:
                        del buffer[0]
                        continue
                    if len(buffer) < 20 + size:
                        break
                    jpeg = bytes(buffer[20:20 + size])
                    del buffer[:20 + size]
                    if frame_hash(jpeg) != checksum:
                        checksum_errors += 1
                        continue
                    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        decode_errors += 1
                        continue
                    records.append({"s": round(time.monotonic() - started, 6), "seq": seq,
                                    "bytes": size, "sha256": hashlib.sha256(jpeg).hexdigest(),
                                    "width": image.shape[1], "height": image.shape[0]})
                    total_frames += 1
                    now = time.monotonic()
                    if now - last_status >= 5:
                        (output / "status.json").write_text(json.dumps({
                            "transport": "USB serial", "valid_frames": total_frames,
                            "last_frame_s": round(now - started, 3), "seq": seq,
                            "checksum_errors": checksum_errors, "decode_errors": decode_errors,
                        }, indent=2), encoding="utf-8")
                        (output / "last.jpg").write_bytes(jpeg)
                        last_status = now
                    with condition:
                        latest.update(seq=seq, jpeg=jpeg)
                        condition.notify_all()
        except (OSError, serial.SerialException) as error:
            errors.append(str(error))
            stop.set()
            with condition:
                condition.notify_all()

    try:
        # Recover either baud without toggling DTR/RTS or resetting the camera.
        port.open()
        port.write(b"\nWIFI\n")
        port.flush()
        time.sleep(.15)
        port.baudrate = 115200
        port.reset_input_buffer()
        port.write(b"\nUSBVIDEO\n")
        port.flush()
        time.sleep(.1)
        port.baudrate = 921600
        port.reset_input_buffer()
        reader = threading.Thread(target=read_frames, daemon=True)
        reader.start()
        with condition:
            ready = condition.wait_for(lambda: latest["jpeg"] is not None or stop.is_set(), timeout=10)
        if not ready or latest["jpeg"] is None:
            raise RuntimeError("No valid USB image within 10 seconds")
        server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        url = f"http://127.0.0.1:{server.server_port}/stream"
        env = os.environ.copy()
        env["QT_QPA_PLATFORM"] = "windows"
        env["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(runtime / "platforms")
        command = [str(executable), url, str(args.test_seconds), str(output / "qt.json")] if args.test_seconds else [str(executable), "--stream-url", url]
        print(f"USB real camera {args.port} -> {url}", flush=True)
        with (output / "viewer.log").open("wb") as log:
            process = subprocess.Popen(command, cwd=runtime, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       creationflags=subprocess.CREATE_NO_WINDOW)
            while process.poll() is None and not stop.wait(.2):
                pass
            if process.poll() is None:
                process.terminate()
            code = process.wait(timeout=5)
        print(f"Viewer exit {code}; valid USB frames {total_frames}", flush=True)
    finally:
        stop.set()
        with condition:
            condition.notify_all()
        if process and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        if server:
            server.shutdown()
            server.server_close()
        if server_thread:
            server_thread.join(2)
        if reader:
            reader.join(2)
        if port.is_open:
            try:
                port.write(b"WIFI\n")
                port.flush()
                time.sleep(.15)
            finally:
                port.close()
        (output / "serial-frames.json").write_text(json.dumps(list(records), indent=2), encoding="utf-8")
        (output / "serial-result.json").write_text(json.dumps({
            "transport": "USB serial 921600; WiFi not used for video",
            "valid_frames": total_frames, "retained_frames": len(records),
            "distinct_jpegs_retained": len({r["sha256"] for r in records}),
            "checksum_errors": checksum_errors, "decode_errors": decode_errors, "errors": errors,
        }, indent=2), encoding="utf-8")
        if latest["jpeg"]:
            (output / "last.jpg").write_bytes(latest["jpeg"])
    return code


if __name__ == "__main__":
    raise SystemExit(main())

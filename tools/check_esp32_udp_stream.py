"""Decode and measure VID1 source frames; Qt live_viewer measures actual display."""
import argparse
from bisect import bisect_right
from datetime import datetime
import hashlib
import json
import math
import socket
import time
from pathlib import Path

import cv2
import numpy as np
from vid1 import Assembler


def summarize(frames, duration, warmup, min_fps, max_gap_ms, decode_errors, width=320, height=240):
    frames = [f for f in frames if 0 <= f['time_s'] < duration]
    bins = [0] * int(duration)
    for frame in frames:
        index = int(frame['time_s'])
        if index < len(bins):
            bins[index] += 1
    times = [f['time_s'] for f in frames if f['time_s'] >= warmup]
    boundaries = [float(warmup)] + times + [duration]
    max_gap = max(b - a for a, b in zip(boundaries, boundaries[1:])) * 1000
    starts = [float(warmup), duration - 1]
    starts.extend(t for t in times if t <= duration - 1)
    rolling_min = min(bisect_right(times, t + 1) - bisect_right(times, t) for t in starts)
    minimum = min(bins[warmup:], default=0)
    wrong_resolution = sum((f['width'], f['height']) != (width, height) for f in frames)
    passed = (bool(frames) and minimum >= min_fps and rolling_min >= min_fps
              and max_gap <= max_gap_ms and decode_errors == 0 and wrong_resolution == 0)
    return dict(total_frames=len(frames), average_fps=round(len(frames) / duration, 2),
                min_fps_after_warmup=minimum, min_rolling_fps_after_warmup=rolling_min,
                max_gap_ms=round(max_gap, 2), per_second_fps=bins,
                wrong_resolution_frames=wrong_resolution, decode_errors=decode_errors, passed=passed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target', help='Camera IPv4:port')
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=240)
    parser.add_argument('--min-fps', type=int, default=20)
    parser.add_argument('--max-gap-ms', type=float, default=150)
    parser.add_argument('--output', type=Path, default=Path('tmp/udp-check'))
    parser.add_argument('--firmware-manifest', type=Path)
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or args.warmup < 0 or args.duration < args.warmup + 1
            or args.width < 1 or args.height < 1 or args.min_fps < 1
            or not math.isfinite(args.max_gap_ms) or args.max_gap_ms <= 0):
        parser.error('duration must cover warmup + 1s; fps and gap must be positive')
    try:
        host, separator, port_text = args.target.partition(':')
        host = str(__import__('ipaddress').IPv4Address(host))
        port = int(port_text) if separator else 5006
        if not 1 <= port <= 65535:
            raise ValueError('port out of range')
    except ValueError as exc:
        parser.error(str(exc))
    args.output.mkdir(parents=True, exist_ok=True)

    # Let the OS choose a source for the actual route, then bind that interface.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
        route.connect((host, port))
        local_ip = route.getsockname()[0]
    # 若目标处于局域网网段 (如 192.168.110.x)，强制匹配物理网卡 IP，绝不误绑到 Mihomo/VPN TUN 虚拟网卡 (198.18.x.x)
    target_prefix = '.'.join(host.split('.')[:3]) + '.'
    for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
        cand = info[4][0]
        if cand.startswith(target_prefix):
            local_ip = cand
            break
    print(f'Camera {host}:{port}; receiver interface {local_ip}', flush=True)
    assembler = Assembler()
    frames = []
    packets = unexpected = decode_errors = resets = 0
    last_jpeg = None
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((local_ip, 0))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
        # 启动即激活双向链路探测 (VPIN -> VPON)，更新 Windows 邻居 ARP 缓存，杜绝开局单向丢包
        sock.settimeout(0.5)
        for _ in range(5):
            sock.sendto(b'VPIN', (host, port))
            try:
                pong, _ = sock.recvfrom(64)
                if pong.startswith(b'VPON'):
                    break
            except (socket.timeout, ConnectionResetError):
                pass
            time.sleep(0.05)
        sock.settimeout(0.05)
        for _ in range(3):
            sock.sendto(b'VREQ', (host, port))
            time.sleep(0.005)
        start = time.monotonic()
        deadline = start + args.duration
        last_heartbeat = start
        last_good = start
        sequence_reset_at = start
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - max(last_good, sequence_reset_at) >= 1.5:
                assembler.reset()
                sequence_reset_at = now
                resets += 1
            if now - last_heartbeat >= .2:
                sock.sendto(b'VREQ', (host, port))
                last_heartbeat = now
            try:
                data, source = sock.recvfrom(2048)
            except (socket.timeout, ConnectionResetError):
                continue
            now = time.monotonic()
            if now >= deadline:
                break
            packets += 1
            if source != (host, port):
                unexpected += 1
                continue
            result = assembler.feed(data, now)
            if result is None:
                continue
            device, seq, jpeg = result
            try:
                image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            except cv2.error:
                image = None
            if image is None:
                decode_errors += 1
                continue
            decoded_at = time.monotonic()
            if decoded_at >= deadline:
                break
            assembler.delivered(device, seq)
            last_good = decoded_at
            frames.append(dict(time_s=decoded_at-start, receive_time_s=now-start, seq=seq,
                               device_id=device, width=image.shape[1], height=image.shape[0],
                               bytes=len(jpeg), sha256=hashlib.sha256(jpeg).hexdigest()))
            last_jpeg = jpeg

    summary = summarize(frames, args.duration, args.warmup, args.min_fps,
                        args.max_gap_ms, decode_errors, args.width, args.height)
    summary.update(duration_s=args.duration, elapsed_s=round(time.monotonic()-start, 3),
                   target=f'{host}:{port}', local_ip=local_ip,
                   measured_at=datetime.now().astimezone().isoformat(),
                   total_packets=packets, unexpected_source_packets=unexpected,
                   invalid_packets=assembler.invalid_packets,
                   duplicate_or_old_packets=assembler.duplicate_or_old_packets,
                   recovery_resets=resets, acceptance_scope='decoded UDP frames; not Qt display',
                   criteria=dict(width=args.width, height=args.height, warmup_s=args.warmup,
                                 min_fps=args.min_fps, max_gap_ms=args.max_gap_ms),
                   tool_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (Path(__file__), Path(__file__).with_name('vid1.py'))})
    if args.firmware_manifest:
        summary['local_build_manifest'] = json.loads(args.firmware_manifest.read_text(encoding='utf-8-sig'))
        summary['firmware_identity_note'] = 'Local build metadata; not proof of the running device version'
    (args.output/'frames.json').write_text(json.dumps(frames, indent=2), encoding='utf-8')
    (args.output/'result.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    if last_jpeg:
        (args.output/'last.jpg').write_bytes(last_jpeg)
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

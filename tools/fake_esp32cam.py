"""模拟 1～3 台 ESP32-CAM，用于没有硬件时测试上位机。

每台设备在 (8080+id) 端口提供 /stream（MJPEG，640x480），画面上有一个缓慢移动的色块；
同时在 UDP 39093 响应设备发现（协议见 docs/通信协议.md）。

用法：
    python tools/fake_esp32cam.py              # 3 台
    python tools/fake_esp32cam.py --count 1 --fps 20
需要 opencv-python 和 numpy。
"""

import argparse
import json
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

COLORS = {1: (255, 0, 0), 2: (0, 200, 0), 3: (0, 0, 255)}  # BGR：蓝、绿、红
DISCOVERY_PORT = 39093


def render(device_id: int, t: float) -> bytes:
    img = np.full((480, 640, 3), 80, np.uint8)
    cv2.putText(img, f"FAKE CAM #{device_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2)
    cx = int(320 + 180 * math.sin(t * 0.6 + device_id))
    cy = int(240 + 120 * math.sin(t * 0.9 + device_id * 2))
    cv2.circle(img, (cx, cy), 35, COLORS[device_id], cv2.FILLED)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert ok
    return buf.tobytes()


def make_handler(device_id: int, fps: float):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace;boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            start = time.monotonic()
            try:
                while True:
                    jpeg = render(device_id, time.monotonic() - start)
                    self.wfile.write(b"\r\n--frame\r\n")
                    self.wfile.write(f"Content-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.flush()
                    time.sleep(1.0 / fps)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

    return Handler


def discovery_loop(ids):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", DISCOVERY_PORT))
    print(f"[discovery] 监听 UDP {DISCOVERY_PORT}")
    while True:
        data, addr = sock.recvfrom(2048)
        try:
            req = json.loads(data)
        except ValueError:
            continue
        if req.get("type") != "discover" or not isinstance(req.get("nonce"), str):
            continue
        for device_id in ids:
            port = 8080 + device_id
            reply = {
                "type": "announce", "version": 1, "nonce": req["nonce"], "id": device_id,
                "ip": "127.0.0.1", "stream": f"http://127.0.0.1:{port}/stream", "control_port": 5005,
            }
            sock.sendto(json.dumps(reply).encode(), addr)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=3, choices=[1, 2, 3])
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--no-discovery", action="store_true")
    parser.add_argument("--bind", default="0.0.0.0",
                        help="HTTP 监听地址；上位机按发现回复的来源 IP 连接，所以默认监听所有网卡")
    args = parser.parse_args()

    ids = list(range(1, args.count + 1))
    for device_id in ids:
        server = ThreadingHTTPServer((args.bind, 8080 + device_id), make_handler(device_id, args.fps))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"[cam {device_id}] http://{args.bind}:{8080 + device_id}/stream")
    if not args.no_discovery:
        threading.Thread(target=discovery_loop, args=(ids,), daemon=True).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

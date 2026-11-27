"""模拟 1～3 台 ESP32-CAM，用于没有硬件时测试上位机。

支持两种模式：
1. artifact（默认）：模拟博物馆展台藏品（瓶子），从正面、侧面、斜俯视三个机位监控；
   随时间演示稳定静止、缓慢倾斜、尺度微调和恢复，用于测试免标定异动监控。
2. color：兼容经典色块模式。

用法：
    python tools/fake_esp32cam.py              # 3 台展品监测模式
    python tools/fake_esp32cam.py --mode color # 经典色块模式
    python tools/fake_esp32cam.py --count 1 --fps 20
"""

import argparse
import json
import math
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

COLORS = {1: (255, 0, 0), 2: (0, 200, 0), 3: (0, 0, 255)}  # BGR
DISCOVERY_PORT = 39093
UDP_BASE_PORT = 5006


def udp_camera_loop(device_id, bind, mode):
    """Offer all production sizes; synthetic uncapped sources run at 30fps."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind, UDP_BASE_PORT + device_id - 1))
    sock.settimeout(.005)
    peer, heartbeat_at, next_frame = None, 0.0, 0.0
    sequence, profile, generation, fps = 0, 0, 0, 12
    confirm = False
    flags = 0
    started = time.monotonic()
    while True:
        try:
            request, sender = sock.recvfrom(2048)
        except (socket.timeout, ConnectionResetError):
            request, sender = b"", None
        now = time.monotonic()
        if request.startswith(b"VPIN"):
            sock.sendto(b"VPON" + request[4:], sender)
        elif request == b"VSTP" and sender == peer:
            peer = None
        elif request.startswith(b"VREQ"):
            if len(request) == 16 and request[:6] == b"VREQA3":
                if request[12] not in (0, 1, 2, 3):
                    continue
                profile = request[12]
                generation = int.from_bytes(request[13:15], "little")
                fps = {0: 0, 1: 0, 2: 10, 3: 0}[profile]
                flags = request[15] & 3
                confirm = True
            elif request[:6] == b"VREQA2" and len(request) == 12:
                profile, fps, confirm = -1, max(5, min(40, request[6])), False
            elif request in (b"VREQ", b"VREQA"):
                profile, fps, confirm = -1, 12, False
            else:
                continue
            peer, heartbeat_at = sender, now
        if peer is None or now - heartbeat_at > 10 or now < next_frame:
            continue
        jpeg = (render_color_blob if mode == "color" else render_artifact)(device_id, now - started)
        width, height = (800, 600) if profile == 3 else (320, 240) if profile in (1, 2) else (640, 480)
        if profile in (1, 2, 3):
            image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(".jpg", image)
            if not ok:
                continue
            jpeg = encoded.tobytes()
        if confirm:
            sock.sendto(struct.pack("<4sBBHHHBBH", b"VCFG", device_id, profile,
                                    generation, width, height, fps, 0, flags), peer)
            confirm = False
        sequence = (sequence + 1) & 65535
        count = (len(jpeg) + 1399) // 1400
        if count <= 47:
            for index in range(count):
                payload = jpeg[index * 1400:(index + 1) * 1400]
                header = struct.pack("<IBBHBBHI", 0x31444956, device_id, profile + 1,
                                     sequence, index, count, len(payload), len(jpeg))
                sock.sendto(header + payload, peer)
        # No catch-up burst if rendering took longer than one frame period.
        interval = 1.0 / (fps or 30)
        next_frame = max(next_frame + interval, time.monotonic() + .001)


def render_color_blob(device_id: int, t: float) -> bytes:
    img = np.full((480, 640, 3), 80, np.uint8)
    cv2.putText(img, f"COLOR CAM #{device_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2)
    cx = int(320 + 180 * math.sin(t * 0.6 + device_id))
    cy = int(240 + 120 * math.sin(t * 0.9 + device_id * 2))
    cv2.circle(img, (cx, cy), 35, COLORS[device_id], cv2.FILLED)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert ok
    return buf.tobytes()


def render_artifact(device_id: int, t: float) -> bytes:
    # 模拟博物馆展柜背景：微暗展台与照明
    img = np.full((480, 640, 3), 35, np.uint8)

    # 展台底座
    cv2.ellipse(img, (320, 380), (160, 40), 0, 0, 360, (60, 60, 60), -1)
    cv2.ellipse(img, (320, 380), (160, 40), 0, 0, 360, (100, 100, 100), 2)

    # 动态演示逻辑：
    # 周期 30 秒：
    # 0~10s: 稳定竖立 (安全)
    # 10~18s: 缓慢倾斜 (姿态倒伏告警)
    # 18~24s: 尺寸增大 (移近镜头/被替换)
    # 24~30s: 恢复正常
    cycle = t % 30.0
    angle = 0.0
    scale = 1.0
    offset_x = 0.0

    if 10.0 <= cycle < 18.0:
        progress = (cycle - 10.0) / 8.0
        # 模拟倾斜，从 0 度倾斜到 18 度再稍微摇晃
        angle = 18.0 * math.sin(progress * math.pi)
    elif 18.0 <= cycle < 24.0:
        progress = (cycle - 18.0) / 6.0
        # 模拟尺寸变大 +28%
        scale = 1.0 + 0.28 * math.sin(progress * math.pi)
        offset_x = 15.0 * math.sin(progress * math.pi)

    # 视角差异：
    # 机位 1（正面）：最清楚看到左右倾角
    # 机位 2（侧面）：倾斜角度投影不同
    # 机位 3（斜俯视）：主要体现平面位置与顶部开口
    if device_id == 1:
        view_name = "CAM 1 (Front View / 正面机位)"
        view_angle = angle
    elif device_id == 2:
        view_name = "CAM 2 (Side View / 侧面机位)"
        view_angle = angle * 0.4  # 侧视投影倾角不同
    else:
        view_name = "CAM 3 (Top-Angle / 斜俯视)"
        view_angle = angle * 0.7
        scale *= 0.9

    # 绘制高对比度藏品（瓷瓶 / 艺术品模型）
    w, h = int(72 * scale), int(190 * scale)
    rect_pts = np.array([
        [-w / 2, -h / 2],
        [w / 2, -h / 2],
        [w / 2, h / 2],
        [-w / 2, h / 2]
    ], dtype=np.float32)

    theta = math.radians(view_angle)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    rot_pts = np.dot(rect_pts, R.T)

    tx = 320 + offset_x
    ty = 270
    rot_pts[:, 0] += tx
    rot_pts[:, 1] += ty
    cv2.fillPoly(img, np.int32([rot_pts]), (215, 215, 220))
    cv2.polylines(img, np.int32([rot_pts]), True, (150, 150, 160), 2)

    # 瓶颈
    nw, nh = int(26 * scale), int(38 * scale)
    neck_pts = np.array([
        [-nw / 2, -h / 2 - nh],
        [nw / 2, -h / 2 - nh],
        [nw / 2, -h / 2],
        [-nw / 2, -h / 2]
    ], dtype=np.float32)
    neck_rot = np.dot(neck_pts, R.T)
    neck_rot[:, 0] += tx
    neck_rot[:, 1] += ty
    cv2.fillPoly(img, np.int32([neck_rot]), (215, 215, 220))
    cv2.polylines(img, np.int32([neck_rot]), True, (150, 150, 160), 2)

    # 瓶口
    cv2.circle(img, (int(tx - (h / 2 + nh) * s), int(ty - (h / 2 + nh) * c)), int(14 * scale), (180, 180, 190), -1)

    # 水印与机位信息
    cv2.putText(img, view_name, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 230, 230), 2)
    state_str = "STATE: NORMAL (0-10s)" if cycle < 10.0 else \
                "STATE: TILTING (10-18s)" if cycle < 18.0 else \
                "STATE: SCALING (18-24s)" if cycle < 24.0 else "STATE: RECOVERING"
    cv2.putText(img, f"DEMO {state_str} t={cycle:.1f}s", (16, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 1)

    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    assert ok
    return buf.tobytes()


def make_handler(device_id: int, fps: float, mode: str):
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
                    t = time.monotonic() - start
                    if mode == "color":
                        jpeg = render_color_blob(device_id, t)
                    else:
                        jpeg = render_artifact(device_id, t)
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
                "udp_stream": UDP_BASE_PORT + device_id - 1,
            }
            sock.sendto(json.dumps(reply).encode(), addr)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=3, choices=[1, 2, 3])
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--mode", choices=["artifact", "color"], default="artifact",
                        help="artifact 为博物馆展品监测模式，color 为经典色块模式")
    parser.add_argument("--no-discovery", action="store_true")
    parser.add_argument("--bind", default="0.0.0.0",
                        help="HTTP 监听地址；上位机按发现回复的来源 IP 连接，所以默认监听所有网卡")
    args = parser.parse_args()

    ids = list(range(1, args.count + 1))
    for device_id in ids:
        server = ThreadingHTTPServer((args.bind, 8080 + device_id), make_handler(device_id, args.fps, args.mode))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        threading.Thread(target=udp_camera_loop, args=(device_id, args.bind, args.mode), daemon=True).start()
        print(f"[cam {device_id}] http://{args.bind}:{8080 + device_id}/stream ({args.mode} mode)")
        print(f"[cam {device_id}] udp://{args.bind}:{UDP_BASE_PORT + device_id - 1} (VGA12/QVGA25)")
    if not args.no_discovery:
        threading.Thread(target=discovery_loop, args=(ids,), daemon=True).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

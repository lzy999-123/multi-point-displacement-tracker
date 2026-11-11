import sys
import time
import socket
import struct
import threading
import serial

def main():
    print("[runner] 打开 COM5 维持运行电平 (dtr=False, rts=False)...")
    ser = serial.Serial('COM5', 115200, timeout=0.2)
    ser.dtr = False
    ser.rts = False

    ready_event = threading.Event()
    stop_flag = threading.Event()

    def reader():
        while not stop_flag.is_set():
            line = ser.readline()
            if line:
                try:
                    txt = line.decode('utf-8', errors='replace').strip()
                    if txt:
                        print(f"  [ESP32] {txt}")
                    if "待机监听中" in txt or "已连接 STA" in txt or "[diag]" in txt:
                        ready_event.set()
                except Exception:
                    pass

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    print("[runner] 探测 ESP32 状态...")
    # 先发送诊断探针 's'
    for _ in range(3):
        ser.write(b"s\n")
        if ready_event.wait(timeout=1.0):
            break

    if not ready_event.is_set():
        print("[runner] 未响应，触发硬件复位脉冲重连 Wi-Fi...")
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        print("[runner] 等待 ESP32 完成 Wi-Fi 关联与 UDP 服务就绪 (最长 30s)...")
        if not ready_event.wait(timeout=30.0):
            print("[runner] 超时未检测到 ESP32 就绪标志！")
            stop_flag.set()
            ser.close()
            sys.exit(1)

    print("\n[runner] >>> ESP32 已就绪！启动 15 秒 UDP VID1 视频流实测 <<<")
    time.sleep(1.0)

    host = "192.168.110.2"
    port = 5006
    local_ip = "192.168.110.156"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((local_ip, 0))
    sock.settimeout(0.05)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)

    test_duration = 15.0
    start_time = time.monotonic()
    deadline = start_time + test_duration
    last_vreq = 0.0

    class FrameSlot:
        def __init__(self):
            self.seq = None
            self.total_chunks = 0
            self.chunks = {}

    slots = [FrameSlot() for _ in range(16)]
    frames_received = []
    total_packets = 0
    magic_expected = 0x31444956  # 'VID1'

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_vreq >= 0.25:
            sock.sendto(b"VREQ", (host, port))
            last_vreq = now

        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue

        total_packets += 1
        if len(data) < 16:
            continue

        magic, dev_id, _, seq, chunk_idx, total_chunks, payload_sz, total_sz = struct.unpack_from(
            "<IBBHBBHI", data, 0
        )
        if magic != magic_expected:
            continue

        s = slots[seq % 16]
        if s.seq != seq:
            s.seq = seq
            s.total_chunks = total_chunks
            s.chunks = {}

        s.chunks[chunk_idx] = data[16 : 16 + payload_sz]
        if len(s.chunks) == s.total_chunks:
            frames_received.append((seq, time.monotonic()))
            s.chunks = {}

    elapsed = time.monotonic() - start_time
    sock.close()
    stop_flag.set()
    time.sleep(0.5)
    ser.close()

    total_frames = len(frames_received)
    avg_fps = total_frames / elapsed if elapsed > 0 else 0

    gaps = []
    for i in range(1, len(frames_received)):
        gaps.append((frames_received[i][1] - frames_received[i-1][1]) * 1000.0)
    max_gap = max(gaps) if gaps else 0.0

    print("\n" + "=" * 50)
    print("           VID1 UDP 局域网视频流评测报告")
    print("=" * 50)
    print(f"  测试时长:        {elapsed:.2f} 秒")
    print(f"  总接收 UDP 分片: {total_packets} 包")
    print(f"  完整解码帧数:    {total_frames} 帧")
    print(f"  平均帧率:        {avg_fps:.1f} FPS (目标: >= 20.0 FPS)")
    print(f"  最大帧间间隔:    {max_gap:.1f} ms (目标: <= 150 ms)")
    print("=" * 50)
    if avg_fps >= 20.0 and max_gap <= 150.0:
        print("  【PASS】 视频流达到工业级高顺畅低抖动验收标准！")
    else:
        print("  【REVIEW】 数据待优化或存在偶发掉帧。")
    print("=" * 50)

if __name__ == "__main__":
    main()

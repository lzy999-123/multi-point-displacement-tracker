#pragma once

// ===== 每台设备烧录前修改 =====
// 设备编号 1～3，对应 STM32 的 USART1～3 和上位机的通道 1～3。
#define DEVICE_ID 1

// ===== 开发板 =====
// 普通 ESP32-CAM（本次 ESP32-D0WD-V3 + OV3660）使用 AI-Thinker 引脚。
// S3 构建保留原 S3-EYE 引脚和 GC2145 软件编码路径。
#if defined(CONFIG_IDF_TARGET_ESP32)
#define BOARD_AI_THINKER
#endif

// ===== 视频参数 =====
// 手机热点下的清晰度升级：传感器原生640x480，目标仍为每路20fps。
#define CAM_FRAME_SIZE FRAMESIZE_VGA
// JPEG 质量 0～100（软件压缩）或 10～63（硬件压缩），见 displacement_cam.ino
// 实际 JPEG 大小随场景变化；帧率与延时必须通过实机验收。
#define CAM_SOFT_JPEG_QUALITY 20
#define CAM_HW_JPEG_QUALITY 18
#define CAM_FRAME_INTERVAL_MS 32
#if defined(BOARD_AI_THINKER)
#define CAM_XCLK_HZ 20000000
#else
#define CAM_XCLK_HZ 20000000
#endif
#define CAM_RAW_FB_COUNT 4
// 按安装方向翻转画面（0 或 1）
#if defined(BOARD_AI_THINKER)
#define CAM_VFLIP 1
#else
#define CAM_VFLIP 0
#endif
#define CAM_HMIRROR 0

// 功率与网络模式调优：
// 发射功率需用实际路由器链路验收，最大功率不保证更低丢包。
// 0: 纯 STA 模式（连接 secrets.h 指定的路由器，禁用内置 AP）
// 1: 纯 AP 模式（独立热点 disp-cam-X，无路由器时的点对点模式）
#define CAM_WIFI_MODE_AP 0
#define CAM_AP_CHANNEL 1
#define CAM_WIFI_TX_POWER WIFI_POWER_19_5dBm
// Wi-Fi 由 Arduino 库初始化，避免重复 esp_wifi_init 及未经验证的队列扩容。
// WPA2 compatibility comparison for the lab AP; no router settings are changed.
#define CAM_WIFI_PMF_CAPABLE 1
// Diagnostic: compare a locally administered station address with the factory MAC.
#define CAM_WIFI_TEST_LOCAL_MAC 0

// ===== 端口（与 docs/通信协议.md 一致）=====
#define STREAM_PORT 81
#define STREAM_UDP_PORT 5006
#define CAM_UDP_PAYLOAD_SIZE 1400
// 固定发送节拍：45ms ≈ 22fps。低于采集帧率(~27.8fps)，给空口留余量，换取稳定帧率。
#define CAM_UDP_FRAME_INTERVAL_MS 45
// 同一帧相邻分片之间的间隔，避免瞬时突发塞满 Wi-Fi 发送缓冲。
#define CAM_UDP_CHUNK_INTERVAL_MS 3
// AF41：IP 优先级4，对应 Wi-Fi WMM 视频接入类别。
#define CAM_UDP_IP_TOS 0x88
// 只对短暂的发送缓冲不足进行有限重试，所有等待均计入整帧预算。
#define CAM_UDP_PACKET_RETRIES 3
#define CAM_UDP_RETRY_INTERVAL_MS 2
// 超过次数或整帧预算后丢弃本帧并退避，让驱动排空队列。
#define CAM_UDP_BACKOFF_MS 30
#define CAM_UDP_BACKOFF_MAX_MS 200
// 单帧发送上限；超过预算即丢弃剩余片段，下一拍读取最新图像。
#define CAM_UDP_FRAME_BUDGET_MS 45
// 给TCP丢包重传保留时间，避免250ms超时触发反复断开/重连。
// 不是整帧延迟的硬上限；等待期间采集任务继续覆盖旧画面。
#define STREAM_SEND_TIMEOUT_MS 1000
// 避免 multipart 尾部小段触发 Nagle/延迟 ACK 等待。可用 ?nodelay=0 对照。
#define STREAM_TCP_NODELAY 1
#define CONTROL_PORT 5005
#define DISCOVERY_PORT 39093

// ===== WiFi =====
// 在同目录新建 secrets.h（参考 secrets.example.h），该文件不提交到仓库。
#if !__has_include("secrets.h")
#error "缺少 secrets.h：把 secrets.example.h 复制为 secrets.h，并填写 WiFi 名称和密码"
#endif
#include "secrets.h"

// ===== 预留静态 IP 参数（当前未启用，不据此保证推流性能）=====
#define CAM_USE_STATIC_IP 0
#define CAM_IP_ADDR  192, 168, 110, 43
#define CAM_IP_GW    192, 168, 110, 1
#define CAM_IP_MASK  255, 255, 255, 0
#define CAM_IP_DNS   192, 168, 110, 1

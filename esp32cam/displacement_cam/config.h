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
// 正式档位：320x240、640x480、800x600均不设置固定发送帧率上限。
// 上位机通过带代次的档位请求切换，实际完整帧与显示帧须分别验收。
// Allocate JPEG capture buffers for the largest supported runtime resolution.
#define CAM_FRAME_SIZE FRAMESIZE_SVGA
// JPEG 质量 0～100（软件压缩）或 10～63（硬件压缩），见 displacement_cam.ino
// 实际 JPEG 大小随场景变化；帧率与延时必须通过实机验收。
#define CAM_SOFT_JPEG_QUALITY 20
#define CAM_HW_JPEG_QUALITY 32
#define CAM_UDP_JPEG_TARGET_BYTES 2600
#define CAM_UDP_JPEG_MIN_QUALITY 24
#define CAM_UDP_JPEG_MAX_QUALITY 42
#if defined(BOARD_AI_THINKER)
// User-selected 24MHz XCLK. OV3660's matching PLL is applied by captureTask
// after every resolution change to retain the original JPEG pixel clock.
#define CAM_XCLK_HZ 24000000
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
// 初始QVGA发送节拍；收到档位请求后由对应档位覆盖。
#define CAM_UDP_FRAME_INTERVAL_MS 0
// 使用标准 IP 优先级；热点上的视频队列优先级须经实测后再启用。
#define CAM_UDP_IP_TOS 0x00
// 整帧发送预算包含分片节拍和短暂缓冲不足重试；超时仍丢弃旧帧。
// udp_send_policy.h 按帧大小限制分片等待，连续失败后冷却50/100/150ms。
// 已开始发送的帧尽量发完：发到一半丢弃的分片同样占用空口，链路慢时会让
// 每帧都残缺、帧率归零。上限须低于上位机组帧槽寿命 500ms（udpstream.cpp
// kSlotLifetimeMs），留出驱动队列排空余量。链路好时整帧仍在几十毫秒内发完。
#define CAM_UDP_FRAME_BUDGET_MS 350
// VREQA反馈窗口初始1帧，30次确认后升至2帧；发送缓冲不足回到1帧。
#define CAM_UDP_ACK_TIMEOUT_MS 300
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

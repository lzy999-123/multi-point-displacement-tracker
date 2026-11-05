#pragma once

// ===== 每台设备烧录前修改 =====
// 设备编号 1～3，对应 STM32 的 USART1～3 和上位机的通道 1～3。
#define DEVICE_ID 1

// ===== 视频参数 =====
// FRAMESIZE_QVGA(320x240) / FRAMESIZE_VGA(640x480) / FRAMESIZE_SVGA(800x600)
#define CAM_FRAME_SIZE FRAMESIZE_VGA
// JPEG 质量 10～63，数值越小画质越好、每帧越大
#define CAM_JPEG_QUALITY 12
// 按安装方向翻转画面（0 或 1）
#define CAM_VFLIP 0
#define CAM_HMIRROR 0

// ===== 端口（与 docs/通信协议.md 一致）=====
#define STREAM_PORT 81
#define CONTROL_PORT 5005
#define DISCOVERY_PORT 39093

// ===== WiFi =====
// 在同目录新建 secrets.h（参考 secrets.example.h），该文件不提交到仓库。
#include "secrets.h"

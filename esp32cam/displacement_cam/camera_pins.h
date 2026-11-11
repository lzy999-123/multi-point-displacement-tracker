#pragma once

// 摄像头引脚。普通 ESP32-CAM 使用下方 AI-Thinker 映射（包括 OV3660）。
// 旧开发板（ESP32-S3，8MB PSRAM，GC2145）已用 esp32cam/camera_probe 实测：
// 按 ESP32-S3-EYE 引脚初始化成功，读到传感器 PID 0x2145。
// config.h 根据 CONFIG_IDF_TARGET_ESP32 自动选择 BOARD_AI_THINKER。

#if defined(BOARD_AI_THINKER)

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define FLASH_LED_GPIO_NUM 4  // 板载闪光灯，上电时拉低

#else  // ESP32-S3-EYE 引脚（与 Arduino CameraWebServer 示例 CAMERA_MODEL_ESP32S3_EYE 相同）

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "当前引脚是 ESP32-S3 的，请把开发板选为 ESP32S3 Dev Module，或在 config.h 定义 BOARD_AI_THINKER"
#endif

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13
// 没有闪光灯。注意 GPIO4 是摄像头 SCCB 数据线，不能当普通 IO 用。

#endif

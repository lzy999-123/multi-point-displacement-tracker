// 摄像头引脚探测程序：按 ESP32-S3-EYE 的引脚初始化摄像头，打印传感器型号、取帧和软件 JPEG 耗时。
// 只用串口输出，不连 WiFi。结果用于确认开发板的摄像头引脚，然后再写正式固件。
//
// 编译：esp32:esp32:esp32s3:PSRAM=opi,FlashSize=8M

#include "esp_camera.h"
#include "img_converters.h"

// 乐鑫 ESP32-S3-EYE（与 Arduino CameraWebServer 示例 CAMERA_MODEL_ESP32S3_EYE 相同）
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

static bool g_ready = false;

static esp_err_t initCamera(int xclkHz) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = xclkHz;
  c.pixel_format = PIXFORMAT_RGB565;  // GC2145 没有硬件 JPEG
  c.frame_size = FRAMESIZE_QVGA;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c);
}

static void measure(framesize_t size, const char *name) {
  sensor_t *s = esp_camera_sensor_get();
  if (s->set_framesize(s, size) != 0) {
    Serial.printf("[probe] %s: 设置分辨率失败\n", name);
    return;
  }
  // 丢掉切换分辨率后的头几帧
  for (int i = 0; i < 3; ++i) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }

  const int n = 10;
  uint32_t grabUs = 0, jpegUs = 0, jpegBytes = 0;
  int ok = 0, w = 0, h = 0;
  for (int i = 0; i < n; ++i) {
    uint32_t t0 = micros();
    camera_fb_t *fb = esp_camera_fb_get();
    uint32_t t1 = micros();
    if (!fb) continue;
    w = fb->width;
    h = fb->height;
    uint8_t *jpg = nullptr;
    size_t jpgLen = 0;
    bool converted = frame2jpg(fb, 12, &jpg, &jpgLen);
    uint32_t t2 = micros();
    esp_camera_fb_return(fb);
    if (!converted) continue;
    free(jpg);
    grabUs += t1 - t0;
    jpegUs += t2 - t1;
    jpegBytes += jpgLen;
    ++ok;
  }
  if (ok == 0) {
    Serial.printf("[probe] %s: 取帧失败\n", name);
    return;
  }
  const float perFrameMs = (grabUs + jpegUs) / 1000.0f / ok;
  Serial.printf("[probe] %s %dx%d: 取帧 %.1f ms, 软件JPEG %.1f ms, 平均 %u 字节, 约 %.1f fps (%d/%d 帧成功)\n",
                name, w, h, grabUs / 1000.0f / ok, jpegUs / 1000.0f / ok, jpegBytes / ok,
                1000.0f / perFrameMs, ok, n);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[probe] ===== 摄像头引脚探测（ESP32-S3-EYE 引脚）=====");
  Serial.printf("[probe] PSRAM: %u 字节\n", ESP.getPsramSize());

  const int xclks[] = {20000000, 10000000};
  esp_err_t err = ESP_FAIL;
  for (int xclk : xclks) {
    err = initCamera(xclk);
    Serial.printf("[probe] XCLK %d MHz: esp_camera_init = 0x%x (%s)\n", xclk / 1000000, err, esp_err_to_name(err));
    if (err == ESP_OK) break;
    esp_camera_deinit();
  }
  if (err != ESP_OK) {
    Serial.println("[probe] 结果：失败。引脚可能不对，或摄像头排线没插好。");
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("[probe] 传感器 PID=0x%04x（GC2145=0x2145，OV2640=0x26，OV5640=0x5640）\n", s->id.PID);
  measure(FRAMESIZE_QVGA, "QVGA");
  measure(FRAMESIZE_VGA, "VGA");
  Serial.println("[probe] 结果：成功。");
  g_ready = true;
}

void loop() {
  // 每 3 秒打印一次，方便复位后错过开头时再读到
  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    Serial.printf("[probe] 运行中，摄像头%s\n", g_ready ? "正常" : "初始化失败");
  }
  delay(10);
}

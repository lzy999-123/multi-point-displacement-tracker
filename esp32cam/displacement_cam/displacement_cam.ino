// ESP32-CAM 固件（阶段 1）：WiFi 连接、HTTP MJPEG 推流、UDP 设备发现。
// 控制通道（UDP 5005 ↔ 串口）在阶段 2 加入。协议见 docs/通信协议.md。
//
// 开发板：AI Thinker ESP32-CAM（Arduino ESP32 核心 2.x 或 3.x）
// 分区方案：Huge APP 或默认均可；需要开启 PSRAM。

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include "esp_http_server.h"

#include "camera_pins.h"
#include "config.h"

#if DEVICE_ID < 1 || DEVICE_ID > 3
#error "DEVICE_ID 必须是 1～3"
#endif

#define PART_BOUNDARY "frame"
static const char *kStreamContentType = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *kStreamBoundary = "\r\n--" PART_BOUNDARY "\r\n";
static const char *kStreamPartHeader = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t streamServer = nullptr;
static WiFiUDP discoveryUdp;

// ---------- 摄像头 ----------

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = CAM_FRAME_SIZE;
  config.jpeg_quality = CAM_JPEG_QUALITY;

  if (psramFound()) {
    // 两个缓冲区 + 只取最新帧：网络慢时丢旧帧，不累积延迟
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("[cam] 未检测到 PSRAM，降为 QVGA 单缓冲");
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[cam] 初始化失败 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, CAM_VFLIP);
  s->set_hmirror(s, CAM_HMIRROR);
  return true;
}

// ---------- MJPEG 推流 ----------

static esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, kStreamContentType);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  Serial.println("[stream] 客户端已连接");
  char partHeader[64];
  uint32_t frames = 0;
  uint32_t windowStart = millis();

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[stream] 取帧失败");
      res = ESP_FAIL;
      break;
    }

    size_t headerLen = snprintf(partHeader, sizeof(partHeader), kStreamPartHeader,
                                static_cast<unsigned>(fb->len));
    res = httpd_resp_send_chunk(req, kStreamBoundary, strlen(kStreamBoundary));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, partHeader, headerLen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, reinterpret_cast<const char *>(fb->buf), fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;  // 客户端断开

    ++frames;
    uint32_t now = millis();
    if (now - windowStart >= 5000) {
      Serial.printf("[stream] %.1f fps\n", frames * 1000.0f / (now - windowStart));
      frames = 0;
      windowStart = now;
    }
  }

  Serial.println("[stream] 客户端断开");
  return res;
}

static void startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = STREAM_PORT;
  config.ctrl_port = STREAM_PORT + 32768;  // 与默认 32768 错开
  config.max_open_sockets = 2;

  httpd_uri_t streamUri = {};
  streamUri.uri = "/stream";
  streamUri.method = HTTP_GET;
  streamUri.handler = streamHandler;

  if (httpd_start(&streamServer, &config) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &streamUri);
    Serial.printf("[stream] http://%s:%d/stream\n", WiFi.localIP().toString().c_str(), STREAM_PORT);
  } else {
    Serial.println("[stream] HTTP 服务启动失败");
  }
}

// ---------- 设备发现 ----------

// 从简单 JSON 文本中取出字符串字段，只接受字母、数字、'-'、'_'，长度不超过 maxLen。
static bool extractToken(const char *json, const char *key, char *out, size_t maxLen) {
  char pattern[24];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char *p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p + strlen(pattern), ':');
  if (!p) return false;
  p = strchr(p, '"');
  if (!p) return false;
  ++p;
  size_t n = 0;
  while (*p && *p != '"') {
    char c = *p++;
    bool ok = isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_';
    if (!ok || n + 1 >= maxLen) return false;
    out[n++] = c;
  }
  if (*p != '"' || n == 0) return false;
  out[n] = '\0';
  return true;
}

static void pollDiscovery() {
  int size = discoveryUdp.parsePacket();
  if (size <= 0) return;

  char buf[256];
  int len = discoveryUdp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = '\0';

  char type[16];
  char nonce[40];
  if (!extractToken(buf, "type", type, sizeof(type)) || strcmp(type, "discover") != 0) return;
  if (!extractToken(buf, "nonce", nonce, sizeof(nonce))) return;

  String ip = WiFi.localIP().toString();
  char reply[256];
  int n = snprintf(reply, sizeof(reply),
                   "{\"type\":\"announce\",\"version\":1,\"nonce\":\"%s\",\"id\":%d,"
                   "\"ip\":\"%s\",\"stream\":\"http://%s:%d/stream\",\"control_port\":%d}",
                   nonce, DEVICE_ID, ip.c_str(), ip.c_str(), STREAM_PORT, CONTROL_PORT);
  if (n <= 0 || n >= static_cast<int>(sizeof(reply))) return;

  discoveryUdp.beginPacket(discoveryUdp.remoteIP(), discoveryUdp.remotePort());
  discoveryUdp.write(reinterpret_cast<const uint8_t *>(reply), n);
  discoveryUdp.endPacket();
}

// ---------- WiFi ----------

static void connectWifi() {
  char hostname[24];
  snprintf(hostname, sizeof(hostname), "disp-cam-%d", DEVICE_ID);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  WiFi.setSleep(false);  // 关闭省电，降低视频延迟
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[wifi] 连接 %s", WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - start > 20000) {
      Serial.println("\n[wifi] 20s 未连上，重启");
      ESP.restart();
    }
  }
  Serial.printf("\n[wifi] 已连接，IP %s，RSSI %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

// ---------- 主程序 ----------

void setup() {
  Serial.begin(115200);
  Serial.printf("\n[boot] 位移跟踪相机 #%d\n", DEVICE_ID);

  pinMode(FLASH_LED_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_LED_GPIO_NUM, LOW);

  if (!initCamera()) {
    delay(3000);
    ESP.restart();
  }
  connectWifi();
  startStreamServer();
  discoveryUdp.begin(DISCOVERY_PORT);
  Serial.printf("[discovery] 监听 UDP %d\n", DISCOVERY_PORT);
}

void loop() {
  // 推流在 HTTP 服务器自己的任务里运行，这里只处理发现请求
  pollDiscovery();

  static uint32_t lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[wifi] 断开，等待自动重连");
    }
  }
  delay(5);
}

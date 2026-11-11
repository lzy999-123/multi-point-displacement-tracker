// ESP32-CAM 固件（阶段 1）：WiFi 连接、HTTP MJPEG 推流、UDP 设备发现。
// 控制通道（UDP 5005 ↔ 串口）在阶段 2 加入。协议见 docs/通信协议.md。
//
// 开发板：AI Thinker ESP32-CAM（Arduino ESP32 核心 2.x 或 3.x）
// 分区方案：Huge APP 或默认均可；需要开启 PSRAM。

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "esp_wifi.h"

#include "config.h"
#include "camera_pins.h"

#if DEVICE_ID < 1 || DEVICE_ID > 3
#error "DEVICE_ID 必须是 1～3"
#endif

#define PART_BOUNDARY "frame"
static const char *kStreamContentType = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *kStreamBoundary = "\r\n--" PART_BOUNDARY "\r\n";
static const char *kStreamPartHeader = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Frame-Seq: %u\r\n\r\n";

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
  config.xclk_freq_hz = 8000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = CAM_FRAME_SIZE;
  config.jpeg_quality = CAM_JPEG_QUALITY;

  if (psramFound()) {
    // 两个缓冲区 + 只取最新帧：网络慢时丢旧帧，不累积延迟
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
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
  if (s->id.PID == OV3660_PID) {
    // Official OV3660 clock formula: 8MHz*25 gives the same 200MHz PLL
    // as 20MHz*30/prediv3. SYSCLK=50MHz, PCLK=10MHz; verify on hardware.
    int result = s->set_pll(s, 0, 25, 1, 0, 0, 0, 1, 10);
    Serial.printf("[clock-test] XCLK=8000000 pll-result=%d regs=%x,%x,%x,%x pclk=%x\n",result,
       s->get_reg(s,0x303a,255),s->get_reg(s,0x303b,255),s->get_reg(s,0x303c,255),
       s->get_reg(s,0x303d,255),s->get_reg(s,0x3824,255));
    if (result != 0) return false;
  }

  Serial.printf("[cam] PID=0x%04x PSRAM=%u single-buffer JPEG\n",s->id.PID,ESP.getPsramSize());
  return true;
}

// ---------- MJPEG 推流 ----------

static esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, kStreamContentType);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  int sock = httpd_req_to_sockfd(req);
  int nodelay = 1;
  setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
  timeval timeout = {1,0};
  setsockopt(sock,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  Serial.println("[stream] 客户端已连接");
  char partHeader[128];
  uint8_t *packet=nullptr; size_t packetCap=0; uint32_t frameSequence=0;
  uint32_t frames = 0;
  uint32_t windowStart = millis();

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[stream] 取帧失败");
      res = ESP_FAIL;
      break;
    }

    size_t headerLen = snprintf(partHeader,sizeof(partHeader),kStreamPartHeader,
                                static_cast<unsigned>(fb->len),++frameSequence);
    const size_t boundaryLen=strlen(kStreamBoundary);
    const size_t total=boundaryLen+headerLen+fb->len;
    if (total>packetCap) {
      free(packet);packet=static_cast<uint8_t *>(ps_malloc(total));packetCap=packet?total:0;
    }
    if (!packet) { esp_camera_fb_return(fb);res=ESP_ERR_NO_MEM;break; }
    memcpy(packet,kStreamBoundary,boundaryLen);
    memcpy(packet+boundaryLen,partHeader,headerLen);
    memcpy(packet+boundaryLen+headerLen,fb->buf,fb->len);
    esp_camera_fb_return(fb);
    res=httpd_resp_send_chunk(req,reinterpret_cast<char *>(packet),total);
    if (res!=ESP_OK) { Serial.printf("[http] send failed result=%x errno=%d\n",res,errno);break; }
    vTaskDelay(pdMS_TO_TICKS(1));

    ++frames;
    uint32_t now = millis();
    if (now - windowStart >= 5000) {
      Serial.printf("[stream] %.1f fps\n", frames * 1000.0f / (now - windowStart));
      frames = 0;
      windowStart = now;
    }
  }

  free(packet);
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
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) Serial.println("[compat-event] associated");
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
      Serial.printf("[compat-event] disconnected reason=%d\n", info.wifi_sta_disconnected.reason);
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP)
      Serial.printf("[compat-event] IP=%s\n", IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
  });
  int count = WiFi.scanNetworks();
  int selected = -1;
  for (int i = 0; i < count; ++i) {
    if (WiFi.SSID(i) == WIFI_SSID && (selected < 0 || WiFi.RSSI(i) > WiFi.RSSI(selected))) selected = i;
  }
  uint8_t bssid[6]{};
  int channel = 0;
  if (selected >= 0) {
    memcpy(bssid, WiFi.BSSID(selected), sizeof(bssid));
    channel = WiFi.channel(selected);
    Serial.printf("[compat] target BSSID=%s channel=%d RSSI=%d auth=%d MAC=%s\n",
                  WiFi.BSSIDstr(selected).c_str(), channel, WiFi.RSSI(selected),
                  int(WiFi.encryptionType(selected)), WiFi.macAddress().c_str());
  } else Serial.println("[compat] target not in scan; trying saved SSID");
  WiFi.scanDelete();
  for (int phase = 0; phase < 2 && WiFi.status() != WL_CONNECTED; ++phase) {
    WiFi.disconnect(false, false); delay(200);
    const uint8_t protocol = phase == 0 ? WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N
                                        : WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
    Serial.printf("[compat] phase=%d protocol-result=%d\n", phase, esp_wifi_set_protocol(WIFI_IF_STA, protocol));
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, channel, selected >= 0 ? bssid : nullptr, false);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    wifi_config_t station{};
    if (esp_wifi_get_config(WIFI_IF_STA, &station) == ESP_OK) {
      station.sta.pmf_cfg.capable = false;
      station.sta.pmf_cfg.required = false;
      Serial.printf("[compat] PMF-off config-result=%d\n", esp_wifi_set_config(WIFI_IF_STA, &station));
    }
    esp_wifi_connect();
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 35000) delay(100);
    Serial.printf("[compat] phase=%d status=%d elapsed=%u power=%d\n", phase,
                  int(WiFi.status()), millis() - start, int(WiFi.getTxPower()));
  }

  if (WiFi.status()!=WL_CONNECTED) { Serial.println("[wifi] no IP; reboot in 30s"); delay(30000);ESP.restart(); }
  WiFi.setAutoReconnect(true);
  Serial.printf("[wifi] ready IP=%s RSSI=%d BSSID=%s\n",WiFi.localIP().toString().c_str(),WiFi.RSSI(),WiFi.BSSIDstr().c_str());
}

// ---------- 主程序 ----------

void setup() {
  Serial.begin(115200);
  Serial.printf("\n[boot] 位移跟踪相机 #%d\n", DEVICE_ID);

  pinMode(FLASH_LED_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_LED_GPIO_NUM, LOW);

  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH);
  connectWifi();
  if (!initCamera()) {
    delay(3000);
    ESP.restart();
  }
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

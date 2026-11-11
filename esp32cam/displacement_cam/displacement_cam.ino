// ESP32-CAM 固件：无线视频推流，性能以真实接收与显示验收为准。
// 支持轻量 VID1 UDP 视频流与标准 HTTP MJPEG。
// 普通 ESP32-CAM (ESP32-D0WD-V3 + OV3660) 采用硬件 JPEG + 零拷贝 UDP 直出。

#include <WiFi.h>
#include <WiFiUdp.h>
#include <atomic>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp_jpeg_enc.h"
#include "soc/lcd_cam_struct.h"
#endif
#include "lwip/sockets.h"
#include "lwip/etharp.h"
#include <fcntl.h>
#include "esp_wifi.h"

#pragma pack(push, 1)
struct VidPacketHeader {
  uint32_t magic;         // 0x31444956 ("VID1")
  uint8_t  device_id;     // 1~3
  uint8_t  reserved;      // 0
  uint16_t frame_seq;     // 0~65535
  uint8_t  chunk_index;   // 0 .. chunk_count-1
  uint8_t  chunk_count;   // 该帧总分片数
  uint16_t payload_size;  // 该包承载的有效 JPEG 数据大小
  uint32_t frame_size;    // 完整 JPEG 图像总大小
};
#pragma pack(pop)

#include "config.h"
#include "camera_pins.h"

#if DEVICE_ID < 1 || DEVICE_ID > 3
#error "DEVICE_ID 必须是 1～3"
#endif

#define PART_BOUNDARY "frame"
static const char *kStreamContentType = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *kStreamBoundary = "\r\n--" PART_BOUNDARY "\r\n";
static const char *kStreamPartHeader = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Frame-Seq: %u\r\nX-Capture-Ms: %u\r\nX-Frame-Age-Ms: %u\r\n\r\n";
static const uint32_t kStreamStallMs = 2000;

static httpd_handle_t streamServer = nullptr;
static WiFiUDP discoveryUdp;

// 上位机推流目标地址与状态（Core 1 通过 UDP 5006 接收心跳并极速直出推流）
static sockaddr_in g_peer = {};
static std::atomic<bool> g_hasPeer{false};
static std::atomic<uint32_t> g_lastHeartbeat{0};
static std::atomic<uint32_t> g_lastWifiEvent{0};

// ---------- 最新 JPEG 副本（仅供 HTTP 兼容客户端读取）----------

static SemaphoreHandle_t g_jpegMutex = nullptr;
static uint8_t *g_jpegBuf = nullptr;
static size_t g_jpegLen = 0;
static uint32_t g_captureMs = 0;
static std::atomic<uint32_t> g_jpegSeq{0};
static std::atomic<int> g_clients{0};
static std::atomic<int> g_udpClients{0};
static bool g_hardwareJpeg = false;

static void publishJpeg(uint8_t *jpg, size_t len, uint32_t captureMs) {
  xSemaphoreTake(g_jpegMutex, portMAX_DELAY);
  uint8_t *old = g_jpegBuf;
  g_jpegBuf = jpg;
  g_jpegLen = len;
  g_captureMs = captureMs;
  g_jpegSeq.fetch_add(1);
  xSemaphoreGive(g_jpegMutex);
  free(old);
}

// ---------- 摄像头初始化 ----------

static esp_err_t initCamera(pixformat_t format) {
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
  config.xclk_freq_hz = CAM_XCLK_HZ;
  config.pixel_format = format;
  config.frame_size = CAM_FRAME_SIZE;
  config.jpeg_quality = CAM_HW_JPEG_QUALITY;
  // 双缓冲区 + CAMERA_GRAB_LATEST：DMA持续循环写入PSRAM，永远获取最新完成曝光的帧
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&config);
}

static bool startCamera() {
  if (!psramFound()) {
    Serial.println("[cam] 未检测到 PSRAM");
    return false;
  }
#if defined(BOARD_AI_THINKER)
  esp_err_t err = initCamera(PIXFORMAT_JPEG);
#else
  esp_err_t err = initCamera(PIXFORMAT_YUV422);
#endif
  if (err != ESP_OK) {
    Serial.printf("[cam] 初始化失败 0x%x (%s)\n", err, esp_err_to_name(err));
    return false;
  }
  const uint16_t pid = esp_camera_sensor_get()->id.PID;
#if defined(BOARD_AI_THINKER)
  g_hardwareJpeg = true;
#else
  if (pid == OV2640_PID || pid == OV3660_PID || pid == OV5640_PID) {
    esp_camera_deinit();
    err = initCamera(PIXFORMAT_JPEG);
    if (err != ESP_OK) {
      Serial.printf("[cam] JPEG 模式初始化失败 0x%x\n", err);
      return false;
    }
    g_hardwareJpeg = true;
  }
#endif

  sensor_t *s = esp_camera_sensor_get();
  if (pid == OV3660_PID) {
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  if (s->set_vflip) s->set_vflip(s, CAM_VFLIP);
  if (s->set_hmirror) s->set_hmirror(s, CAM_HMIRROR);
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (pid == 0x2145) LCD_CAM.cam_ctrl1.cam_clk_inv = 1;
#endif
  Serial.printf("[cam] 传感器 PID=0x%04x，%s，PSRAM=%u，XCLK=%d\n",
                pid, g_hardwareJpeg ? "硬件 JPEG" : "软件 JPEG",
                static_cast<unsigned>(ESP.getPsramSize()), CAM_XCLK_HZ);
  return true;
}

static bool fastJpeg(camera_fb_t *fb, uint8_t **jpg, size_t *len) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  static jpeg_enc_handle_t encoder = nullptr;
  static uint8_t *output = nullptr;
  static uint8_t *input = nullptr;
  static int capacity = 0;
  if (!encoder) {
    jpeg_enc_config_t config = DEFAULT_JPEG_ENC_CONFIG();
    config.width = fb->width; config.height = fb->height;
    config.src_type = JPEG_PIXEL_FORMAT_YCbYCr;
    config.subsampling = JPEG_SUBSAMPLE_420;
    config.quality = CAM_SOFT_JPEG_QUALITY;
    if (jpeg_enc_open(&config, &encoder) != JPEG_ERR_OK) return false;
    capacity = fb->len + 1024;
    output = (uint8_t *)jpeg_calloc_align(capacity, 16);
    if (!output) return false;
  }
  if (!output) return false;
  const uint8_t *source = fb->buf;
  if ((uintptr_t)source & 15U) {
    if (!input) input = (uint8_t *)jpeg_calloc_align(fb->len, 16);
    if (!input) return false;
    memcpy(input, source, fb->len); source = input;
  }
  int encoded = 0;
  jpeg_error_t err = jpeg_enc_process(encoder, source, fb->len, output, capacity, &encoded);
  if (err != JPEG_ERR_OK || encoded <= 0) return false;
  *jpg = (uint8_t *)ps_malloc(encoded);
  if (!*jpg) return false;
  memcpy(*jpg, output, encoded); *len = encoded; return true;
#else
  return frame2jpg(fb, CAM_SOFT_JPEG_QUALITY, jpg, len);
#endif
}

// ---------- 采集任务 (Core 1：硬件 DMA 极速取帧) ----------

static void captureTask(void *) {
  uint32_t frames = 0;
  uint32_t windowStart = millis();

  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    const int httpClients = g_clients.load();
    const int udpClients = g_udpClients.load();
    if (httpClients == 0 && udpClients == 0) {
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    uint8_t *jpg = nullptr;
    size_t len = 0;
    if (g_hardwareJpeg) {
      jpg = static_cast<uint8_t *>(ps_malloc(fb->len));
      if (jpg) {
        memcpy(jpg, fb->buf, fb->len);
        len = fb->len;
      }
    } else if (!fastJpeg(fb, &jpg, &len)) {
      jpg = nullptr;
    }
    const uint32_t captureMs = uint32_t(uint64_t(fb->timestamp.tv_sec) * 1000 + fb->timestamp.tv_usec / 1000);
    esp_camera_fb_return(fb);

    if (jpg && len > 0) {
      publishJpeg(jpg, len, captureMs);
      ++frames;
    }

    const uint32_t now = millis();
    if (now - windowStart >= 5000) {
      Serial.printf("[cam] 采集帧率: %.1f fps (内部RAM %u 字节)\n",
                    frames * 1000.0f / (now - windowStart),
                    static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
      frames = 0;
      windowStart = now;
    }
  }
}

// ---------- UDP 视频流任务 (Core 0：与 Wi-Fi / lwIP 同核运行) ----------

static void udpVideoTask(void *) {
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    Serial.println("[udp-stream] socket 失败");
    vTaskDelete(nullptr);
    return;
  }

  sockaddr_in local = {};
  local.sin_len = sizeof(local);
  local.sin_family = AF_INET;
  local.sin_port = htons(STREAM_UDP_PORT);
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
    Serial.printf("[udp-stream] bind 端口 %d 失败 errno=%d\n", STREAM_UDP_PORT, errno);
    close(fd);
    vTaskDelete(nullptr);
    return;
  }
  Serial.printf("[udp-stream] 监听端口 %d (Core 0 运行，与 Wi-Fi 协议栈同核)\n", STREAM_UDP_PORT);

  sockaddr_in peer = {};
  bool hasPeer = false;
  uint32_t lastHeartbeat = 0;
  uint32_t lastSeq = 0;
  constexpr size_t kMaxChunkPayload = CAM_UDP_PAYLOAD_SIZE;
  constexpr size_t kHeaderSize = sizeof(VidPacketHeader);
  static uint8_t packetBuf[kHeaderSize + kMaxChunkPayload];
  static uint8_t frameBuf[32768];

  uint32_t framesSent = 0, chunksSent = 0, sendFailures = 0;
  uint32_t windowStart = millis();

  for (;;) {
    // 1. 非阻塞轮询上位机保活心跳 ("VREQ" / "VPIN")，吞吐并清除失效端口残留的 ICMP 错误
    sockaddr_in incoming = {};
    socklen_t inLen = sizeof(incoming);
    static char reqBuf[256];
    int rlen = 0;
    while (true) {
      inLen = sizeof(incoming);
      rlen = recvfrom(fd, reqBuf, sizeof(reqBuf), MSG_DONTWAIT,
                      reinterpret_cast<sockaddr *>(&incoming), &inLen);
      if (rlen < 0) {
        if (errno == ECONNRESET || errno == ECONNREFUSED) {
          continue; // 清除上个会话遗留的 ICMP 端口不可达，继续排空队列收取新上位机心跳
        }
        break; // EAGAIN / EWOULDBLOCK，队列已空
      }
      if (rlen >= 4 && memcmp(reqBuf, "VPIN", 4) == 0) {
        memcpy(reqBuf, "VPON", 4);
        incoming.sin_family = AF_INET;
        incoming.sin_len = sizeof(incoming);
        sendto(fd, reqBuf, rlen, 0, reinterpret_cast<sockaddr *>(&incoming), sizeof(incoming));
        continue;
      }
      if (rlen >= 4 && memcmp(reqBuf, "VREQ", 4) == 0) {
        incoming.sin_len = sizeof(incoming);
        incoming.sin_family = AF_INET;
        peer = incoming;
        if (!hasPeer) {
          Serial.printf("[udp] 上位机接入: %s:%d\n",
                        inet_ntoa(peer.sin_addr), ntohs(peer.sin_port));
        }
        hasPeer = true;
        lastHeartbeat = millis();
        g_udpClients.store(1);
      }
    }

    if (!hasPeer || (millis() - lastHeartbeat > 10000)) {
      g_udpClients.store(0);
      hasPeer = false;
      vTaskDelay(pdMS_TO_TICKS(15));
      continue;
    }

    // 2. 等待新帧（最新帧覆盖机制）
    const uint32_t currentSeq = g_jpegSeq.load();
    if (currentSeq == lastSeq) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    size_t frameLen = 0;
    xSemaphoreTake(g_jpegMutex, portMAX_DELAY);
    frameLen = g_jpegLen;
    if (frameLen > 0 && frameLen <= sizeof(frameBuf)) {
      memcpy(frameBuf, g_jpegBuf, frameLen);
    } else {
      frameLen = 0;
    }
    xSemaphoreGive(g_jpegMutex);
    lastSeq = currentSeq;

    if (frameLen == 0) continue;

    // 3. 构造 VID1 数据报分片发送
    const uint8_t chunkCount = static_cast<uint8_t>((frameLen + kMaxChunkPayload - 1) / kMaxChunkPayload);
    VidPacketHeader *hdr = reinterpret_cast<VidPacketHeader *>(packetBuf);
    hdr->magic = 0x31444956;  // 'VID1'
    hdr->device_id = DEVICE_ID;
    hdr->reserved = 0;
    hdr->frame_seq = static_cast<uint16_t>(currentSeq & 0xFFFF);
    hdr->chunk_count = chunkCount;
    hdr->frame_size = static_cast<uint32_t>(frameLen);

    bool frameOk = true;
    for (uint8_t c = 0; c < chunkCount; ++c) {
      const size_t offset = c * kMaxChunkPayload;
      const size_t chunkBytes = (c == chunkCount - 1) ? (frameLen - offset) : kMaxChunkPayload;

      hdr->chunk_index = c;
      hdr->payload_size = static_cast<uint16_t>(chunkBytes);
      memcpy(packetBuf + kHeaderSize, frameBuf + offset, chunkBytes);

      const size_t totalBytes = kHeaderSize + chunkBytes;
      int sent = -1;
      for (int retry = 0; retry < 8; ++retry) {
        sent = sendto(fd, packetBuf, totalBytes, 0,
                      reinterpret_cast<sockaddr *>(&peer), sizeof(peer));
        if (sent == static_cast<int>(totalBytes)) break;
        vTaskDelay(pdMS_TO_TICKS(3));
      }

      if (sent == static_cast<int>(totalBytes)) {
        ++chunksSent;
      } else {
        frameOk = false;
        ++sendFailures;
        if (sendFailures <= 10) {
          Serial.printf("[udp-err] sendto chunk=%d/%d ret=%d errno=%d\n", c, chunkCount, sent, errno);
        }
        break; // 立即放弃残缺帧剩余分片，杜绝把破损包推入信道
      }

      if (c + 1 < chunkCount) {
        vTaskDelay(pdMS_TO_TICKS(4)); // 分片平滑间隔 4ms
      }
    }

    if (frameOk) {
      ++framesSent;
    }

    const uint32_t now = millis();
    if (now - windowStart >= 5000) {
      Serial.printf("[udp-stream] UDP 推流 %.1f fps (分片 %u, 失败 %u, 内部RAM %u 字节)\n",
                    framesSent * 1000.0f / (now - windowStart), chunksSent, sendFailures,
                    static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
      framesSent = chunksSent = sendFailures = 0;
      windowStart = now;
    }
  }
}

// ---------- HTTP MJPEG 推流与快照 ----------

static esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, kStreamContentType);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  const int sock = httpd_req_to_sockfd(req);
  int noDelay = 1;
  setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay));
  const struct timeval sendTimeout = {STREAM_SEND_TIMEOUT_MS / 1000,
                                     (STREAM_SEND_TIMEOUT_MS % 1000) * 1000};
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));

  Serial.println("[http] 客户端连接");
  g_clients.fetch_add(1);
  uint8_t *local = nullptr;
  size_t localCap = 0;
  uint32_t lastSeq = g_jpegSeq.load();
  uint32_t lastFrameMs = millis();
  const size_t boundaryLen = strlen(kStreamBoundary);
  constexpr size_t kHeaderRoom = 224;

  while (true) {
    const uint32_t seq = g_jpegSeq.load();
    if (seq == lastSeq) {
      if (millis() - lastFrameMs > kStreamStallMs) {
        res = ESP_FAIL;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    xSemaphoreTake(g_jpegMutex, portMAX_DELAY);
    const size_t len = g_jpegLen;
    if (len + kHeaderRoom > localCap) {
      free(local);
      local = static_cast<uint8_t *>(ps_malloc(len + kHeaderRoom));
      localCap = local ? len + kHeaderRoom : 0;
    }
    size_t prefix = 0;
    if (local) {
      memcpy(local, kStreamBoundary, boundaryLen);
      prefix = boundaryLen + snprintf(reinterpret_cast<char *>(local) + boundaryLen, kHeaderRoom - boundaryLen,
                                      kStreamPartHeader, static_cast<unsigned>(len), static_cast<unsigned>(g_jpegSeq.load()),
                                      static_cast<unsigned>(g_captureMs), static_cast<unsigned>(millis() - g_captureMs));
      memcpy(local + prefix, g_jpegBuf, len);
    }
    lastSeq = g_jpegSeq.load();
    xSemaphoreGive(g_jpegMutex);
    if (!local) {
      res = ESP_ERR_NO_MEM;
      break;
    }
    lastFrameMs = millis();

    res = httpd_resp_send_chunk(req, reinterpret_cast<const char *>(local), prefix + len);
    if (res != ESP_OK) {
      Serial.printf("[http] send failed: result=0x%x errno=%d bytes=%u\n", res, errno,
                    static_cast<unsigned>(prefix + len));
      break;
    }
  }

  free(local);
  g_clients.fetch_sub(1);
  Serial.println("[http] 客户端断开");
  return res;
}

static esp_err_t captureHandler(httpd_req_t *req) {
  g_clients.fetch_add(1);
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    g_clients.fetch_sub(1);
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No camera frame");
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t err = httpd_resp_send(req, reinterpret_cast<const char *>(fb->buf), fb->len);
  esp_camera_fb_return(fb);
  g_clients.fetch_sub(1);
  return err;
}

static void startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = STREAM_PORT;
  config.ctrl_port = STREAM_PORT + 32768;
  config.max_open_sockets = 2;
  config.core_id = 0;
  config.send_wait_timeout = (STREAM_SEND_TIMEOUT_MS + 999) / 1000;

  httpd_uri_t streamUri = {};
  streamUri.uri = "/stream";
  streamUri.method = HTTP_GET;
  streamUri.handler = streamHandler;
  httpd_uri_t captureUri = {};
  captureUri.uri = "/capture";
  captureUri.method = HTTP_GET;
  captureUri.handler = captureHandler;

  if (httpd_start(&streamServer, &config) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &streamUri);
    httpd_register_uri_handler(streamServer, &captureUri);
    Serial.printf("[http] 端口 %d 就绪\n", STREAM_PORT);
  }
}

// ---------- 设备发现 (UDP 广播) ----------

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
  discoveryUdp.flush();
  if (len <= 0) return;
  buf[len] = '\0';

  char type[16];
  char nonce[40];
  if (!extractToken(buf, "type", type, sizeof(type)) || strcmp(type, "discover") != 0) return;
  if (!extractToken(buf, "nonce", nonce, sizeof(nonce))) return;

  String ip = (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  char reply[256];
  int n = snprintf(reply, sizeof(reply),
                   "{\"type\":\"announce\",\"version\":1,\"nonce\":\"%s\",\"id\":%d,"
                   "\"ip\":\"%s\",\"stream\":\"http://%s:%d/stream\",\"udp_stream\":%d,\"control_port\":%d}",
                   nonce, DEVICE_ID, ip.c_str(), ip.c_str(), STREAM_PORT, STREAM_UDP_PORT, CONTROL_PORT);
  if (n <= 0 || n >= static_cast<int>(sizeof(reply))) return;

  discoveryUdp.beginPacket(discoveryUdp.remoteIP(), discoveryUdp.remotePort());
  discoveryUdp.write(reinterpret_cast<const uint8_t *>(reply), n);
  discoveryUdp.endPacket();
}

// ---------- Wi-Fi 初始化 (彻底消除单射频时分复用与休眠延迟) ----------

static void setupWifi() {
#if CAM_WIFI_MODE_AP
  // 模式 1: 纯 AP 独立直连热点模式
  char ssid[24];
  snprintf(ssid, sizeof(ssid), "disp-cam-%d", DEVICE_ID);
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP);

  const IPAddress ip(192, 168, 4, 1), mask(255, 255, 255, 0), lease(192, 168, 4, 2);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  bool ok = WiFi.AP.begin() &&
            WiFi.AP.config(ip, ip, mask, lease, WiFi.dnsIP()) &&
            WiFi.AP.create(ssid, CAM_DIRECT_AP_PASSWORD, CAM_AP_CHANNEL, 0, 4);
#else
  bool ok = WiFi.softAPConfig(ip, ip, mask) &&
            WiFi.softAP(ssid, CAM_DIRECT_AP_PASSWORD, CAM_AP_CHANNEL, 0, 4);
#endif
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.printf("\n[wifi] 独立直连 AP 启动: SSID=%s IP=%s 信道=%d 状态=%d\n",
                ssid, WiFi.softAPIP().toString().c_str(), CAM_AP_CHANNEL, ok);
#else
  // 模式 0: 纯 STA 路由器模式
  char hostname[24];
  snprintf(hostname, sizeof(hostname), "disp-cam-%d", DEVICE_ID);

  WiFi.persistent(false);
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  WiFi.setTxPower(CAM_WIFI_TX_POWER);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    g_lastWifiEvent.store(millis());
    if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
      Serial.println("\n[wifi-event] 已关联 AP");
      esp_wifi_set_ps(WIFI_PS_NONE);
      WiFi.setSleep(false);
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.printf("\n[wifi-event] 获取到 IP: %s\n", IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
      esp_wifi_set_ps(WIFI_PS_NONE);
      WiFi.setSleep(false);
    } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.printf("\n[wifi-event] STA 断开, reason=%d\n", info.wifi_sta_disconnected.reason);
    }
  });

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[wifi] 连接 %s", WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - start > 25000) {
      Serial.println("\n[wifi] 25s 未连上，重启");
      ESP.restart();
    }
  }
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setSleep(false);
  WiFi.setTxPower(CAM_WIFI_TX_POWER);
  Serial.printf("\n[wifi] 已连接 STA，IP %s，RSSI %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  wifi_ap_record_t ap{};
  if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
    Serial.printf("[wifi] auth=%d pairwise=%d group=%d channel=%d\n", ap.authmode, ap.pairwise_cipher, ap.group_cipher, ap.primary);
#endif
}

// ---------- 诊断命令 ----------

static void pollDiagnostics() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 'n' || c == 'N') {
      Serial.printf("[net] STA MAC=%s mask=%s gateway=%s\n", WiFi.macAddress().c_str(),
                    WiFi.subnetMask().toString().c_str(), WiFi.gatewayIP().toString().c_str());
      for (int i = 0; i < ARP_TABLE_SIZE; ++i) {
        ip4_addr_t *ip; struct netif *iface; struct eth_addr *mac;
        if (etharp_get_entry(i, &ip, &iface, &mac)) {
          Serial.printf("[arp] %s %02x:%02x:%02x:%02x:%02x:%02x\n", ip4addr_ntoa(ip),
                        mac->addr[0], mac->addr[1], mac->addr[2], mac->addr[3], mac->addr[4], mac->addr[5]);
        }
      }
    }
    if (c == 's' || c == 'S') {
      wifi_ps_type_t ps = WIFI_PS_NONE; esp_wifi_get_ps(&ps);
      Serial.printf("[diag] IP=%s Heap=%u PSRAM=%u Clients(UDP=%d,HTTP=%d) RSSI=%d power=%d PS=%d\n",
                    (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str(),
                    ESP.getFreeHeap(), ESP.getFreePsram(),
                    g_udpClients.load(), g_clients.load(), WiFi.RSSI(), int(WiFi.getTxPower()), int(ps));
    }
#if !CAM_WIFI_MODE_AP
    // Local serial diagnostics only; keep experiments independent of a working video link.
    if (c == 'p' || c == 'P') {
      const bool ok = WiFi.setTxPower(c == 'p' ? WIFI_POWER_15dBm : CAM_WIFI_TX_POWER);
      Serial.printf("[radio] power result=%d value=%d\n", ok, int(WiFi.getTxPower()));
    }
    if (c == 'b' || c == 'g' || c == 'r') {
      WiFi.disconnect(false, false);
      if (c != 'r') {
        const uint8_t protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                 (c == 'g' ? WIFI_PROTOCOL_11N : 0);
        Serial.printf("[radio] protocol=%u result=%d\n", protocol,
                      esp_wifi_set_protocol(WIFI_IF_STA, protocol));
      }
      WiFi.reconnect();
    }
#endif
  }
}

// ---------- 主程序 ----------

void setup() {
  Serial.begin(115200);
  Serial.printf("\n[boot] 位移跟踪相机 #%d (极速 UDP 直出引擎)\n", DEVICE_ID);

#ifdef FLASH_LED_GPIO_NUM
  pinMode(FLASH_LED_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_LED_GPIO_NUM, LOW);
#endif

#if defined(PWDN_GPIO_NUM) && PWDN_GPIO_NUM != -1
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH); // 进入传感器 power-down；不等于切断供电轨
  delay(30);
  digitalWrite(PWDN_GPIO_NUM, LOW);  // 再上电恢复
  delay(30);
#endif

  // 先完成联网，再启动摄像头时钟与 DMA，减少初始化工作的重叠。
  setupWifi();

  g_jpegMutex = xSemaphoreCreateMutex();
  if (!g_jpegMutex) { delay(3000); ESP.restart(); return; }
  if (!startCamera()) {
    delay(3000);
    ESP.restart();
  }

  startStreamServer();

  // Core 1 (优先级 5)：纯净硬件 DMA 持续采集，绝不被网络拥塞阻塞
  if (xTaskCreatePinnedToCore(captureTask, "capture", 8192, nullptr, 5, nullptr, 1) != pdPASS) {
    Serial.println("[boot] captureTask 创建失败");
    delay(3000); ESP.restart(); return;
  }

  // Core 0 (优先级 4)：与 Wi-Fi / lwIP 同核运行，平滑分片极速 UDP 直出推流
  if (xTaskCreatePinnedToCore(udpVideoTask, "udp-stream", 6144, nullptr, 4, nullptr, 0) != pdPASS) {
    Serial.println("[boot] udpVideoTask 创建失败");
    delay(3000); ESP.restart(); return;
  }

  discoveryUdp.begin(DISCOVERY_PORT);
  Serial.printf("[discovery] 广播发现 UDP %d, 视频流 UDP %d\n", DISCOVERY_PORT, STREAM_UDP_PORT);
}

void loop() {
  pollDiscovery();
  pollDiagnostics();

#if !CAM_WIFI_MODE_AP
  static uint32_t lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    // AutoReconnect owns normal recovery. Do not restart association/DHCP every 5s.
    if (WiFi.status() != WL_CONNECTED && millis() - g_lastWifiEvent.load() >= 30000) {
      g_lastWifiEvent.store(millis());
      Serial.println("[wifi] 30s 无联网进展，重试连接");
      WiFi.reconnect();
    }
  }
#endif
  delay(10);
}

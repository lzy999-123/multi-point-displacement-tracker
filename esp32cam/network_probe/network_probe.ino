// Diagnostic only: camera disabled; ordinary lab-router UDP/TCP traffic.
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include "esp_camera.h"
#include "driver/ledc.h"
#define BOARD_AI_THINKER
#include "camera_pins.h"
#include "secrets.h"

WiFiUDP udp;
WiFiServer server(81);
uint32_t received = 0;
bool cameraStarted = false;
int cameraClockHz = 20000000;
bool cameraDram = false;

// Read-only reachability checks: compare router/WAN traffic with PC peer traffic.
void probeHttp(const char *host, const char *label) {
  WiFiClient connection;
  uint32_t started = millis();
  bool connected = connection.connect(host, 80, 2500);
  Serial.printf("[path] %s host=%s camera=%d tcp=%d connect_ms=%u\n",
                label, host, cameraStarted, connected, millis() - started);
  if (!connected) return;
  connection.printf("HEAD / HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n", host);
  started = millis();
  while (!connection.available() && connection.connected() && millis() - started < 2500) delay(5);
  int bytes = connection.available();
  char head[129] = {};
  size_t count = bytes > 0 ? connection.readBytes(head, min(bytes, 128)) : 0;
  Serial.printf("[path] %s reply_bytes=%d head_bytes=%u wait_ms=%u head=%s\n",
                label, bytes, unsigned(count), millis() - started, head);
  connection.stop();
}

void startCamera() {
  camera_config_t c{};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0=Y2_GPIO_NUM; c.pin_d1=Y3_GPIO_NUM; c.pin_d2=Y4_GPIO_NUM; c.pin_d3=Y5_GPIO_NUM;
  c.pin_d4=Y6_GPIO_NUM; c.pin_d5=Y7_GPIO_NUM; c.pin_d6=Y8_GPIO_NUM; c.pin_d7=Y9_GPIO_NUM;
  c.pin_xclk=XCLK_GPIO_NUM; c.pin_pclk=PCLK_GPIO_NUM; c.pin_vsync=VSYNC_GPIO_NUM;
  c.pin_href=HREF_GPIO_NUM; c.pin_sccb_sda=SIOD_GPIO_NUM; c.pin_sccb_scl=SIOC_GPIO_NUM;
  c.pin_pwdn=PWDN_GPIO_NUM; c.pin_reset=RESET_GPIO_NUM;
  c.xclk_freq_hz=cameraClockHz; c.pixel_format=PIXFORMAT_JPEG; c.frame_size=FRAMESIZE_QVGA;
  c.jpeg_quality=20; c.fb_count=2; c.fb_location=cameraDram ? CAMERA_FB_IN_DRAM : CAMERA_FB_IN_PSRAM;
  c.grab_mode=CAMERA_GRAB_LATEST;
  esp_err_t result=esp_camera_init(&c);cameraStarted=result==ESP_OK;
  Serial.printf("[probe] camera-init=%d heap=%u xclk=%d dram=%d\n",result,ESP.getFreeHeap(),cameraClockHz,cameraDram);
}

void setup() {
  Serial.begin(115200);
  pinMode(32, OUTPUT);
  digitalWrite(32, HIGH); // Keep the sensor in power-down during this comparison.
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(250);
  Serial.printf("[probe] ip=%s mac=%s rssi=%d sdk=%s\n", WiFi.localIP().toString().c_str(),
                WiFi.macAddress().c_str(), WiFi.RSSI(), ESP.getSdkVersion());
  udp.begin(5006);
  server.begin();
}

void loop() {
  int size = udp.parsePacket();
  if (size > 0) {
    uint8_t buf[1500];
    int count = udp.read(buf, sizeof(buf));
    IPAddress ip = udp.remoteIP();
    uint16_t port = udp.remotePort();
    bool broadcast = count >= 4 && memcmp(buf, "VBRC", 4) == 0;
    IPAddress target = broadcast ? IPAddress(uint32_t(WiFi.localIP()) | ~uint32_t(WiFi.subnetMask())) : ip;
    bool start = udp.beginPacket(target, port);
    size_t written = udp.write(buf, count);
    bool sent = udp.endPacket();
    Serial.printf("[probe] rx=%u from=%s:%u bytes=%d broadcast=%d begin=%d written=%u sent=%d\n",
                  ++received, ip.toString().c_str(), port, count, broadcast, start, written, sent);
  }
  WiFiClient client = server.available();
  if (client) {
    Serial.printf("[probe] TCP accepted %s\n", client.remoteIP().toString().c_str());
    client.print("HTTP/1.0 200 OK\r\nContent-Length: 5\r\nConnection: close\r\n\r\nprobe");
    client.stop();
  }
  if (Serial.available()) {
    char command = Serial.read();
    if (command == '1') WiFi.setTxPower(WIFI_POWER_8_5dBm);
    if (command == '2') WiFi.setTxPower(WIFI_POWER_15dBm);
    if (command == '3') WiFi.setTxPower(WIFI_POWER_19_5dBm);
    if ((command == 'o' || command == 'v') && cameraStarted) {
      esp_err_t result = command == 'o' ? ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0)
                                       : ledc_timer_resume(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
      Serial.printf("[clock] paused=%d result=%d\n", command == 'o', result);
    }
    if (command == 't') {
      String gateway = WiFi.gatewayIP().toString();
      probeHttp(gateway.c_str(), "gateway");
      probeHttp("example.com", "external-http");
    }
    if (command == 'c' && !cameraStarted) startCamera();
    if (command == 'j' || command == 'k' || command == 'l' || command == 'd') {
      if (cameraStarted) { esp_camera_deinit(); cameraStarted=false; }
      cameraClockHz = command == 'j' ? 10000000 : command == 'k' ? 16000000 : 20000000;
      cameraDram = command == 'd';
      startCamera();
    }
    if (command == 'x' && cameraStarted) { esp_camera_deinit(); cameraStarted=false; }
    if (command == 'f' && cameraStarted) {
      camera_fb_t *fb=esp_camera_fb_get();
      if(fb) { Serial.printf("[probe] frame=%u\n",fb->len);esp_camera_fb_return(fb); }
    }
    if (command == 'b' || command == 'g') {
      WiFi.disconnect();
      esp_wifi_set_protocol(WIFI_IF_STA, command == 'b' ? WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G : WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    Serial.printf("[probe] command=%c power=%d state=%d ip=%s\n", command, int(WiFi.getTxPower()), int(WiFi.status()), WiFi.localIP().toString().c_str());
  }
  delay(1);
}

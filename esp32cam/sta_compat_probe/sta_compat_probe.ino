// Temporary diagnostic: camera is held in power-down; no video claims.
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include "secrets.h"

WiFiUDP echo;
WiFiServer server(81);
uint32_t lastReport = 0;

void setup() {
  Serial.begin(115200);
  pinMode(32, OUTPUT); digitalWrite(32, HIGH);
  pinMode(4, OUTPUT); digitalWrite(4, LOW);
  Serial.printf("[compat] SDK=%s MAC pending; camera off\n", ESP.getSdkVersion());
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
  if (WiFi.status() == WL_CONNECTED) { echo.begin(5006); server.begin(); }
  Serial.printf("[compat-done] connected=%d IP=%s RSSI=%d\n",
                WiFi.status() == WL_CONNECTED, WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

void loop() {
  int size = echo.parsePacket();
  if (size > 0) {
    uint8_t data[1500];
    int count = echo.read(data, sizeof(data));
    echo.beginPacket(echo.remoteIP(), echo.remotePort()); echo.write(data, count);
    Serial.printf("[compat-echo] bytes=%d sent=%d\n", count, echo.endPacket());
  }
  WiFiClient peer = server.available();
  if (peer) {
    peer.print("HTTP/1.0 200 OK\r\nContent-Length: 5\r\nConnection: close\r\n\r\nprobe");
    peer.stop();
  }
  if (millis() - lastReport >= 5000) {
    lastReport = millis();
    Serial.printf("[compat-state] connected=%d IP=%s RSSI=%d\n",
                  WiFi.status() == WL_CONNECTED, WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }
  delay(5);
}

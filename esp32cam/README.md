# ESP32-CAM 固件

阶段 1 功能：连接 WiFi、HTTP MJPEG 推流（`http://IP:81/stream`）、响应上位机的 UDP 设备发现。
控制通道和串口转发在阶段 2 加入。

## 烧录步骤（Arduino IDE）

1. 安装 ESP32 开发板支持（Boards Manager 搜索 "esp32"，Espressif 官方包，2.x 或 3.x）。
2. 开发板选 **AI Thinker ESP32-CAM**，PSRAM 选 Enabled。
3. 把 `displacement_cam/secrets.example.h` 复制为 `secrets.h`，填写 WiFi 名称和密码。
4. 在 `config.h` 修改 `DEVICE_ID`（三台分别为 1、2、3）。
5. 用 USB-TTL 烧录：GPIO0 接 GND 后上电进入下载模式，烧录完成后断开 GPIO0 再重新上电。
6. 串口监视器（115200）会打印 IP 和视频地址，浏览器打开该地址可以直接看画面。

## 注意

- 供电用稳定的 5V，电流 ≥500mA。电源不足时开 WiFi 会掉电重启，串口会出现 `Brownout detector was triggered`。
- 三台设备的 `DEVICE_ID` 不能重复，上位机发现编号冲突时会提示，并只连接先发现的那台。
- 视频同一时刻只服务一个客户端，浏览器在看时上位机连不上。

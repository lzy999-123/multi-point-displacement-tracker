# ESP32-CAM 固件

2026-11-12 当前配置：三台均为普通 ESP32-D0WD-V3＋OV3660，4MB Flash / 4MB PSRAM，分别为 COM3、COM5、COM11（ID 1/2/3）。已完成每块 4MB 完整备份及写入校验，使用 Arduino-ESP32 3.3.12、手机热点 STA、原生 640×480、硬件 JPEG 质量 18。当前 UDP 每帧节拍 45ms、分片间隔 3ms；这只是发送目标，不代表三路稳定 20fps 已通过。用户后续截图显示三路恢复到约 20fps，长测中的波动仍保留记录。测试记录与固件快照见 [三路实测](../output/三路ESP32CAM实测-20261112/实测记录.md)。

为每块板单独构建时，使用 `build.ps1 -Board ESP32Cam -Framework 3.3.12 -DeviceId 1`（另两块分别用 2、3）；不要把同一个 ID 的 BIN 烧给三块板。以下为历史板型、网络和分辨率记录。

历史记录（2026-11-11 S3接回）：旧ESP32-S3＋GC2145已识别为COM6，已完成整片备份校验，当前运行隔离构建的3.3.12快速HTTP编码版本，通过实验室DH4504向Qt传图；完整稳定性验收仍失败。该S3构建与下方默认OV3660源码/旧2.0.17构建不同，不要混刷，源码及BIN冻结位置见 [S3恢复与实测](../output/实验室网络实测-20261111/S3接回恢复与实测.md)。

历史记录：2026-11-11 本机 COM5 已烧录实验室 STA 固件，连接 `DH4504`，IP 为 `192.168.110.96`。实验室稳定传图尚未通过，见 [本轮实测](../output/实验室网络实测-20261111/实测记录.md)。下文纯 AP 默认模式及通过数据属于此前直连配置。

## 当前新板：普通 ESP32-CAM + OV3660（2026-11-09）

USB/COM5 已识别为 ESP32-D0WD-V3 rev3.1、4MB Flash，原厂启动标识 ESP32-CAM-MB。
普通 ESP32 自动选择 AI-Thinker 摄像头引脚、20MHz XCLK 和硬件 JPEG；不使用 S3 专用寄存器或软件编码库。
本机使用已安装的 Arduino-ESP32 3.3.12，320×240、JPEG quality=20，当前默认纯 AP 模式。
热点为 `disp-cam-1`，信道1、20MHz带宽、相机IP `192.168.4.1`。密码由本地 `secrets.h` 配置。
Qt 默认使用 UDP 5006，HTTP 81 保留兼容。帧内分片有2ms间隔和有限重试，超过发送预算或缺片则放弃旧帧。
OV3660 的亮度、饱和度和垂直翻转参考本机官方 CameraWebServer 示例；实物方向仍需观察确认。

```powershell
# 新板：编译 / 编译并烧录
.\esp32cam\build.ps1
.\esp32cam\build.ps1 -Flash -Port COM5
# 旧 S3 板：保持隔离的 2.0.17 工具链
.\esp32cam\build.ps1 -Board S3
```

新板输出：`D:\esp32\build-out\displacement-cam-ov3660-3.3.12\bin`。
最新真实 Qt 自动启动130秒测试已通过，预热10秒后逐秒最低25帧、最长画面间隔110ms。
正式程序自行管理 COM5 和 WLAN，会在退出时释放串口并恢复启动前的WiFi；使用 `host/run.cmd`。
详见 [最新修复与验收](../docs/Qt画面流畅性修复与实测-2026-11-09.md)。
较早 HTTP 测试的740ms停顿属于历史对照，保留于 [旧记录](../docs/OV3660实机优化-2026-11-09.md)。
FQBN：`esp32:esp32:esp32cam:FlashMode=dio,FlashFreq=40`。
新板引导程序写入 `0x1000`、Flash 为 4MB；旧 S3 引导程序为 `0x0`、Flash 为 8MB，不能混用。
`build-info.json` 记录构建版本与源码/BIN 哈希，不记录 WiFi 密码。
`secrets.h` 保留本地 WiFi 设置；切回路由器组网需将 `CAM_WIFI_MODE_AP=0` 后重新烧录。
三台相机并发不能使用当前三个独立热点作为同一张WiFi网络，须另行验收共享路由器链路。

以下 GC2145 性能数据和 S3 工具链说明为旧板记录，不代表新 OV3660 实测结果。

阶段 1 功能：连接 WiFi、HTTP MJPEG 推流（`http://IP:81/stream`）、响应上位机的 UDP 设备发现。
控制通道和串口转发在阶段 2 加入。

2026-11-06 已在真实 ESP32-S3 + GC2145 上编译、烧录并收到 320×240 图像，但板端约 14.8fps 的采集速率没有转化为流畅传输；35秒仅收到11张完整图像，仍未通过视频流畅性验收。详见 [单机实测记录](../docs/ESP32单机实测-2026-11-06.md)。

此前 GC2145 目标为**无线320×240 @20fps**，不以USB或160×120代替验收。
源码已采用YUV422输入、优化软件JPEG编码器、双缓冲和40ms采集间隔，保留标准MJPEG与最新帧覆盖。
工具链固定为隔离安装的 Arduino-ESP32 **2.0.17**；发送等待改为1000ms，给TCP重传保留时间，减少过早断开。
实板采集约25fps、压缩约17ms，但无线20fps目标仍未通过，不能把采集速率或单个流畅区间当成完成证明。
研究依据见 [GitHub调研](../docs/GitHub参考项目调研-2026-11-06.md)，各实机对照见 [优化记录](../docs/ESP32推流优化-2026-11-06.md)。

## 硬件

当前开发板：ESP32-S3（QFN56，8MB PSRAM，8MB Flash）+ GC2145 摄像头，板载 CH340 串口。
摄像头引脚按乐鑫 ESP32-S3-EYE，已用 `camera_probe/` 实测确认。

GC2145 没有硬件 JPEG，当前固件取YUV422帧后用CPU优化编码器压缩。早期RGB565探测数据（旧编码路径）：

| 分辨率 | 取帧 | 软件 JPEG | 每帧大小 | 帧率 |
| --- | --- | --- | --- | --- |
| QVGA 320×240 | 2.9 ms | 64 ms | 约 2.2 KB | 约 15 fps |
| VGA 640×480 | 8.1 ms | 263 ms | 约 15 KB | 约 3.7 fps |

当前默认QVGA（320×240），双缓冲下实测优化压缩约16.7～17.2ms，连续采集约25fps；40ms采集间隔保留余量。
正式固件把压缩（核 1）和网络发送（核 0）分开，实际帧率以上板和Qt窗口的完整时段记录为准。

用 AI-Thinker ESP32-CAM（ESP32 + OV2640）时，在 `config.h` 打开 `BOARD_AI_THINKER`，固件会自动使用硬件 JPEG。

## 烧录

工具链装在 `D:\esp32`（见 `D:\esp32\README.md`）。

1. 把 `displacement_cam/secrets.example.h` 复制为 `secrets.h`，填写 WiFi 名称和密码（ESP32 只支持 2.4GHz）。
   `secrets.h` 已在 `.gitignore` 中忽略。缺少该文件时编译会直接报错提示。
2. 在 `config.h` 修改 `DEVICE_ID`（三台分别为 1、2、3）。
3. 使用固定版本脚本编译并上传（本板实测230400可靠）：

```powershell
# 在项目根目录，只编译：
.\esp32cam\build.ps1
# 编译并烧录（先关闭串口监视器和视频客户端）：
.\esp32cam\build.ps1 -Flash -Port COM5
```

脚本针对当前S3板，默认构建产物在 `D:\esp32\build-out\displacement-cam-2.0.17\bin`，
同级 `build-info.json` 保存版本、应用BIN和非秘密源码的SHA256。使用ASCII构建路径以避开旧版链接器的中文路径问题。

本机隔离工具链在 `D:\esp32\compat-2.0.17`，未替换原有3.3.12安装。
官方2.0.17源码和配套GCC8.4.0压缩包已与乐鑫包索引的SHA256核对。
脚本要求以下现有文件；换电脑时需要先准备同版本工具链，脚本不会自动下载或静默换版本：

- `arduino-cli.yaml` 指向隔离的 `sketchbook`。
- `sketchbook/hardware/espressif/esp32` 指向 `core-extract/esp32-2.0.17`。
- `compiler/xtensa-esp32s3-elf` 为 esp-2021r2-patch5 GCC8.4.0。
- 原有 `D:\esp32\arduino15\packages\esp32\tools\esptool_py\5.3.1` 与 `D:\esp32\venv\Scripts\python.exe`。
- 本机官方 `esp32s3-libs/3.3.12` 中的 `libesp_new_jpeg.a` 及配套头文件：仅复用软件编码库，WiFi/相机驱动仍用2.0.17。脚本同时记录编码库SHA256。

本版需要上述额外编码库的链接配置，建议使用已验证脚本；仅在IDE中选择2.0.17不足以完成构建。

4. 串口监视器（115200）会打印 IP、视频地址和每 5 秒的帧率统计。

## 注意

- 供电要稳定的 5V。电源不足时开 WiFi 会掉电重启，串口会出现 `Brownout detector was triggered`。
- 三台设备的 `DEVICE_ID` 不能重复。上位机发现编号冲突时会提示，并只连接先发现的那台。
- 视频同一时刻只服务一个客户端，浏览器在看时上位机连不上。
- GPIO4～13、15～18 被摄像头占用，GPIO35～37 被 PSRAM 占用，阶段 2 接 STM32 串口时不能用这些引脚。
- 原厂固件的整片备份在 `D:\esp32\backup`，可以随时写回。

# 上位机（Qt 6 + OpenCV）

当前更新（2026-11-11 S3接回）：通道1现为COM6、`http://192.168.110.59:81/stream?nodelay=0`，电脑保持实验室DH4504。已验证真实视频显示，但稳定性仍未通过。使用 `run.cmd`；证据见 [S3恢复与实测](../output/实验室网络实测-20261111/S3接回恢复与实测.md)。以下COM5等配置按对应历史条件理解。

2026-11-11 当前通道 1 配置已更新到实验室 `udp://192.168.110.96:5006`，保持电脑当前 WLAN。当前只有 COM5 在线，实验室传图验收失败，详见 [本轮实测](../output/实验室网络实测-20261111/实测记录.md)。以下手机双路配置是此前演示记录，不能作为当前实验室链路已通过的证明。

## 构建

依赖：Qt 6.10（MSVC 2022 64 位）、OpenCV 4.12（默认 `D:/opencv_build/build/install`，可用环境变量 `OPENCV_DIR` 修改）。

- Qt Creator：打开 `DisplacementTracker.pro`。
- 命令行：`build.cmd release`，输出在 `build/DisplacementTracker-release/release/`。

运行时需要 Qt 和 OpenCV 的 `bin` 目录在 PATH 中（Qt Creator 运行时自动处理 Qt 部分）。

也可双击 `run.cmd`：将构建好的 EXE、Qt 插件和 OpenCV DLL 部署到 `build/run/` 再启动。
需要手动重新构建时先运行 `build.cmd release`。部署脚本默认使用上述本机依赖路径，可通过参数覆盖。
本机 Qt 还需要 Windows ICU 实现与转发 DLL 配套，`deploy.ps1` 会复制到运行目录，不更改系统 DLL。

## 使用

本机在 2026-11-11 已配置两台相机连接手机热点 `Redmi K70E`。使用项目根目录
`演示图像上传.cmd`，或 `run.cmd`，可打开两路 HTTP 视频：通道 1 使用 COM5，通道 2 使用 COM3。
两块板的设备编号分别为 1、2。保持电脑连接该手机热点，并给两块板供电；视频通过 WiFi 传输。
现场地址和实测数据见项目 `output/双路图像实测-20261111/`，DHCP 地址变化后需更新配置。

`camera-runtime.ini` 的 `[camera]`、`[camera2]`、`[camera3]` 分别配置各通道的
`serialPort` 与 `url`；公用的 `wifiProfile` 仍放在 `[camera]`。空串口表示独立供电，空
`wifiProfile` 表示保持电脑当前 WLAN。程序随窗口打开所有已配置的 CH340 串口，退出时一同释放。
每路画面旁有“旋转180°”开关，保存到对应配置组的 `rotate180`。画面、掩膜和检测框一起旋转，
状态文字保持正向，点击取色会映射回原始图像坐标。

项目内的 Qt Creator 构建、命令行构建与演示目录统一读取 `host/camera-runtime.ini`，
不再被各构建目录残留的旧配置影响。独立复制到项目外运行时使用 EXE 旁的配置。
部署脚本会同时更新项目里已有的 Qt Creator Release EXE 与依赖。
配置了 `serialPort` 的通道随该 CH340 的插拔启动和停止；缺少该串口时显示“设备已断开”，
其他通道继续运行。相机改为独立供电或只拔 USB 管理线时，清空对应 `serialPort` 以保持无线接收。

下面是先前单机直连热点方案的说明，供需要恢复该固件时使用。

本机 OV3660 默认使用相机热点 `disp-cam-1`，视频为 `udp://192.168.4.1:5006`。
双击 `run.cmd` 或 `run-direct.cmd`，程序自动打开本地配置的相机 USB 串口、连接已保存的相机 WLAN，
然后显示视频。关闭程序会释放串口、恢复此前的 WiFi；没有后台守护进程。
当前是纯 AP，直连期间电脑 WiFi 不再连接实验室路由器，不提供 NAPT 或互联网转发。

本地 `camera-runtime.ini`（不提交 Git）配置 `camera/serialPort`、`camera/wifiProfile`、`camera/url`，
部署脚本会复制到 EXE 同目录。独立供电时可清空 `serialPort`；路由器组网时清空 `wifiProfile`，
并将相机固件的 `CAM_WIFI_MODE_AP` 改为 0 后重新烧录，使用设备自动发现。
相机 WLAN 配置已在本机保存；换电脑需先使用本地 `secrets.h` 中的热点密码连接一次。
程序在连接期间使用 Windows WLAN 流媒体模式并暂时禁用背景扫描，退出时恢复原扫描设置。
正式程序只允许一个实例，避免多个客户端覆盖相机推流目标。

当前验收要求是无线320×240 @20fps。
此前的 `run-usb.cmd` / `tools/run_usb_camera.py` 仅为旧诊断固件保留，当前正式固件不再包含USB视频命令，不用于本轮验收。
Qt支持 `--stream-url http://.../stream` 指定通道1地址。

启动后自动广播查找设备，发现的设备按编号填入对应监测点并开始接收画面，也可以手动输入地址点"连接"。
在右下角调整色块阈值，或直接点击画面中的色块自动取色。绿色框为跟踪目标，绿线为目标相对画面中心的偏移。

## 测试

```
build.cmd release tests\core_test.pro
build\core_test-release\release\core_test.exe
```

测试覆盖 MJPEG 解析、色块检测、取色和设备发现解析。联机测试需要先运行模拟相机：

```
python ..\tools\fake_esp32cam.py
set FAKE_CAM_URL=http://127.0.0.1:8081/stream
build\core_test-release\release\core_test.exe
```

没有硬件时也可以先运行 `fake_esp32cam.py`，再打开上位机，会自动发现 3 台模拟相机（蓝、绿、红色块）。

## 实机显示验收

`tests/live_viewer.pro` 使用实际 `MainWindow`、`Channel`、检测和绘制路径，分别记录收到的图像和真正绘制的新图像：

```powershell
.\build.cmd release tests\live_viewer.pro
.\deploy.ps1 -Executables live_viewer
.\build\run\live_viewer.exe http://IP:81/stream 120 D:\esp32\build-out\viewer-result.json
```

运行期间会显示实际三路窗口；到时保存逐秒数据和窗口 PNG 后关闭。当前默认验收严格对应无线320×240 @20fps：
预热2秒后每秒至少20张新图像、最长无新图像不超过150ms、分辨率必须320×240且没有重连。
可在输出JSON路径后追加最低逐秒帧数和最大间隔两个参数；报告会保存所用阈值。
退出码0表示通过，1表示未通过；没有连上相机也会如实记零帧。提前关闭窗口没有验收报告，不能算通过。
可以用 `live_viewer.exe auto 130 输出路径.json` 验证正式程序共用的串口和 WLAN 自动启动逻辑，
此模式为上电和切网预留10秒，并对后续120秒逐秒验收；显式URL模式仍预热2秒。
验收期间窗口置顶且不周期性截图，避免窗口被遮挡或主线程PNG编码干扰绘制统计。
输出包含原始解码/绘制时间；结束后才保存最终截图。`auto` 模式还保存串口日志。
这个标准仅描述测试窗口内的表现，不代表三台相机并发、端到端光学延迟或无限期稳定。

双路真实窗口验证使用 `tests/dual_viewer.pro`，与正式程序共用 `CameraRuntime`、`MainWindow`、
解码、检测和绘制路径：构建、部署后运行 `dual_viewer.exe 45 输出路径.json`。
预热 10 秒后逐秒记录两路实际绘制的新图像，输出完整时间、两份串口日志及窗口截图。
`simultaneous_display_passed` 表示两路在每个完整秒内均显示了至少一张新图像，且分辨率正确、无重连；
`strict_20fps_passed` 另行判断每路每秒至少 20 帧且最大间隔不超过 150ms。
退出码对应同时显示验证，不能据此宣称通过稳定 20fps 验收。

旋转与点击映射测试：构建 `tests/viewer_test.pro`，运行 `viewer_test.exe`。
当前只插通道 1 的实测可用 `dual_viewer.exe 25 输出路径.json one-connected`；
要求通道 1 持续收到新画面、通道 2 不建立视频连接，并单独记录严格 20fps 性能结果。

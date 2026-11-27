# 多检测点特征跟踪上位机（Qt / OpenCV / ONNX Runtime）

当前主线为**无下位机、三路固定机位 ESP32 无线图传 + 目标实时三态跟踪监测**，项目总览见根 [README.md](../README.md)。

## 纯无线视频链路

- 上位机 `DisplacementTracker.exe` 仅接收来自 ESP32-CAM 的无线 UDP 视频流（VID1 协议，端口 `5006`），不包含本地摄像头（`VideoCapture` / `QCamera`）或 USB 串口视频输入。
- `CameraRuntime` / `CameraSerialSession` 仅在配置了 `serialPort` 时读取 115200 波特率文本诊断日志并检测 CH340 在线；若改用独立 5V 供电，可清空 `camera-runtime.ini` 中的 `serialPort`。
- 串口枚举、WLAN 流媒体模式配置与网卡枚举均在后台线程执行，不阻塞三路画面渲染。

## 构建、部署与运行

依赖：**Qt 6.10.1 MSVC2022 64 位**、**OpenCV 4.12**、**ONNX Runtime**（内置于 `host/third_party/onnxruntime/`）与 **YOLO26n 模型**（`host/models/yolo26n.onnx`）。

- **日常启动**：双击 `run.cmd`（自动增量构建、部署依赖/模型并启动 `build/run/DisplacementTracker.exe`）。
- **配置文件**：`host/camera-runtime.ini`（部署时同步至 `host/build/run/camera-runtime.ini`），默认纯无线模式，启动档位 `vga12`（640×480）+ 锁定 `jpegQuality=30`，工具栏支持实时切换分辨率与硬件 JPEG 质量。

## 核心模块与跟踪逻辑

- **`yolodetector.{h,cpp}`**：封装 YOLO26n ONNX 推理，支持标准正向与 90° 旋转兜底检测（适配完全倾倒的瓶体/柱状目标）。
- **`targettracker.{h,cpp}`**：协同 YOLO26n 语义定位、OpenCV CSRT 帧间跟踪与 ORB 关键点丢失重找，配合旋转不变形状门控、异常帧 HOLD 缓存、非对称尺寸滤波、360 ms 窗口速度与固定机位 ROI 像素变化门控，防止静止目标框抖误报移动与旁路背景误锁。
- **`targetstatetracker.{h,cpp}` & `monitorcontroller.{h,cpp}`**：实时输出各视角与三视角融合后的三维状态（**是否移动过 / 尺寸变大变小 / 是否倾斜或倾倒**），结合防抖迟滞与失联 freshness 检测驱动右下角状态横幅、三状态胶囊、分路卡片及事件日志。
- **`mainwindow.{h,cpp}` & `videowidget.{h,cpp}`**：三路等比画面显示、左键拖拽框选与三路联动锁定、看板标签栏右侧**清除全部**快捷按钮，以及精简后的**参数设置**页（倾斜报警门限、倾倒判定门限、尺寸突变门限、位移偏离门限、运动轨迹显示与画面清晰度增强）。

## 回归测试与实机验收

```powershell
# 在 host/ 目录构建并部署主程序与回归测试
.\build.cmd release DisplacementTracker.pro
.\build.cmd release tests\core_test.pro
.\build.cmd release tests\monitor_test.pro
.\build.cmd release tests\profile_switch_test.pro
.\build.cmd release tests\profile_live.pro
.\deploy.ps1 -Executables @('DisplacementTracker','core_test','monitor_test','profile_switch_test','profile_live')
```

- `monitor_test.exe`：目标框选与 YOLO/CSRT 跟踪、防静止抖动、移动/缩放/倾倒三态判定、跨视角融合、防抖与界面布局回归（**29 通过，0 失败**）。
- `core_test.exe`：UDP/VID1 协议、组帧、恢复、无线高清质量切换与色块检测回归（**29 通过，0 失败**，1 个可选联机用例跳过）。
- `profile_switch_test.exe`：三路分辨率切换、配置确认与持久化回归（**5 通过，0 失败**）。
- `profile_live.exe`：三路真实无线窗口帧率与稳定性验收工具。

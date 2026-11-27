# 三视角固定机位藏品监测上位机

当前主线为**无下位机、三路固定机位 ESP32 无线图传 + 藏品异动监测**，总览见根 [README.md](../README.md)。

## 纯无线视频链路说明

- 上位机 `DisplacementTracker.exe` 仅接收来自 ESP32-CAM 的无线 UDP 视频流（VID1 协议，端口 `5006`），无任何本地摄像头（`VideoCapture` / `QCamera`）或 USB 串口视频输入。
- `CameraRuntime` / `CameraSerialSession` 仅在配置了 `serialPort` 时读取 115200 波特率文本诊断日志并检测 CH340 在线；若改用独立 5V 供电，可清空 `camera-runtime.ini` 中的 `serialPort`。
- 串口枚举、周期 WLAN 流媒体模式配置与网卡枚举均在后台线程执行，不阻塞三路画面绘制。

## 构建、部署与运行

依赖：Qt 6.10.1 MSVC2022 64 位、OpenCV 4.12。

- 日常启动：双击 `run.cmd`（自动增量构建、部署依赖并启动 `build/run/DisplacementTracker.exe`）。
- 配置文件：`host/camera-runtime.ini`（部署时会同步到 `host/build/run/camera-runtime.ini`）。当前配置为纯无线模式（未配置 `serialPort`），对应 `Redmi K70E` 热点，ID1/2/3 默认 IP `10.166.101.145 / .157 / .129`，默认启动档位为 `vga12`（640×480）+ `jpegQuality=30`（工具栏支持实时切换锁定 `30 / 24 / 20 / 16` 或自动 `18～42`）。

## 监测操作与三视角融合逻辑

1. **锁定目标**：在三路实时画面中**左键拖拽框选目标区域**（推荐，支持多尺度模板匹配 + 局部 Lab 背景对比轮廓细化，覆盖大幅移动、变大、变小、倾斜、消失与恢复），也可单击轮廓内部或切换 HSV 色块模式。
2. **标定基准**：看到三路绿色目标框后，点击顶部**“一键全机位锁定基准”**或各路**“标定”**。
3. **异动输出**：
   - 单路变化按 500 ms 触发防抖、1000 ms 恢复防抖；超过 1 秒无新帧自动标记视频不可用。
   - 右下角融合看板区分**单视角预警（1/3 路）**与**多视角联合确认告警（≥2/3 路）**，自动识别“某视角缩放 + 侧视角位移 = 目标空间移动”。
   - 支持**异动声音提示**、**多视角确认告警自动截图存证**（输出到配置目录旁 `alerts/`）以及一键**导出日志**。

## 回归测试与实机验收

```powershell
# 在 host/ 目录构建并部署回归测试
.\build.cmd release tests\core_test.pro
.\build.cmd release tests\monitor_test.pro
.\build.cmd release tests\profile_switch_test.pro
.\build.cmd release tests\profile_live.pro
.\deploy.ps1 -Executables @('DisplacementTracker','core_test','monitor_test','profile_switch_test','profile_live')
```

- `core_test.exe`：UDP/VID1 协议、组帧、恢复、无线高清质量切换与色块回归（29 通过，0 失败，1 联机用例跳过）。
- `monitor_test.exe`：单点选轮廓、ROI 拖拽框选跟踪（静止/移动/变大/变小/倾斜/消失/恢复）、近对称主轴屏蔽、防抖、新鲜度、单/多视角分级融合回归（20 通过，0 失败）。
- `profile_switch_test.exe`：三路分辨率切换、配置确认与持久化回归（5 通过，0 失败）。
- `profile_live.exe`：三路真实无线窗口实测验收器（实测记录见 `output/v25-museum-20261123/优化与实测报告.md`）。

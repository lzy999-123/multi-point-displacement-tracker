# 上位机（Qt 6 + OpenCV）

## 构建

依赖：Qt 6.10（MSVC 2022 64 位）、OpenCV 4.12（默认 `D:/opencv_build/build/install`，可用环境变量 `OPENCV_DIR` 修改）。

- Qt Creator：打开 `DisplacementTracker.pro`。
- 命令行：`build.cmd release`，输出在 `build/DisplacementTracker-release/release/`。

运行时需要 Qt 和 OpenCV 的 `bin` 目录在 PATH 中（Qt Creator 运行时自动处理 Qt 部分）。

## 使用

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

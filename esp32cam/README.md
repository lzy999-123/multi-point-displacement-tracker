# ESP32-CAM 无线图传固件

当前主线固件为 `adaptive-udp-v26`，运行于 3 台普通 **ESP32-CAM（ESP32-D0WD-V3 + OV3660，4MB Flash / 4MB PSRAM）**，经 2.4 GHz Wi-Fi 通过自定义 UDP 协议（VID1，端口 `5006`）向 Qt 上位机持续推流。

## 核心特性（`adaptive-udp-v26`）

- **传感器与画质优化**：24 MHz XCLK + 配套 PLL 乘数 25 / PCLK 分频 10，原生满速采集约 **27.7 fps**；优化 OV3660 零延迟硬件 ISP 参数（`brightness=0, contrast=1, saturation=0, sharpness=2, denoise=1, lenc=1, bpc=1, wpc=1`），消除原厂默认发灰雾感。
- **动态分辨率与质量切换**：上位机通过 `VREQA3` 实时切换 `320×240` / `640×480`（默认）/ `800×600` 原生硬件 JPEG 档位，并通过 `VREQ02` 实时调整硬件 JPEG 质量（锁定 `30 / 24 / 20 / 16` 或自动 `18～42`，默认 `640×480` 锁定 `q=30`），切换完成后校验 JPEG SOF 尺寸并回发 `VCFG`。
- **三机共享热点抗拥塞**：针对 3 台设备共用 2.4 GHz 手机热点，内置分档平滑节拍底限、8 槽 PSRAM 帧队列、机位相位错峰（`DEVICE_ID * 12 ms`）、`ENOMEM` AIMD 自适应退让与首片拥塞早退机制。
- **固定机位编号与自动发现**：通过 `DEVICE_ID`（1～3）与上位机画面框 1～3 严格绑定，支持 UDP 广播发现与上位机热插拔重连，不依赖 USB COM 口顺序。

## 目录结构

- `displacement_cam/displacement_cam.ino`：主固件入口（采集任务、UDP 推流、设备发现与控制命令处理）
- `displacement_cam/config.h`：设备编号（`DEVICE_ID`）、端口与无线参数配置
- `displacement_cam/camera_pins.h`：板级摄像头引脚映射（AI-Thinker ESP32-CAM / 兼容旧 ESP32-S3）
- `displacement_cam/udp_video_profile.h`：分辨率与 JPEG 质量档位定义
- `displacement_cam/udp_send_policy.h`：AIMD 拥塞控制与帧发送预算策略
- `displacement_cam/secrets.example.h`：Wi-Fi SSID 与密码模板
- `build.ps1`：固件编译与烧录脚本

## 编译与烧录

工具链安装在 `D:\esp32`（Arduino-ESP32 `3.3.12`，FQBN：`esp32:esp32:esp32cam:FlashMode=dio,FlashFreq=40`）。

1. 将 `displacement_cam/secrets.example.h` 复制为 `displacement_cam/secrets.h` 并填写 2.4 GHz Wi-Fi 名称与密码（该文件已被 `.gitignore` 忽略）。
2. 为三块板分别传入 `-DeviceId 1` / `2` / `3` 编译并烧录（切勿把同一 `DEVICE_ID` 的固件烧给多块板）：

```powershell
# 仅编译（示例：设备 1）
.\esp32cam\build.ps1 -Board ESP32Cam -Framework 3.3.12 -DeviceId 1

# 编译并烧录到指定串口（烧录前先关闭串口监视器与占用串口的程序）
.\esp32cam\build.ps1 -Board ESP32Cam -Framework 3.3.12 -DeviceId 1 -Flash -Port COM5
```

> 产物默认输出到 `D:\esp32\build-out\displacement-cam-ov3660-3.3.12\bin`（使用纯 ASCII 路径避免链接器中文路径问题），同级 `build-info.json` 记录构建版本与源码/BIN 的 SHA256（不含 Wi-Fi 密码）。若需兼容早期 ESP32-S3 测试板，可使用 `.\esp32cam\build.ps1 -Board S3`。

## 使用注意

- **独立稳定 5V 供电**：供电不足在开启 Wi-Fi 时会触发 `Brownout detector was triggered` 导致掉电重启；USB 线仅作供电、烧录与可选 115200 波特率诊断日志，正式推流完全走 Wi-Fi。
- **编号唯一**：三台设备的 `DEVICE_ID`（1～3）不可重复；上位机发现冲突时会提示并仅连接先响应设备。
- **单客户端推流**：每台设备同一时刻仅向当前请求的上位机推流，排查网络或串口时请先关闭上位机。

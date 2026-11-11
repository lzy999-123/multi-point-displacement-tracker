# 2026-11-06 软件验证记录

对象：`MDK-ARM/displacement_tracker.uvprojx`，目标 `displacement_tracker`。
硬件选择：用户确认普通 PWM 舵机；F103C8T6、8MHz 晶振、180°舵机映射为当前开发默认配置，仍需核对新实物。

## 最终构建

运行 `stm32/build.ps1`，Keil ARM Compiler V5.06 update 5 (build 528) 全量构建：

- 0 Error(s), 0 Warning(s)
- Code=10380，RO-data=340，RW-data=16，ZI-data=6568 字节
- 输出：`stm32/build/firmware/displacement_tracker.hex`
- SHA256：`F59E961256382D697DE8573C961EA6426A66936FB878E5E21C61AE1F5EFA8ABE`
- 日志：`stm32/build/keil-build.log`，编译产物和日志未纳入 Git

## 控制与协议测试

运行 `stm32/tests/build.cmd`，MSVC `/W4 /WX /MT` 编译真实 `Core/Src/tracker.c` 与 C 测试程序。
最终 **17 项场景全部通过**：启动等待新数据、CRC 已知向量与线格式、坏帧重同步、坏帧内嵌合法帧恢复、
无效消息不续链、截断帧超时恢复、限位与脉宽、手动方向和模式命令、超时保持且不自启、迟到数据不能掩盖失联、
控制节拍/死区/单步限幅、ACK 忙时保留及遥测、ACK 队列满拒绝执行、三路状态隔离及接收状态重置、
270°映射及非法配置拒绝、毫秒计数溢出、100000字节随机坏流恢复。

程序输出：`17 scenarios passed (real tracker.c; no hardware attached).`
软件测试验证配置下的方向符号关系，不能替代安装后手动方向验证。

## 来源与边界

- 复核 `reference_sources.json` 的 124 个源文件 SHA256，参考目录源文件保持一致；未改动参考项目。
- 复核 Keil 源文件均在本项目内，不含 `main_gimbal.c` / FOC 入口和外部源码依赖。
- `git diff --check` 通过。
- 未烧录、未启动电机，未测量 PWM 时序、真实角度、温度或供电；三路串口并发实机行为未验收。
- ESP32 控制桥与 Qt 控制层尚未实现；本轮交付为 STM32 固件及软件验证，不代表完整系统实物闭环完成。

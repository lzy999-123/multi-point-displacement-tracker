# ESP32-CAM 实验室传图问题：GitHub 原始资料核查

日期：2026-11-11。用户要求检查 GitHub 解决方案。当前主控实读为普通 ESP32-D0WD-V3（COM5）+ OV3660；另一块为 ESP32-S3（COM6）+ GC2145。通道配置保持 ESP32-CAM=1、S3=2。

本轮普通网页搜索工具返回503，因此改为直接访问GitHub官方API，读取仓库源码、issue正文、完整评论及当前状态。原始响应归档在被忽略的 `tmp/github-cam-20261111/`，不把搜索摘要当作已验证修复。

## 最相关的具体对照：8MHz XCLK

[arduino-esp32 #5834](https://github.com/espressif/arduino-esp32/issues/5834) 记录部分ESP32板在LEDC/I²S输出5/10/20MHz时WiFi吞吐恶化，作者报告8MHz改善；[作者的修复说明](https://github.com/espressif/arduino-esp32/issues/5834#issuecomment-964598319) 明确针对OV2640搭配8MHz和传感器倍频。[后续使用者反馈](https://github.com/espressif/arduino-esp32/issues/5834#issuecomment-1883068251) 也报告8MHz改善。以上为他人的硬件观察和配置变通，不代表乐鑫已合入一个适用于所有ESP32-CAM的驱动补丁，更不证明本机OV3660已经适用。

此前本机10/16/20MHz探索未通过，8MHz尚未实际对照。当前OV3660不能照搬OV2640寄存器。核对[官方OV3660驱动](https://github.com/espressif/esp32-camera/blob/master/sensors/ov3660.c)，Git blob `1e5dbe0a81ef1f14d94872eb657ed1cca4f71c38`，其时钟公式为：VCO由XCLK、倍频和预分频得到；PLL输出/4为SYSCLK，PLL输出/2再除PCLK分频为PCLK。

本轮隔离构建采用8MHz XCLK、OV3660公开 `set_pll` 参数 `(0,25,1,0,0,0,1,10)`。按驱动公式计算SYSCLK=50MHz、PCLK=10MHz，与默认20MHz/倍率30/预分频编码3的计算值一致。**这是由驱动公式推导的候选配置，不是已测量时钟或帧率。** 保持320×240、硬件JPEG；需初始化成功后读回寄存器、收到真实图像并完成Qt逐秒验收，才能判定有效。

构建与写入通过，固定副本及哈希见 `output/实验室网络实测-20261111/github-8mhz-cam/build-info.json`。测试入口 `tmp/github-cam-20261111/check_8mhz.py` 在联网阶段将传感器保持PWDN状态，先区分认证失败和摄像头启动后退化。若WiFi未完成，摄像头及8MHz PLL路径尚未执行，不能将该结果表述为“8MHz也无效”。实际终态见同目录result.json。

本次实测终态：75秒等待后仍未完成联网，记录29次断连事件，没有PLL读回，也没有进入视频测试。因此8MHz改善假设尚未检验。已写回本次试验前的COM5单缓冲HTTP、65秒联网等待版本，应用SHA256 `7AE297A15BD75BA4B572B3F151DC0CE39F179E2F7411AE9935679E4867A20306`；恢复日志 `github-8mhz-cam/restore-previous-cam.log` 显示写入摘要校验成功。S3未在本轮改写，继续保持编号2；ESP32-CAM编号1。

## 相机运行与WiFi共存

[esp32-camera #662](https://github.com/espressif/esp32-camera/issues/662) 中原作者首先报告S3初始化摄像头后网络恶化，后续讨论也涉及普通ESP32和时钟干扰。[相关评论](https://github.com/espressif/esp32-camera/issues/662#issuecomment-2211762627) 报告外置天线改善，但另有使用者无法复现。因此不能把issue关闭状态理解为一个统一软件修复已经合入，也不能据此直接要求用户买天线。

[esp32-camera官方说明](https://github.com/espressif/esp32-camera#readme) 明确指出YUV/RGB对PSRAM压力较大，WiFi启用时尤为明显，建议需要RGB时先采JPEG再转换；多缓冲连续采集增加CPU/内存负担。当前OV3660具备硬件JPEG，优先保留JPEG输入，不能套用GC2145的软件压缩路径。

## 可供网络与协议实现参考的完整项目

- [rzeldent/esp32cam-rtsp](https://github.com/rzeldent/esp32cam-rtsp)：RTSP、HTTP MJPEG和快照，README确认RTSP媒体仍是JPEG over RTP，可用UDP或TCP。它可作为标准协议/配置界面参考；不能因项目提供RTSP就推导本机WiFi认证与吞吐问题必然解决。
- [s60sc/ESP32-CAM_MJPEG2SD](https://github.com/s60sc/ESP32-CAM_MJPEG2SD)：明确支持OV3660，并有路由器联网和MJPEG实现。README的OV5640/OV3660章节还提醒部分为OV2640设计的底板在替换传感器时存在温升/供电适配问题。该项目说明仅是检查线索，本机尚无温度或电压测量，不能直接认定故障就在供电或温升。

本轮没有运行第三方固件或安装整个项目。更换协议和复制整仓不是当前已证明的修复；优先验证8MHz候选的实际适用性，并独立定位摄像头尚未启动时的实验室WiFi认证问题。完整目标仍为实验室网络内真实传图、持续显示和原有320×240/20fps验收。

## 继续实测：PMF关闭与8MHz已进入真实传图路径

用户确认ESP32-CAM使用板载天线。本节补充后续结果，不覆盖上文第一次未进入相机初始化的试验。

相机关闭的同源兼容性对照：Arduino-ESP32 3.3.12取得`192.168.110.96`，串口确认收到电脑的4、512、1400字节UDP并调用回包，电脑每种长度5次均无应答；2.0.17在b/g/n和b/g各35秒内均未取得IP。因此降级未解决问题，关闭PMF也没有证明能恢复有效回传。完整串口和汇总在`output/实验室网络实测-20261111/sta-core-comparison-confirmed/`；先前未等待完整串口行的对照不作为结论依据。

后续8MHz候选同时采用扫描后固定BSSID、15dBm和PMF关闭设置，保持传感器PWDN直到联网完成。此次约1.4秒取得IP，确认PID=0x3660、PSRAM=4194304字节，PLL API返回0、寄存器读回`0,19,11,0 / pclk=a`（十六进制）。这些是API和寄存器证据，未使用示波器测量物理时钟。

| 测量 | ESP32-CAM，通道1 | ESP32-S3，通道2 |
| --- | --- | --- |
| HTTP源流25秒 | 2张完整JPEG，均320×240；首帧22703.65ms | 本项未测 |
| 真实Qt双路60秒，新绘制帧总数 | 50 | 293 |
| 预热后最低逐秒绘制数 | 0 | 0 |
| 预热后最长无新绘制间隔 | 34605ms | 20767ms |
| Qt重连状态 | 有 | 有 |
| 20fps/150ms验收 | 未通过 | 未通过 |

摄像头启动后的首次源流窗口没有记录WiFi断连，但实际TCP图像传送严重停顿；后续Qt串口出现`httpd_resp_send_chunk`失败、`errno=11`。不能把WiFi保持关联解释成视频稳定。抓包已看到零星相机TCP回包，也不能据此认定回程正常。

候选应用SHA256：`e92d512e2a77429d6861998bc92221361f0aa49b29ceb11c1682a9bf3dcedd8b`。固定二进制在`D:/esp32/build-out/ov3660-pmf-off-8mhz-20261111/frozen/`；配置和源码副本（不含secrets.h）、构建清单、源流结果、实际图像、Qt双路逐秒计数和串口记录在`output/实验室网络实测-20261111/pmf-off-8mhz-cam/`。

结论：8MHz候选已收到真实图像，但没有达到稳定流畅要求；本次同时调整联网和HTTP实现，亦不能将偶发图像归因于8MHz单项。它作为实验固件保留，不作为已验证修复推广。

## 摄像头关闭的回程与临时MAC对照

同一份3.3.12诊断固件，传感器PWDN保持HIGH，无摄像头初始化。原始MAC取得`.96`；临时使用本地管理MAC`7E:D5:44:33:39:9C`后取得`.44`。两个阶段均完成DHCP，但到网关TCP/80、到电脑TCP/18081分别在约2503ms超时，电脑到板TCP/81各3次超时，UDP的4、512、1400字节各5次均无回包。原始MAC阶段串口确认收到请求并调用回包；电脑反向测试监听器没有收到板端连接。

证据：`output/实验室网络实测-20261111/sta-return-path/result.json`及串口日志。反向TCP可能受电脑入站策略影响，不能单独用它证明路由器故障；同时存在的网关TCP失败也不能用Qt或摄像头编码解释。临时MAC及IP未写入正式配置，测试结束写回视频固件，烧录器确认设备恢复出厂MAC。

这些结果不支持继续把“换视频协议”“更换Qt解码器”作为当前首要修复；也未证实AP固定终端隔离、板载天线损坏、供电异常或具体RF干扰源。下一项需要实体位置对照：相同固件、同一实验室SSID、同一摄像头，改变ESP32-CAM与电脑金属外壳及另一块板的距离后重测。测试前后需分别记录，不将不同固件的差异算作位置收益。

位置对照随后已执行：用户确认将ESP32-CAM移开电脑和S3；未远程测量实际间距。保持上述8MHz候选应用哈希及DH4504，复位后观察88秒，仍未完成联网和图像服务启动，串口出现认证断连；尚未执行相机初始化及PLL配置。因此本次没有进入源流25秒或Qt60秒测量，不能写成“移动后视频0fps”或认定移动必然无效。记录见`output/实验室网络实测-20261111/position-separated-20cm/`。当前COM5留在同一实验候选视频固件，COM6未改写，第一路ESP32-CAM、第二路S3配置仍保留；当前状态以`current-status.json`为准。

当前需要实体天线端和供电接线的检查资料，才能核实板载/外置天线的硬件选择及实际供电方式；没有照片、电压测量或其他实物证据之前，不认定已找到硬件根因。实验室网络稳定传图目标仍未完成。

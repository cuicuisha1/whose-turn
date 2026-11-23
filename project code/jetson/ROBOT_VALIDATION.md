# 机器人软件 v0.2.1 验证记录

日期：2026-10-28。主机为 Windows AMD64，Python 3.11.4、NumPy 2.4.3、OpenCV headless 4.10.0.84。Jetson 推荐依赖组合和硬件运行仍未验证。

- **v0.2.1 的 60 项测试全部通过，0 失败，0 错误，0 跳过。** 包含既有 24 项视觉测试、29 项架构测试及复核新增的 7 项故障回归。记录见 tests.log（`../build/robot/tests.log`） 和带执行源码/配置/HTML 哈希的 tests_report.json（`../build/robot/tests_report.json`）。
- 集成测试实际启动视觉 Python 子进程和临时 HTTP 服务；验证了有界命令队列、目标确认、旧帧/未来帧/过期数据、深度缺失、目标换代、暂停恢复、故障锁存与退出后的目标清空。
- 用故障子进程验证崩溃后的有限重试、重试耗尽与无消息超时后的进程回收；模拟输入没有接触相机或执行机构。
- Arduino 适配器通过注入客户端验证仅发送 `status`，并实际连接现有原生固件模拟器，确认上电 `FAULT / BootReset=1` 原样保留，没有发送清故障或动作命令。
- HTTP 验证了页面/API 成功响应、禁缓存、非法路径 404 和 POST 不可用。本地沙箱阻止回环连接，因此该项通过批准后的本机测试执行；不访问外网。
- 浏览器实际打开本机状态页并检查布局，已看到持续更新的 `TRACKING`、目标深度和虚拟云台角度，页面明确显示“模拟输入 / 虚拟执行”。
- 初版 v0.2.0 的 9.015 秒交互演示：181 次主循环、173 次有效观测、163 次有关注目标；经过 `STARTING → PERSON_DETECTED → TRACKING → PAUSED → PERSON_DETECTED → TRACKING → STOPPED`，零视觉重启、零清理错误、退出码 0。模拟过程含遮挡与深度失效，暂停期间仍持续采集。
- 本机缺少 `pyrealsense2` 的真实相机配置演示：2 次有限重启后报告 `vision_process_failed`，退出码 2，0 次有效观测、0 个目标；没有切换为合成输入，清理无错误。

本地演示原始记录位于完整项目的 `build/robot/demo/20261028T162700Z_e2cbd166/`；缺失 SDK 的故障记录位于 `build/robot/missing_camera/20261028T162700Z_a4fcc953/`。它们用于本机复查，不随轻量 ZIP 打包。

**未完成实机验收**：Jetson 启动/性能/温度/长期资源占用、真实相机取流和拔插恢复、人体检出率、真实串口通信与运动标定、麦克风/ASR/TTS、手机胸屏部署。模拟角度和本机循环次数不能作为上述验收证据。原 Arduino 源码保持当前版本，未烧录或启用运动。

v0.1 视觉验证历史保留在 [VALIDATION.md](VALIDATION.md)。当前命令 `python -m jetson.check` 输出到 `build/robot`，不会覆盖原 `build/vision` 历史报告。

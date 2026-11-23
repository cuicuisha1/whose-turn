**当前研究原型是独立 [whoseturn](../whoseturn/README.md)；本入口仍为视觉、虚拟头部及文本状态页。**

# 机器人软件 v0.2.2：统一运行入口

v0.2.2（2026-11-10）移除底盘模块、轮速字段和 chassis_policy，状态 schema 升至 2；实体模型保留四轮。原 v0.2.1 的代码复核修复退出信号锁竞争、输出失败后的收尾和串口关闭状态，详见 [CODE_REVIEW.md](CODE_REVIEW.md)。

2026-10-28，《靠近之前 / Before We Approach》。这一版把已完成的视觉程序接入机器人主循环，形成可运行、可测试、可继续扩展的软件架构。**当前可运行的是感知、行为决策、虚拟执行、文本交互和只读显示；真实头部执行、麦克风、ASR/TTS 仍未接通；导航已退出本轮项目范围。**

在包含 `jetson`、`host` 文件夹的 **`project code` 目录**运行下列命令。ZIP 解压后的根目录同样适用。Python 需要 3.10+。

## 立即运行

基础模拟只依赖 NumPy，不需要连接相机、Arduino 或下载模型。已有环境可先检查 `python -c "import numpy; print(numpy.__version__)"`；新环境按[视觉说明](README.md)安装基础依赖。

```bash
python -m jetson.robot --duration 10
```

程序启动后，在**运行它的电脑上**打开 `http://127.0.0.1:8765/`。可以看到目标位置、相机深度、行为状态、虚拟云台角度和模块状态。默认 `simulation.json` 使用合成 RGB-D 数据，并周期性制造目标消失和深度缺失。页面明确标识模拟输入；绿色矩形的检测结果是测试夹具，不代表模型检出能力。

交互演示：

```bash
python -m jetson.robot --scenario jetson/scenarios/demo.json --duration 12
```

脚本在第 2 秒输入“你好”、第 4 秒暂停、第 5 秒恢复、第 6 秒询问距离、第 9 秒退出。事件从主程序启动计时；慢速设备若尚无有效视觉，文本事件会被忽略。也可以开控制台：

```bash
python -m jetson.robot --console
```

依次输入 `text 你好`、`text 你能看见我吗`、`text 我离你多远`、`pause`、`resume`、`quit`。文字回复显示在状态页，未进行声音播放。`reset` 仅清除原因已经消失的**主程序故障锁存**，不清除 Arduino 故障，不重启已失败的视觉进程，也不恢复暂停状态。视觉重试耗尽后程序退出，需要修复原因再重新启动。

参数：

| 参数 | 含义 |
| --- | --- |
| `--config 路径` | 指定 JSON 配置，默认 `config/simulation.json` |
| `--duration 10` | 运行 10 秒；默认 0 表示持续运行 |
| `--no-web` | 不开启 HTTP 状态页 |
| `--port 8766` | 覆盖状态页端口；0 由系统分配，实际地址写在启动输出中 |
| `--output-dir 路径` | 运行日志根目录；相对路径从代码根目录解析 |
| `--scenario 路径` | 重放按 `at_s` 升序排列的事件列表 |
| `--console` | 接受终端文本命令 |

正常退出码为 0；没有收到有效视觉、运行中出现锁存故障、配置/进程/清理失败时为 2。对本地模拟的退出码 0 不能解读为实机已通过验收。

## 连接 Jetson 与相机

Jetson 当前环境仍未确认，先在设备上执行：

```bash
python3 -m jetson.diagnose --output jetson/runs/environment_jetson.json
```

依照[视觉说明](README.md)确认 Python、OpenCV、RealSense SDK 与相机权限，先单独验证视觉：

```bash
python3 -m jetson.main --duration 60 --stdout
```

然后运行整合版：

```bash
python3 -m jetson.robot --config jetson/config/hardware_observe.json --duration 60
```

此配置使用真实 RealSense + OpenCV HOG，执行层仍是虚拟设备。多相机时需要在配置中明确填写 `vision.serial`。相机或 SDK 失败会报告错误，不会自动切换为合成输入。YOLO 需要已有本地模型及与 Jetson 匹配的运行环境；模型路径相对配置文件目录解析。

远程查看时可通过已有 SSH 连接转发端口：`ssh -L 8765:127.0.0.1:8765 用户名@Jetson地址`，再在电脑浏览器打开本机端口。也可自行复制配置，将 `interface.host` 改成 `0.0.0.0`，在可信局域网内打开 `http://Jetson地址:8765/`。后者没有身份验证，适用于开发状态显示，不适合暴露到公网。

## Arduino 接口

默认 `arduino.mode` 为 `disabled`，不打开串口。需要读取底层状态时，安装已有主机工具的依赖 `python -m pip install -r host/requirements.txt`，复制硬件配置并添加：

```json
"arduino": {
  "mode": "monitor",
  "port": "/dev/serial/by-id/替换为实际设备名称",
  "poll_hz": 2.0,
  "timeout_s": 0.3,
  "stale_after_s": 1.5
}
```

Windows 使用实际 `COM` 端口。串口由监控线程独占，每次只发送现有协议的 `status` 请求，不建立运动会话、不发心跳、不自动重连。不要同时让 Arduino IDE 串口监视器、手动控制台和此监控器占用同一端口。

现有固件上电进入 `FAULT / BootReset=1`，监控会如实报告并让主程序进入 `FAULT`。不会为了演示自动执行 `hello / clear_fault / prepare / motion / servo / stop`。实机调试仍通过原来的 [host 工具](../host/robot_test.py) 和固件验收流程独立完成。主程序 `reset` 不等于底层清故障。

## 配置、日志与数据接口

- 配置按 `runtime / vision / behavior / control / arduino / interface / audio` 分区；未知字段、重复键、错误类型、NaN、越界参数会在启动前报错。默认参数及限制见 [config.py](core/config.py)。
- 输出在 `jetson/runs/robot/时间_随机ID/`；包含 `config.json`、`latest.json`、`state.jsonl`、`events.jsonl`、`summary.json` 与 `vision_worker.log`。JSONL 和工作进程日志按大小轮换；每个新运行仍会创建新目录，长期部署需要自行管理历史运行目录。
- `GET /api/status` 与顶层 `latest.json` 具有相同主体结构。HTTP 会独立检查数据年龄并清除过期目标。网页仅显示状态，没有远程动作接口。
- 离线 `latest.json` 是记录文件，消费者必须自己检查 `updated_monotonic_s` 和 `perception_expires_at`；主进程被强制终止时文件不会继续更新。`vision/` 内工作进程日志仅用于诊断，不能当作执行输入。
- 时间戳来自同一台主机的单调时钟，只在本次主机启动内有意义。API 带有 `served_monotonic_s` 供浏览器计算剩余有效期，其他电脑不能直接把其数值与自己的时钟相减。
- 文本事件和字幕会写入本地状态日志；v0.2 不录制麦克风或相机视频，不调用云端服务。距离字幕反映提问时的测量，页面距离栏才是持续更新的当前值。

## 验证与继续开发

```bash
python -m jetson.check
python scripts/package_robot.py
```

检查包括原有视觉测试和新增架构测试，报告在 `build/robot/`。未安装 OpenCV 时相关测试明确跳过；单独解压包不含原生固件模拟器，相应测试会跳过。打包脚本要求执行源码、配置、HTML 与通过测试时的 SHA-256 一致；输出到 `output/robot_software_v0.2.1/`，旧 v0.1 与 v0.2 ZIP 保留。

本机复现全部检查时，OpenCV 安装在毕业设计根目录的 `tools/vision-test`：

```powershell
$env:PYTHONPATH = (Resolve-Path '..\tools\vision-test').Path
$env:PYTHONIOENCODING = 'utf-8'
python -m jetson.check
```

架构、模块协议和扩展顺序见 [ARCHITECTURE.md](ARCHITECTURE.md)。实测结果见 [ROBOT_VALIDATION.md](ROBOT_VALIDATION.md)，Jetson 实机清单见 [robot_commissioning.csv](robot_commissioning.csv)。

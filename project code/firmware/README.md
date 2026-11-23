# Arduino 头部控制｜head-1

2026-11-10。目标板仍为 Uno R4 WiFi，双舵机 D10/D11 与 USB 串口保留。物理模型保留四轮；当前固件不初始化、控制或读取轮电机、驱动板或编码器。原 N20/TT 驱动适配、CPR 和 drive 模式已退出本轮软件范围。

## 当前接线范围

| Uno 端 | 用途 |
| --- | --- |
| D10 | 水平舵机信号 |
| D11 | 俯仰舵机信号 |
| GND | 与舵机电源控制参考地共地 |
| USB | Jetson/电脑数据及 Uno 供电，115200 |
| 其他 GPIO，包括 A4/A5 | 本固件不使用，不连接轮驱动控制 |

舵机使用已有外部稳压支路，不由 Uno 5V 引脚提供动力。继续采用无电池拖线方向；按 [本轮硬件边界](../../03_硬件方案与接线/头部交互_四轮仅结构_2026-11-10.md) 接线并验收。轮电机动力应断开，驱动板控制线不接入当前系统；删除软件不会自动关闭旧驱动板。

## 启用条件

`config.hpp` 只保留 `powerWiringVerified` 和 `servoLimitsVerified`，默认均为 false。完成真实验收后填写记录再改配置、重新编译；限位默认 1400–1600 μs，不代表实测安全行程。

上电进入 FAULT，PWM 关闭。显式 hello、clear_fault 后可 prepare：
- `bench`：要求供电已验收，只检查通信，不允许舵机命令。
- `head`：再要求真实舵机限位已验收，进入 ARMED，生成新 arm_epoch。
- prepare 本身不输出 PWM；之后的 servo 才发送脉宽。

心跳或成功舵机指令保持 200 ms 通信有效期，主机约每 50 ms 发送心跳。心跳持续时，舵机可保持最近目标；不包含过去的轮速动作租约。USB 断开、通信过期、活动状态下非法帧/旧序号/错误 session/epoch、PWM 写入失败或循环超时将关闭 PWM 并锁存故障，不自动清故障或恢复动作。

`stop` 和 `status` 不要求 session。stop 关闭 PWM，保留已锁存故障。PWM 关闭不等于舵机掉电或实际机械停止，头部需要可靠支撑；物理开关仍负责切电。WDT 测试必须在非活动状态下显式声明 outputs_off，只测试复位请求，不证明切断外部电源。

## 调试与协议

```powershell
python scripts/build_firmware.py
python scripts/test_phase1.py
python host/robot_test.py --simulate console
python host/robot_test.py --port COM5 console
```

在控制台输入：

```text
status
clear
prepare head
servo 1500 1500
stop
quit
```

模拟配置的供电/限位标记为测试值；不可复制到生产配置。模拟故障注入为 `sim-servo-fail` / `sim-servo-restore`。

每行一个 JSON，最多 384 字节；拒绝重复/额外字段、非预期类型、嵌套对象与尾随 JSON。序号必须递增，不自动重试 ACK 超时的命令。示例中的 session/sequence/epoch 仅说明结构：

```json
{"type":"hello","session":12345,"sequence":1}
{"type":"clear_fault","session":12345,"sequence":2}
{"type":"prepare","mode":"head","session":12345,"sequence":3}
{"type":"servo","pan_us":1500,"tilt_us":1500,"session":12345,"sequence":4,"arm_epoch":1}
{"type":"heartbeat","session":12345,"sequence":5}
{"type":"stop"}
```

status 包含 `protocol_version:"head-1"`、状态、故障、session/epoch、最后命令脉宽、servo_active、pwm_output_disabled、供电/限位验收标记、心跳年龄和 WDT 信息。脉宽是最后命令值，不是角度反馈。主机与研究适配器检查 head-1；旧 motion、encoder_raw、prepare drive 指令及 wheels_rpm 字段均不再接受。接收活动期间的旧指令会触发故障并关闭 PWM。

工具链沿用 Arduino CLI、renesas_uno 1.6.0、ArduinoJson 6.21.5；不新增库。构建只编译，不烧录或访问串口。代码、原生测试、编译产物和证据可用 `python scripts/package_phase1.py` 重新打包；修改前四轮实现已归档。

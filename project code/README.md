# 当前软件｜头部交互，四轮仅保留实体结构

2026-11-10：Arduino 协议 `head-1`；统一软件 v0.2.2，状态 schema 2。实体模型保留四轮；软件不包含四轮运动、编码器、转速标定、里程计或导航开发任务。项目主线是《轮到谁？》的头部转向与参与协商。

在本目录运行：

```powershell
python -m whoseturn --demo
python -m whoseturn --demo --mode firmware_sim
```

第二条需要先构建原生模拟器；两者均不连接真实硬件。研究操作、语音预览与真机前提见 [研究原型说明](whoseturn/README.md)。供电、舵机机械限位和实验配置仍需实测，不因删除四轮而自动标为通过。

| 工作 | 入口 |
| --- | --- |
| 研究流程、头部动作与日志 | [whoseturn](whoseturn/README.md) |
| Arduino 草图 | [robot_controller.ino](firmware/robot_controller/robot_controller.ino) |
| 供电记录、舵机限位配置 | [config.hpp](firmware/robot_controller/config.hpp) |
| USB 协议与接线边界 | [firmware/README.md](firmware/README.md) |
| 串口调试工具 | [robot_test.py](host/robot_test.py) |
| 实机验收表 | [commissioning.csv](firmware/commissioning.csv) |
| 独立视觉 | [jetson/README.md](jetson/README.md) |
| 视觉、虚拟头部与文本状态页 | [jetson/ROBOT_README.md](jetson/ROBOT_README.md) |
| 修改与验证记录 | [本轮记录](四轮移除_验证记录_2026-11-10.md) |

构建与测试：

```powershell
python scripts/build_firmware.py
python scripts/test_phase1.py
python -m unittest discover -s whoseturn/tests -v
python -m jetson.check
python host/robot_test.py --simulate console
```

控制台使用 `clear` → `prepare head` → `servo 1500 1500` → `stop`。真实固件默认拒绝 prepare，需完成供电和舵机验收。bench 模式只允许通信检查，不接受舵机动作；prepare 不会自行输出 PWM 或让头部回中。

保留心跳、USB 断连、序号、session、arm_epoch、故障锁存与 WDT；软件 stop 关闭舵机 PWM，不等于切断供电或确认机械停止。草图中用户原有的受限串口发送方式保留。

工具链仍从本目录或毕业设计根目录的 `tools` 查找。构建证据位于 `build`，当前固件交付包用 `python scripts/package_phase1.py` 生成到 `output/current_source`。该包只含底层固件与调试工具；研究入口仍在本工作目录中。旧四轮软件快照位于上级淘汰目录，旧发布包均视为历史，不与 head-1 混用。

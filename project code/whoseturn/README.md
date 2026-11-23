# Whose Turn? 最小研究原型 v0.1.1

2026-11-10 修订：实体保留四轮，当前软件只控制头部，底层要求 head-1 固件；不再依赖轮电机板、I²C、CPR 或编码器标定。Python 3.10+；默认标准库运行，无硬件和声音。统一 `jetson.robot` 与独立视觉程序保持原路径；这里是单独的实验入口，不同时启动其他串口控制器。

在 `project code` 目录：

```powershell
python -m whoseturn --demo
python -m unittest discover -s whoseturn/tests -v
python -m whoseturn --demo --mode firmware_sim
python -m whoseturn --demo --audio wav --audio-dir whoseturn/audio_preview
python -m whoseturn --audio wav --audio-dir whoseturn/audio_preview
```

`firmware_sim` 使用已有 `build/tests/simulator.exe`（Linux 同名无扩展名），运行生产协议／状态机与 FakeHal，不连接 USB。`audio_preview` 是本机合成英语预览；可通过 `make_preview_audio.ps1` 重建，正式研究前需试听并冻结录音。Linux WAV 使用系统 `aplay`，尚未在 Jetson 验证。

交互控制台：`start` → 等待 `WAIT_A` → B 到场时 `join` → 等待回到 `WAIT_A` → 回应与协商结束后 `advance` → `WAIT_B` → `finish materials` / `finish interaction` / `finish skip`。随时 `mark B begins speaking` 记观察、`sync LABEL` 标记同步、`stop` 或 Ctrl+C 停止。明确请求帮助时可 `repair`；不用沉默自动触发。

`--variant explicit` 是形成性版本：确认前增加欢迎与轮次说明；默认 `head_only` 没有该语句。不把两个版本视为已验证优劣。

真机命令形式（**当前示例配置会拒绝**）：

```bash
python -m whoseturn --mode hardware --port /dev/serial/by-id/ACTUAL_DEVICE --config /path/to/verified_config.json --motors-disconnected --head-centred --audio wav --audio-dir whoseturn/audio_preview
```

`--motors-disconnected` 是保留实体轮电机时的物理断线确认，不产生轮控制命令。仅在实际完成头部标定、轮动力断开和头部对中后使用这些参数，不能把标志当作实测。固件 `config.hpp` 的供电和舵机限位验证门槛仍生效，不由程序自动解除。进入控制台后显式 `clear`、`prepare`，再 `start`。中断／故障后新开会话，不自动恢复或重放。

可加 `--vision-latest /same-host/path/latest.json` 读取独立视觉程序的原始 `perception` 状态，检查来源和帧龄后仅记日志。不是统一机器人入口的顶层 latest 文件；也不是 A/B 身份识别。主研究不依赖摄像机正确跟踪，失效须在分析中保留。

输出：每次在 `build/whoseturn/runs/` 新建目录，含 `run.json`（配置、台词、源码与音频哈希）、`events.jsonl`、`latest.json`、`summary.json`。默认上限 20 分钟；不自动录音录像、不自动推断人的理解。日志不自动删除，按批准的数据管理计划归档。

`measured_pose:null` 表示无角度反馈；插值结束只表示命令完成。`participant_study_completed:false` 表示软件不认证研究完成，模拟记录始终标注 `synthetic_scenario:true`。真实硬件运行也不自动把 `hardware_verified` 变为 true，验收需另有记录。

研究方案与操作规则见 [当前研究目录](../../01_论文与主题/WhoseTurn_2026-11-05/01_研究定位与论证.md)。

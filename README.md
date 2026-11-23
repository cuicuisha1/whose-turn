# Whose Turn?

Creative Robotics graduate project for the UAL Creative Computing Institute.

*Whose Turn?* stages an encounter between a robot and two visitors. While the first visitor is answering a question, a second visitor arrives. The robot briefly turns towards the newcomer, pauses, then turns back to the first visitor. It later turns towards the second visitor and gives them a turn to speak. The project asks how visitors tell whether a turn of the head acknowledges their presence or invites them to speak.

The mechanical head uses orientation and timing to cue attention. The prototype has two versions: `head_only` uses the brief turn alone, and `explicit` adds a spoken welcome and explains whose turn it is before the same movement. An operator cues the stages of the encounter from the console. The physical design retains its four wheels as part of the body, while the firmware controls only the head.

## Run the demo

From the repository root, run the following with Python 3.10 or later. The default demo uses a virtual head, requires no Python packages or hardware, and runs without sound.

```sh
cd "project code"
python -m whoseturn --demo
python -m whoseturn --demo --variant explicit
python -m unittest discover -s whoseturn/tests -v
```

The demo plays through a scripted encounter and ends in `DONE`. Each run saves a configuration snapshot, event log and summary under `project code/build/whoseturn/runs/`. These runs are marked as synthetic. To hear the included English preview prompts, add `--audio wav --audio-dir whoseturn/audio_preview` to the demo command. WAV playback uses built-in Windows support or `aplay` on Linux.

## Components and dependencies

| Component | Requirements and entry point |
| --- | --- |
| Study prototype | The [Whose Turn? operator guide](project%20code/whoseturn/README.md) covers both variants, the interactive console and log files. |
| Arduino head controller | The [firmware guide](project%20code/firmware/README.md) covers the Uno R4 WiFi, two servos and the `head-1` USB serial protocol. Install Arduino CLI and run `arduino-cli core install arduino:renesas_uno@1.6.0` and `arduino-cli lib install ArduinoJson@6.21.5`, then run `python scripts/build_firmware.py` from `project code` to compile. |
| Native firmware simulator | Use a C++17 compiler and set `ARDUINOJSON_INCLUDE` to the `src` directory of ArduinoJson 6.21.5. The Windows helper `python scripts/setup_native.py` can install Zig. From `project code`, run `python scripts/test_phase1.py`, then `python -m whoseturn --demo --mode firmware_sim` to exercise the production protocol against a simulated hardware layer. |
| Serial host tools | From `project code`, install `python -m pip install -r host/requirements.txt` for USB serial access. The firmware simulator runs without this dependency. |
| Jetson vision and dashboard | The [Jetson guide](project%20code/jetson/README.md) describes a separate RGB-D vision pipeline. Its [requirements](project%20code/jetson/requirements.txt) cover NumPy. The guide describes platform-specific OpenCV and RealSense setup. Run `python -m jetson.check` from `project code` for the offline checks. |

## Design and research materials

- [Creative Robotics presentation (English)](02_汇报与展示/WhoseTurn_CreativeRobotics_12页_2026-11-05/Whose_Turn_Creative_Robotics_12_Slides_EN.pptx)
- [Research plan, study protocol and bibliography (Chinese)](01_论文与主题/WhoseTurn_2026-11-05/README.md)
- [Pan–tilt head CAD preview](04_机械结构与CAD/摄像头云台_检查与优化_2026-11-09/预览/04_优化装配细节.png) and [chest display and speaker CAD preview](04_机械结构与CAD/MOX_胸屏与扬声器固定件_2026-11-18/预览/胸屏与扬声器装配.png)
- [Hardware scope and wiring notes (Chinese)](03_硬件方案与接线/头部交互_四轮仅结构_2026-11-10.md)
- [Software validation record (Chinese)](project%20code/四轮移除_验证记录_2026-11-10.md)

The available evidence covers software tests and firmware compilation. Physical commissioning and participant sessions are pending in the project records. The supplied firmware configuration keeps physical servo operation disabled until the power wiring and servo limits have been verified.

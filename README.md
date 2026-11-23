# Whose Turn? — MOX robot software

Creative Robotics graduate project at the UAL Creative Computing Institute.

MOX uses head movement to acknowledge a second visitor during an interaction with the first. In the fixed condition, it turns after a set delay. In the pause condition, it waits for a sustained pause in speech or a timeout. Both conditions use the same sequence: turn towards the second visitor, hold, then return to the first visitor. An operator marks the second visitor's arrival.

This repository contains MOX Phase 7.2 (v0.2.3). The program centres the robot head using a RealSense camera, plays prerecorded prompts, and records trial events. It can reacquire a face during centring after brief occlusion or movement.

## Run a simulation

From the repository root, run the following with Python 3.12. The simulations use the standard library and synthetic inputs.

```sh
cd robot_project
python -B -S mox_phase72/robot_phase72.py --mode simulate --scope full --condition fixed --scenario moving --tilt-sign -1
python -B -S mox_phase72/robot_phase72.py --mode simulate --scope full --condition pause --scenario moving --tilt-sign -1
```

Each run should reach `COMPLETED` and write its summary and event log under `robot_project/camera_test/phase72/`. The logs identify simulated runs.

## Run the offline tests

Run the full suite on Linux with Python 3.12 from `robot_project`. The camera process and audio preparation tests use Unix sockets and symbolic links.

```sh
cd mox_phase72
python -B -S -m unittest discover -s tests -v
```

## Hardware setup

Live mode uses a Jetson with a RealSense RGB-D camera, a head controller running firmware 2.0.0, and ALSA audio devices. In the Jetson Python environment, install compatible builds of NumPy, OpenCV with YuNet support, the RealSense Python bindings (`pyrealsense2`), PySerial, and WebRTC VAD. Install the ALSA tools `aplay` and `arecord`. Supply the YuNet ONNX model via `--model` or place it at `robot_project/models/face_detection_yunet_2023mar.onnx`.

From `robot_project`, run the preflight check before connecting the motion system:

```sh
python mox_phase72/robot_phase72.py --mode check --scope full
```

For a live trial, use `--mode live` with the calibrated configuration, the controller's serial port, the verified tilt direction, and a trial and pair identifier. Use `--help` to see the camera and audio options. The console accepts `STATUS`, `JOIN` when ready, `END` during observation, and `STOP` or `QUIT` to end an attempt. Software STOP leaves servo PWM active.

The simulations and tests cover the control flow with synthetic inputs. Live motion and participant observations require separate verification.

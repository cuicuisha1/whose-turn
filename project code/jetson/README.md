# Jetson 视觉程序 v0.1

**2026-10-28 更新：统一机器人软件已进入 v0.2.1，入口与运行方法见 [ROBOT_README.md](ROBOT_README.md)，分层架构见 [ARCHITECTURE.md](ARCHITECTURE.md)。** 本页保留独立视觉 CLI `python -m jetson.main` 的说明；下文“没有串口/执行接口”仅指这个独立视觉程序。当前全量检查输出在 `build/robot`，新交付包通过 `scripts/package_robot.py` 生成；旧 v0.1 ZIP 和验证报告为历史快照。

2026-10-23。用于《靠近之前》的 **Jetson Orin Nano Super + RealSense D435if** 首阶段视觉开发。入口是 `main.py`，相机模块是 `vision/realsense_camera.py`。在 `project code` 目录运行下面的命令。

当前已完成本地实现与离线测试。Jetson 的 JetPack、Python、SDK、CUDA 环境尚未确认，真实 D435if 取流、人体检出率、测距误差及 Jetson 帧率均待实测。程序没有串口、电机或舵机控制接口。

## 1. 已实现的功能

| 功能 | 行为 |
| --- | --- |
| RGB-D输入 | 默认请求640×480、30FPS彩色BGR和16位深度，深度对齐到彩色图 |
| 相机信息 | 输出序列号、固件、USB类型、深度比例、两路帧号与时间戳及其时钟域 |
| 稳定取流 | 独立采集线程、单帧缓存；不积累历史画面；超时后关闭并重建pipeline，重试间隔0.5–5秒 |
| 设备固定 | 首次连接锁定序列号；重连不自动改用另一台相机；多相机需指定`--serial` |
| 基础人体检测 | 默认OpenCV HOG自带行人检测器，无需下载权重；另有Ultralytics YOLO本地模型接口 |
| 距离 | 检测框内部躯干区域的有效深度中值；排除0、非有限值和距离范围外的值；检查覆盖率和四分位距 |
| 短时跟踪 | 空间位置/IoU关联、连续ID和中心平滑；短时遮挡保留ID但不输出旧坐标/距离 |
| 关注目标 | 初次选择有有效距离的最近目标，之后保持该ID；输出归一化中心和水平/垂直误差 |
| 可观察性 | 本地双画面预览、JSON实时状态、滚动事件/观测日志、运行摘要；可通过SSH无窗口运行 |
| 离线演示 | 明确标为`simulation`的合成数据，定期注入遮挡和深度缺失；不会在真机失败时自动启用 |

HOG适合先核对完整管线，对近距离半身、遮挡、侧身、小目标和光照变化的检出能力有限。它的分数是SVM margin，不是概率，因此输出`confidence:null`。人物交叉或长时间消失可能换ID；这不是身份识别系统。需要更好的检出效果时，选装YOLO并实测。

## 2. 先确认 Jetson 环境

把整个`project code`复制到Jetson，例如放到`~/robot_project/project code`。不要复制Windows的`.venv`、`build`或`tools`作为Linux运行环境。Jetson已有环境不需要为本程序重刷。

也可以只传输交付包`Jetson视觉程序_v0.1.zip`并解压到一个新目录。这个ZIP的根目录直接包含`jetson`，后续命令在该解压目录运行即可，其他操作相同。

在Jetson终端中：

```bash
cd ~/robot_project/"project code"
python3 -m jetson.diagnose --output jetson/runs/environment_jetson.json
```

这个命令只需Python标准库，读取L4T/系统版本、Python、依赖导入结果、USB树和SDK设备枚举，不开启相机流、不安装软件。找不到依赖也会产生报告。报告里的`hardware_stream_tested:false`是正常的。

本程序要求 **Python 3.10或更新版本**。若系统仍是3.8，先根据诊断结果选兼容的独立Python环境及SDK，不要直接替换系统Python。未确认JetPack版本前，不批量升级CUDA、PyTorch、TensorRT或相机固件。

## 3. 安装基础依赖

以下适用于具有Python 3.10+的Ubuntu/Jetson环境：

```bash
sudo apt-get update
sudo apt-get install -y python3-venv python3-dev python3-opencv python3-numpy libusb-1.0-0 usbutils
python3 -m venv --system-site-packages jetson/.venv
source jetson/.venv/bin/activate
python -m pip install -r jetson/requirements.txt
python -c "import numpy, cv2; print(numpy.__version__, cv2.__version__)"
```

使用`--system-site-packages`是为了保留Jetson系统中的OpenCV。NumPy约束为1.x，避免许多JetPack预装OpenCV与NumPy 2的ABI冲突。不要同时安装`opencv-python`和`opencv-python-headless`来覆盖系统OpenCV；headless包不能显示窗口。

如果当前虚拟环境没有RealSense Python绑定，可先检查是否有适合该Python/aarch64平台的官方二进制包：

```bash
python -m pip install --only-binary=:all: pyrealsense2
python -m jetson.diagnose --output jetson/runs/environment_jetson.json
```

若没有匹配wheel，按[RealSense官方Jetson安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)和[Python绑定构建说明](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md)选择匹配版本；RSUSB后端提供不修改内核驱动的构建路径。Python wheel不等于已经安装了Linux设备权限规则、`realsense-viewer`或SDK命令行工具。权限错误应按官方说明安装udev规则并重新插拔相机，不要把长期以root运行Python作为解决方案。

相机接Jetson的USB 3.x端口，使用支持数据传输的USB 3线。用`lsusb -t`核对链路；设备枚举成功不代表深度流成功。运行程序前关闭占用相机的Viewer或其他进程。

## 4. 分三步运行

### A. 没有相机也可先跑软件管线

```bash
python -m jetson.main --simulate --duration 10 --stdout
```

合成绿色框按预设轨迹移动，每8秒包含短时遮挡和一段深度缺失。模拟检测器读取的是测试夹具，**这不验证真实人体检测准确率**。没有OpenCV时也能运行无窗口模拟。

本地桌面有OpenCV时可观察测试画面：

```bash
python -m jetson.main --simulate --preview
```

### B. 先验证真实相机输入

```bash
python -m jetson.main --detector none --preview --duration 60
```

左侧是彩色图，右侧是对齐深度；黑色深度像素表示没有测量。相机模式`detection_enabled:false`，`person_detected:null`，不据此判断有没有人。默认深度有效范围是0.2–6米，仅为软件筛选范围，不是D435if的精度保证。

通过SSH运行时省略`--preview`：

```bash
python -m jetson.main --detector none --duration 60 --stdout
```

默认请求640×480@30FPS；必要时可显式改为`--fps 15`。程序不静默降低分辨率或切换到普通USB摄像头；若请求的profile不可用，事件日志会记录错误并重试。

### C. 加入真实人体检测、测距和跟踪

```bash
python -m jetson.main --detector hog --preview
```

画面标出人体框、短时ID、距离、躯干测量区和关注中心。没有有效深度时显示`depth unknown`，保留当前视觉位置但不生成假距离。按`q`、Esc或Ctrl+C退出。

默认上限为8次检测/秒、5次JSON输出/秒，采集独立运行。实际速率由硬件性能决定；推理时间也计入帧龄，超过500ms的结果被标记无效。若推理速度不足，先检查负载/模型并降低检测频率；不要仅扩大过期阈值来掩盖延迟。一次推理阻塞会延迟主线程输出，接收端还必须自行检查消息时效。

## 5. 可选：YOLO

适配器支持本地检测模型`.pt`及本机兼容的TensorRT`.engine`，通过类别名`person`选择目标。它不自动下载模型，也不在GPU失败后静默退回另一后端。

先依据[Ultralytics官方Jetson指南](https://docs.ultralytics.com/guides/nvidia-jetson/)配置与实际JetPack匹配的PyTorch/torchvision/Ultralytics。GPU环境未确认时不直接执行无约束的`pip install torch`。准备官方COCO人体检测权重，例如`yolo11n.pt`，放到`jetson/models`；模型许可按其发布方要求使用。

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
python -m jetson.main --detector yolo --model jetson/models/yolo11n.pt --device 0 --preview
```

要显式用CPU，把`--device 0`换成`--device cpu`。TensorRT引擎应在匹配目标GPU、TensorRT版本及输入大小的环境中生成；不能默认拿Windows或其他型号导出的引擎直接运行。v0.1本地验收没有运行YOLO、CUDA或TensorRT。

## 6. 输出数据与后续接入

每次运行新建`jetson/runs/<UTC时间_随机ID>/`：

| 文件 | 内容 |
| --- | --- |
| `run.json` | 本次启动参数、程序版本、Python版本 |
| `latest.json` | 原子替换的最新状态；退出时写入无效状态并清空目标 |
| `observations.jsonl` | 每行一个JSON；单文件约5MB，最多3个旧文件 |
| `events.log` | 连接、断线、重试与异常；单文件约2MB，最多2个旧文件 |
| `summary.json` | 帧数、有效/无效输出数、重连次数、近期推理平均耗时、退出码 |
| `last_preview.png` | 只有显式使用`--snapshot`时保存的最后诊断画面 |

默认不保存RGB/深度录像。日志上限按每次运行计算，多次运行的旧文件夹需自行归档。退出码0表示本次正常退出，2表示启动/运行错误或定时运行结束前没有当前有效结果；退出码不代表通过了长时间硬件验收。

关键字段：

| 字段 | 含义 |
| --- | --- |
| `valid` | 相机当前在取流、结果仍在帧龄阈值内、相机会话没有改变；不是检测准确率保证 |
| `simulated` / `source` | 明确区分真实相机与测试夹具 |
| `person_detected` | true/false仅用于有效且启用检测的结果；无效或未检测时为null |
| `target.center_x/center_y` | 平滑后的0–1图像坐标；左上角为原点 |
| `target.distance_m` | 当前躯干区域的光轴Z深度中值，单位米；无效时null |
| `target.depth` | 有效率、样本数、四分位距、无效原因和像素ROI |
| `attention` | 归一化水平/垂直偏差，右/上为正；仅供虚拟关注标记，没有舵机角度映射 |
| `frame.received_monotonic_s` | 主机接到这对图像的单调时钟时间；包含后续对齐/推理耗时 |
| `frame.color_timestamp_ms/depth_timestamp_ms` | 相机时间戳；各自带时钟域，不能直接与主机时间相减 |
| `camera_generation` / `run_id` | 相机重启次数对应的会话标识 / 进程运行标识；不跨会话复用目标 |

有效图像里没有检测到人，与相机失效是两件事。深度中值可能混入衣物、遮挡物或背景；这里不做人体分割，也不是全身最近点/到机器人外壳的距离。前向人体检测不能代替障碍物检测或导航。

后续消费者必须同时检查：`valid:true`、`simulated:false`、当前消息仍新鲜；需要距离时还要检查`target.depth.valid:true`及非null距离。对同一Jetson的进程，可使用当前`time.monotonic()`减去`frame.received_monotonic_s`核对帧龄。跨主机必须建立接收端超时机制，不能直接比较不同机器的monotonic时间。进程被强制终止时无法保证写出最后一条无效消息，因此不能永久信任磁盘上残留的`valid:true`。

## 7. 测试和实机验收

```bash
python -m unittest discover -s jetson/tests -p 'test_*.py' -v
```

若需要保存测试日志、环境版本及源码SHA-256，运行`python -m jetson.check`，结果写入`build/vision`。完整工程中可用`python scripts/package_vision.py`重新打包与测试记录匹配的源码；发布包不包含模型权重、运行录像或Python环境。

测试涵盖深度空洞/异常/混合区域、有效性与过期、跟踪与遮挡、重连时清空数据、SDK调用契约、帧数据拷贝、退出JSON和OpenCV检测器/渲染调用。SDK测试使用替身，不证明真实标定或USB稳定性；若未安装OpenCV，对应运行测试会明确跳过。

实机操作记录填入[commissioning.csv](commissioning.csv)。至少完成：

1. 相机模式连续运行30分钟，记录帧率、帧龄、异常与重连次数。
2. 拔出相机后确认`valid:false`且`target:null`；重新插入同一台相机，确认恢复、generation增加、旧目标不延续。
3. 在实测距离与不同站姿、光照下核对检测和距离；记录丢检/误检而不是只保存成功截图。
4. 遮挡深度/走出画面/两人交叉，检查未知距离、目标消失及ID切换是否符合预期。
5. 关闭/退出程序，确认相机可再次打开、`latest.json`的目标被清空。

## 8. 实现依据

- [RealSense Python RGB-D对齐示例](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py)：pipeline、实际深度比例和对齐接口。
- [RealSense API说明](https://github.com/realsenseai/librealsense/wiki/API-How-To)：时间戳、帧号、后台采集及帧缓冲生命周期。
- [OpenCV HOG接口](https://docs.opencv.org/4.x/d5/d33/structcv_1_1HOGDescriptor.html)：默认行人检测器和多尺度检测。
- [Ultralytics预测接口](https://docs.ultralytics.com/modes/predict/)：本地BGR图像输入、类别过滤和检测框输出。

这些是接口依据，不能替代本机性能、真实人体检出率或相机测距验收。

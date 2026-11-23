# v0.1 验证记录

日期：2026-10-23。验证主机为Windows AMD64，Python 3.11.4、NumPy 2.4.3、OpenCV headless 4.10.0.84。部署建议中的NumPy 1.x约束用于兼容Jetson系统OpenCV；该Jetson依赖组合尚待实测。

- `python -m jetson.check`：**24项通过，0失败，0错误，0跳过**。详细日志与源码哈希见tests.log（`../build/vision/tests.log`）及tests_report.json（`../build/vision/tests_report.json`）。
- 9秒合成RGB-D演示：处理72帧，发布45条有效状态，43条有关注目标，正常结束；合成数据包含短时遮挡和深度缺失。最终`latest.json`为`stopped`且目标清空。
- 实际OpenCV HOG后端在合成图像上运行3秒：处理24帧，发布15条有效状态，正常结束；绿色测试矩形没有被识别成人。这项只验证检测器接口和运行流程，不验证真实人体检出率。
- 预览PNG已实际生成并查看；中文目录下保存截图已通过回归测试。
- 本机缺少RealSense SDK时，真机模式明确报错、退出码2、输出无效状态，没有回退到合成数据。
- 相机SDK调用契约、数据拷贝、重复帧、时间域、序列号绑定使用替身测试；断连重试使用注入故障的假相机测试。
- 已核对原有6个Arduino固件源码SHA-256，与此前编译记录一致。本次没有修改Arduino源码或启用执行机构。

**未验证**：Jetson环境安装、真实D435if RGB-D取流/对齐/测距、拔插恢复、长期稳定性、实际场景人体检出率、多人ID连续性、YOLO/PyTorch/CUDA/TensorRT运行。`commissioning.csv`各项保持“未测”。

上述合成数据帧率和耗时仅来自本地测试，不能作为Jetson性能指标。

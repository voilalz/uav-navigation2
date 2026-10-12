# UAV Navigation 2

依据三份上传方案实现的固定翼无人机自主导航软件工程。核心为 **400 Hz 捷联惯导 + 100 Hz、20维 ESKF + 30 Hz 导航输出**，配套视觉/地形定位、MCU任务运行时、通信协议、飞控桥接、仿真与回放。

当前版本 **v0.1.0**。本次验收对象是宿主机可运行工程；STM32实机、Jetson相机/GPU、ArduPilot原生External AHRS后端、真实参考库和飞行精度尚需目标平台联调。原始方案中已有的仿真结果是设计材料中的历史描述，本工程实际结果见 [验证记录](docs/verification.md)。

## 快速运行

需要 CMake ≥3.20、支持C++17的编译器。Python3用于建库、通信桥和批量评估；默认工程没有Eigen、ROS、OpenCV、CUDA或模型下载依赖。

```sh
git clone https://github.com/voilalz/uav-navigation2.git
cd uav-navigation2
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 4
ctest --test-dir build -C Release --output-on-failure
```

Linux/macOS/MinGW单配置构建：

```sh
./build/nav/nav_sim 1200 1 out/sim combined
./build/nav/nav_replay out/sim/events.csv out/sim/replayed.csv
python tools/verify_replay.py out/sim/estimate.csv out/sim/replayed.csv
python tools/analyze_log.py out/sim/estimate.csv out/sim/truth.csv
./build/nav_pipeline_demo
./build/companion/nav_companion --demo
```

Visual Studio多配置构建的可执行文件分别在 `build/nav/Release/`、`build/companion/Release/`、`build/Release/`，文件名带`.exe`。VSCode已提供CMake构建、测试和GDB/MSVC调试配置。本次实际运行环境为Linux/GNU13；Windows/macOS已配置CI，尚未将远端CI或Win7运行写成已验证。

## 工程模块

| 路径 | 已实现内容 |
|---|---|
| `nav/include/nav`、`nav/src` | 定长矩阵/四元数、WGS-84、捷联、20维ESKF、时延历史、IMU阵列、SPSC队列、DEM/TERCOM、模式/健康管理 |
| `nav/sim`、`nav/apps` | 独立解析运动真值、MEMS误差、延迟/离群/丢失量测、三种仿真场景、事件日志回放、MCU输出链路示例 |
| `companion` | PGM读写、下视投影/平面正射、粗精NCC、平面VO、DSM网格配准、稀疏双帧三角化、瓦片LRU/SHA256、图像文件回放 |
| `firmware` | ADIS16470/RM3100/UBX解析与总线接口、时间扩展/PPS/曝光配对、四片IMU五点降采样、单写者任务运行时、FreeRTOS/BSP移植入口 |
| `integration` | 视觉JSONL→带CRC的POSITION_FIX/二维NED VO；导航帧接收；UART/UDP→MAVLink GPS_INPUT桥与超时/健康门限 |
| `tools` | DOM/DSM切片与完整性校验、演示影像生成、误差预算、100次Monte Carlo、独立真值评估与回放核对 |
| `deploy/jetson` | 静态网络配置、飞控桥systemd服务、离线图像回放服务 |
| `.github/workflows`、`.vscode` | 三平台构建/测试、地址与未定义行为检查、开发调试入口 |

## 图像级回放和实际量测链路

```sh
python tools/build_reference.py demo --output out/reference-demo
./build/companion/nav_companion --replay out/reference-demo/frames.manifest > out/reference-demo/measurements.jsonl
python -m integration.companion_sender --input out/reference-demo/measurements.jsonl \
  --origin-lat 30 --origin-lon 120 --output out/reference-demo/measurements.bin
```

上述过程实际运行图像、VO和地形算法。演示图像为明确标记的合成输入，不能用其匹配效果代替真实跨季节/跨光谱评估。`--output`只生成协议记录；发送到MCU需显式把它换成 `--udp <MCU地址>:<端口>`，同时保证图像曝光时间属于当前MCU时钟会话。实时相机采集与GPU零拷贝须由厂商SDK适配。

`nav_pipeline_demo`运行“实际图像NCC → POSITION_FIX序列化/CRC → MCU量测队列 → ESKF修正”，用于验证模块衔接。二维VO明确发送NED坐标系和二维协方差，禁止伪造垂直速度。

## 数字仿真

`nav_sim [duration_s] [seed] [output_dir|-] [combined|relative|inertial]`，时长1–7200秒；`-`关闭大体积原始日志。组合场景包含GNSS起始对准辅助、转弯/爬升、VO尺度偏差、30–400ms延迟、错误位置和120秒绝对修正中断。`relative`关闭绝对位置修正；`inertial`只保留惯导与气压高度阻尼。

```sh
python tools/monte_carlo.py build/nav/nav_sim --duration 1200 --runs 100 \
  --workers 4 --output out/monte-carlo.json
python tools/error_budget.py
```

100次、每次20分钟的组合导航数字仿真：平均水平RMS 10.25m，单次RMS范围7.25–13.97m，最坏瞬时水平误差57.08m；每次均为30Hz输出，168次注入的2km级错误位置全部被拒绝。完整种子记录保存在 `docs/validation/`。该结果只适用于给定误差、初值和量测模型。

## 阅读顺序与移植

1. [导航核心：坐标、状态、时延和故障语义](docs/nav-core.md)
2. [视觉算法、参考库和回放格式](docs/companion.md)
3. [完整二进制协议](docs/protocol.md)
4. [STM32硬件/FreeRTOS移植](docs/hardware-port.md)
5. [飞控桥接及高度基准](docs/flight-controller.md)
6. [验证与交付范围](docs/verification.md)

原始方案完整保留在 `docs/source/`。学习型SuperPoint/LightGlue/TensorRT和稠密SfM/BA保留替换接口，未接入时显式返回不可用；当前CPU基线提供可运行算法，不声称完成这些供应商/模型相关后端。

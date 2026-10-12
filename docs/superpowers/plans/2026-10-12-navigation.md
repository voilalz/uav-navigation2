# UAV Navigation 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans for the core and superpowers:dispatching-parallel-agents for independent peripheral modules. Steps use checkbox syntax for tracking.

**Goal:** 按三份上传方案提交完整、可运行的导航软件工程。

**Architecture:** 固定内存导航核心与独立视觉量测服务；显式二进制协议连接 MCU、Jetson 与飞控。硬件适配不改变算法核心。

**Tech Stack:** C++17、C99、CMake、CTest、Python3 标准库；OpenCV 为可选增强。

**Spec:** `docs/implementation-design.md`、`docs/source/software-design.md`、`docs/source/navigation-design.md`、`docs/source/hardware-design.md`。

## Global Constraints

- 导航核心固定内存、无异常、无 RTTI；NED/FRD、SI、微秒时间戳。
- 20 维 ESKF，400/100/30 Hz；所有供应商相关接口保持独立。
- 对输入时间、量测协方差、地图边界和链路健康做显式校验。
- 不把未运行的目标板/飞控/学习模型验证写成通过。

## Review Focus

- 异步量测：未来、过旧、乱序时间不污染状态。
- 地图/图像：低纹理、多峰、出界和非有限高程不产生定位。
- 串口：截断、超长、CRC 错和伪同步字可恢复。
- GNSS 恢复：不通过一致性门限时不得切入 M0。
- 数值异常：非正定 R、奇异姿态、NaN 和过长 IMU 间隔不得进入滤波。

### Task 1: 导航核心

**Files:** `nav/include/nav/{math,earth,types,strapdown,eskf,state_history,nav_core,imu_array,spsc_queue,terrain}.hpp`、对应 `nav/src/`、`nav/tests/test_core.cpp`。

**Interfaces:** `Vec3` 为 `Vec<3>`；`NominalState` 与 `Measurement` 见 `docs/contracts.md`；`NavCore::initialize/onImu/onMeasurement/output`；不依赖协议和图像。

- [x] 写静止惯导、位置门限、航向/VO 雅可比、延迟与模式、IMU 故障和队列测试。
- [x] 编译未实现入口，确认失败来自功能缺失。
- [x] 实现数学、机械编排、ESKF 和核心调度，执行 `test_core`。
- [x] 用解析/数值差分独立核对模型；提交导航核心。

### Task 2: C99 链路协议（可独立并行）

**Files:** `nav/include/nav/protocol.h`、`nav/src/protocol.c`、`nav/tests/test_protocol.c`、`docs/protocol.md`。

**Interfaces:** A5 5A | id | len | seq_le16 | payload | CRC_le16；7 个消息 ID。长度上限 255。显式字段读写，状态化 parser，CRC 覆盖 id 至 payload。

- [x] 写 CRC 已知向量、字节往返、噪声/截断恢复测试并先运行失败。
- [x] 实现编解码和消息 codec；运行 C99 测试；提交模块。

### Task 3: 视觉/地形/参考库（可独立并行）

**Files:** `companion/include/companion/`、`companion/src/`、`companion/tests/`、`tools/build_reference.py`。

**Interfaces:** 只读取图像、地图、导航先验；输出位置/速度质量和协方差，由主工程桥接为 Measurement。

- [x] 写合成纹理平移、相机正射、VO 尺度、DSM 平移、多峰拒绝、瓦片哈希测试并先失败。
- [x] 实现无需下载模型的 CPU 基线、可替换 matcher、参考库工具；运行模块测试；提交模块。

### Task 4: 固件/飞控移植（可独立并行）

**Files:** `firmware/`、`integration/`、`deploy/`、设备/时间/链接测试。

**Interfaces:** MCU 队列使用 `ImuSample`、`Measurement`；独立设备驱动、统一时钟；协议 NAV_OUTPUT codec 接飞控桥。

- [x] 写芯片解析、时钟溢出、串口超时和飞控转换测试并先失败。
- [x] 实现驱动、FreeRTOS/BSP 接口、UDP/串口服务与 systemd；文档明确板级约束；提交模块。

### Task 5: 仿真、回放、构建与交付

**Files:** `nav/sim/`、`nav/apps/`、`tools/`、根/子目录 CMake、CI、README、`docs/verification.md`。

**Interfaces:** `nav_sim duration seed output scenario` 生成独立真值/估计 CSV 与 JSON 指标；`nav_replay` 回放 IMU/量测 CSV；CTest 调用各模块与回归。

- [x] 建立能拒绝无效 CLI 的冒烟测试。
- [x] 实现仿真/回放，运行完整 CTest、长仿真和有限种子批次，记录实际指标。
- [x] 独立审查全工程，修正影响运行的缺陷，保留实机未验证清单。
- [x] 通过 GitHub 连接器提交用户指定仓库，读取远端提交验证；给出链接和运行命令。

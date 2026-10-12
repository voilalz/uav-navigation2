# 导航线协议 v1

实现文件为 `nav/include/nav/protocol.h` 和 `nav/src/protocol.c`，纯 C99、固定缓存、无需堆分配。结构体仅表示主机数据，**不得直接发送结构体内存或依赖 packed/alignment 属性**。所有整数、浮点按下表逐字段显式小端序列化。接口可由 C++ 的 `extern "C"` 使用。

## 帧与 CRC

| 字节偏移 | 长度 | 字段 |
|---|---:|---|
| 0 | 1 | 固定 `0xA5` |
| 1 | 1 | 固定 `0x5A` |
| 2 | 1 | `msg_id` |
| 3 | 1 | `payload_len`，0–255 |
| 4 | 2 | `seq`，uint16，小端，按链路模 65536 递增 |
| 6 | L | `payload` |
| 6+L | 2 | CRC，uint16，小端 |

总长度 `L+8`，最大 263 字节。CRC 覆盖 `msg_id、payload_len、seq、payload`，**不包含**同步头和 CRC 字段。CRC-16/CCITT-FALSE 参数：poly `0x1021`、init `0xFFFF`、refin/refout=false、xorout=0；字符串 `123456789` 的校验值为 `0x29B1`。

`u8/u32/u64` 分别为无符号 8/32/64 位整数；`f32/f64` 为 IEEE-754 binary32/binary64。编译时检查字节和浮点格式，不允许 fast-math，以免编译器省略 NaN/Inf 检查。支持通常整数与浮点同字节序的 IEEE-754 平台，不支持浮点混合字节序平台。

时间基准为 MCU 启动以来微秒时钟。相机量测的 `t_us` 是 MCU 时间域中的**曝光中点**，不是 Jetson 处理完成时间。导航坐标 NED，机体坐标 FRD；位置使用 deg/deg/椭球高 m，内部导航的 rad 必须在边界转换。

| ID | 消息 | C 数据类型 | 载荷/整帧字节 | 方向/建议频率 |
|---|---|---|---:|---|
| 0x01 | NAV_OUTPUT | `nav_wire_output_t` | 85 / 93 | MCU → 飞控、Jetson / 30 Hz |
| 0x02 | TIME_SYNC | `nav_wire_time_sync_t` | 13 / 21 | MCU → Jetson / 1 Hz |
| 0x03 | CAM_TRIGGER | `nav_wire_cam_trigger_t` | 17 / 25 | MCU → Jetson / 每帧 |
| 0x04 | NAV_PRIOR | `nav_wire_prior_t` | 69 / 77 | MCU → Jetson / 10 Hz |
| 0x10 | VO_VELOCITY | `nav_wire_vo_velocity_t` | 107 / 115 | Jetson → MCU / 30 Hz |
| 0x11 | POSITION_FIX | `nav_wire_position_fix_t` | 106 / 114 | Jetson → MCU / 0.5–2 Hz |
| 0x20 | HEALTH | `nav_wire_health_t` | 29 / 37 | 双向 / 1 Hz |

附件仅确定消息 ID 和帧外壳，并未逐字段给出载荷。以下定义是本仓库实施后的精确互操作约定。NAV_OUTPUT 在此协议中是 **93 字节整帧**；附件中 82 字节的带宽估算不适用于该布局。30 Hz 约为 2.79 kB/s，未含运输层开销。

## NAV_OUTPUT，载荷 85 字节

| 偏移 | 类型 | 字段 | 单位/约定 |
|---:|---|---|---|
| 0 | u64 | `t_us` | MCU 时间，µs |
| 8 | f64 | `lat_deg` | 纬度，deg |
| 16 | f64 | `lon_deg` | 经度，deg |
| 24 | f64 | `height_m` | 椭球高，m |
| 32 | f32[3] | `vel_ned` | N/E/D，m/s |
| 44 | f32[4] | `quaternion` | q_nb，w/x/y/z，把 FRD 转到 NED |
| 60 | f32[3] | `angular_rate` | FRD x/y/z，rad/s |
| 72 | f32 | `h_sigma` | 水平位置标准差，m |
| 76 | f32 | `v_sigma` | 垂直位置标准差，m |
| 80 | u8 | `mode` | 见模式表 |
| 81 | u32 | `health` | 应用层健康位掩码，协议透明传递 |

Python `struct` 格式：`<Qddd3f4f3f2fBI`。四元数要求有限且非零；编码器不会替发送方归一化，应由姿态生产/消费模块维持单位四元数。

| mode | 含义 |
|---:|---|
| 0 | GNSS |
| 1 | 视觉 |
| 2 | 地形 |
| 3 | 推算 |
| 4 | 重捕获 |

## TIME_SYNC，载荷 13 字节

| 偏移 | 类型 | 字段 | 含义 |
|---:|---|---|---|
| 0 | u64 | `t_us` | MCU 捕获的 PPS 边沿时间；无有效 PPS 时为本次发送时刻 |
| 8 | u32 | `pulse_count` | 启动后 PPS 计数，模 2³² |
| 12 | u8 | `pps_valid` | 只能为 0 或 1 |

Python 格式 `<QIB`。该消息承载时钟锚点，时钟偏差/漂移估计、PPS 驱动和远端时钟转换由固件与相机适配层实现。

## CAM_TRIGGER，载荷 17 字节

| 偏移 | 类型 | 字段 | 含义 |
|---:|---|---|---|
| 0 | u32 | `frame_id` | 同一相机流内的帧号，模 2³² |
| 4 | u64 | `t_mid_exposure_us` | MCU 时间域曝光中点，µs |
| 12 | u32 | `exposure_us` | 曝光时长，µs；0 表示未知 |
| 16 | u8 | `camera_id` | 应用分配的相机流 ID |

Python 格式 `<IQIB`。以 `(camera_id, frame_id)` 配对相机图像，视觉回传量测的 `t_us` 使用此曝光中点。

## NAV_PRIOR，载荷 69 字节

| 偏移 | 类型 | 字段 | 含义 |
|---:|---|---|---|
| 0 | u64 | `t_us` | 先验时刻，µs |
| 8 | f64 | `lat_deg` | 纬度，deg |
| 16 | f64 | `lon_deg` | 经度，deg |
| 24 | f64 | `height_m` | 椭球高，m |
| 32 | f32[3] | `vel_ned` | m/s |
| 44 | f32[4] | `quaternion` | q_nb，w/x/y/z |
| 60 | f32 | `h_sigma` | m |
| 64 | f32 | `v_sigma` | m |
| 68 | u8 | `mode` | 与 NAV_OUTPUT 相同 |

Python 格式 `<Qddd3f4f2fB`。先验用于图像检索区域和姿态约束，其有效性与完整精度由导航模块负责。

## VO_VELOCITY 107 字节，POSITION_FIX 106 字节

| 偏移 | 类型 | VO_VELOCITY 字段 | POSITION_FIX 字段 |
|---:|---|---|---|
| 0 | u64 | `t_us`，曝光中点 | `t_us`，曝光中点 |
| 8 | f64[3] | `velocity_body`，由 frame 指定坐标系，m/s | `lat_deg,lon_deg,height_m`，deg/deg/m |
| 32 | f64[9] | `covariance`，与速度相同坐标系，(m/s)² | `covariance`，当地 NED，m² |
| 104 | u8 | `source` | `source` |
| 105 | u8 | `dimensions`，1–3 | `dimensions`，1–3 |
| 106 | u8 | `frame`，BODY=0、NED=1 | 无此字段 |

Python 格式：VO 为 `<Q12dBBB`，位置为 `<Q12dBB`。九个协方差元素按行主序 `R00,R01,R02,R10,R11,R12,R20,R21,R22` 传递，保留所有非对角元。位置 covariance 是当地 NED 米制协方差，不是经纬度平方；量测端需要先统一高度基准。

VO 的 `frame` 必须显式指定：`NAV_WIRE_VELOCITY_BODY=0` 表示 FRD 机体系，`NAV_WIRE_VELOCITY_NED=1` 表示 NED 导航系；其他值拒绝。结构成员名 `velocity_body` 为现有 C 调用代码保留，其实际三个速度分量和 covariance 均由 frame 定义。二维平面视觉 VO 应发送 frame=NED、dimensions=2、source=VISION，仅观测 N/E；接收方映射到 NED 速度量测，不能将其当成机体速度或额外约束 D。三维机体 VO 使用 frame=BODY、dimensions=3，并由接收方应用姿态观测模型。

`dimensions` 指定左上 1×1、2×2 或 3×3 活跃子矩阵，按对应坐标轴前缀使用。固定三个值和九个协方差元素始终全部传输；未活跃字段仍必须有限，可用 0 填充。典型 VO 与完整位置修正使用 dimensions=3，二维位置为 N/E。

| source | 含义 |
|---:|---|
| 0 | 未指定 |
| 1 | GNSS |
| 2 | 视觉 |
| 3 | 三维地形 |
| 4 | TERCOM |

活跃 covariance 必须对称正定。对称容差为 `1e-9 * max(1,abs(Rij),abs(Rji))`，以固定大小 Cholesky 检查对称平均 `(R+Rᵀ)/2` 的严格正枢轴，检查中间运算是否有限。检查不会改写传输矩阵或丢弃非对角元。

## HEALTH，载荷 29 字节

| 偏移 | 类型 | 字段 | 含义 |
|---:|---|---|---|
| 0 | u64 | `t_us` | MCU 时间域，µs |
| 8 | u32 | `uptime_ms` | 本组件运行时间，模 2³² |
| 12 | u32 | `health` | 应用层健康位掩码 |
| 16 | u32 | `rx_crc_errors` | 累积 CRC 错误 |
| 20 | u32 | `rx_length_errors` | 已知消息长度错误 |
| 24 | u32 | `queue_overflows` | 应用层队列溢出 |
| 28 | u8 | `component_id` | 应用分配的组件 ID |

Python 格式 `<Q5IB`。协议解析计数采用 uint32 饱和累加，达到最大值不会回绕；队列计数由应用层提供。

## C API 和流式解析

发送：`nav_encode_*(&message, payload, capacity, &payload_len)` 生成载荷，然后 `nav_frame_encode(msg_id, seq, payload, payload_len, wire, capacity, &wire_len)` 包帧。包帧支持 payload 和输出缓冲重叠/原地包帧。消息编码器与原始结构体的缓冲不应重叠。

接收初始化 `nav_parser_init(&parser)`；逐字节输入 `nav_parser_feed(&parser, byte, &frame)`。只有 `NAV_PARSE_FRAME` 才能消费 frame 内容，再按 ID 调用对应 `nav_decode_*`。每次产帧后用 `nav_parser_next` 取尽可能积存的帧：

```c
nav_parse_result_t result = nav_parser_feed(&parser, next_byte, &frame);
while (result == NAV_PARSE_FRAME) {
    /* 按 frame.msg_id 分派，并检查 nav_decode_* 的返回值。 */
    consume_frame(&frame);
    result = nav_parser_next(&parser, &frame);
}
```

`consume_frame` 是调用方自己的分派函数。每条链路独立 parser 实例；同一实例由调用方保证串行使用。codec 返回 `NAV_PROTOCOL_OK`、`BAD_ARGUMENT`、`BAD_LENGTH` 或 `BAD_VALUE`；失败不修改目标消息、输出缓存或输出长度。解析器 INCOMPLETE/BAD_ARGUMENT 不修改 frame；有效 frame 的 payload 仅前 payload_len 字节有意义。

解析规则：

1. 丢弃前导噪声，保留可能跨输入边界的 `A5 5A`；支持重叠同步头。
2. 七种已知 ID 要求载荷长度精确匹配，长度错误立即拒绝并继续扫描。未知 ID 可承载 0–255 字节有效 CRC 数据供未来扩展，应用自行选择是否消费。
3. 在候选帧未收全前等待，不扫描内部同步字。因此合法任意载荷内嵌另一完整有效帧也不会截断外帧。
4. 候选收全后 CRC 错误则丢弃一个字节并重新扫描剩余数据。缓存固定 263 字节，噪声、CRC 错误和字节错位都不会导致堆分配或越界。
5. 如果传输停止在截断/伪长候选中，协议缺少转义与边界，无法同时保证任意载荷可逆和立即识别内部后续帧。运输层确认足够的帧间超时后调用 `nav_parser_reset_stream` 丢弃当前未完成数据；该操作保留统计并累计 discarded_bytes。先用 `nav_parser_next` 取尽完整已缓存帧，再决定超时清空。`nav_parser_init` 则重置全部统计。

协议层拒绝 NaN/Inf、经纬度越界、负 sigma、零四元数、未知 mode/source/速度 frame、非法 dimensions、活跃 covariance 非正定或不对称。协议层不判断时间是否未来/过旧、序号重复/丢包或量测是否物理可信，这些由导航门限、时延历史与链路管理判断。CRC 只检验意外传输损坏，不提供身份认证。

## 互操作样例和验证

NAV_OUTPUT：t_us=`0x0102030405060708`、纬经高 `(45,-90,100)`、速度 `(1,-2,0.5)`、四元数 `(1,0,0,0)`、角速度 `(0.25,-0.5,2)`、sigma `(3,4)`、mode=2、health=`0x78563412`、seq=65535。下列 93 字节由 Python `struct.pack` 和 `binascii.crc_hqx(init=65535)` 独立计算；CRC=`0x7401`，线上尾字节 `01 74`：

```text
a55a0155ffff0807060504030201000000000080464000000000008056c000000000000059400000803f000000c00000003f0000803f0000000000000000000000000000803e000000bf00000040000040400000804002123456780174
```

测试 `nav/tests/test_protocol.c` 使用独立黄金字节、七种消息读写、255 字节载荷、原地包帧、奇地址解码、嵌套有效帧、坏帧后的多个缓存帧、全部 93×8 个单比特破坏、全部 93 个截断位置、数值与协方差拒绝、饱和计数。开发先用可编译桩观察缺失功能测试失败，再替换为真实实现。

```sh
gcc -std=c99 -Wall -Wextra -Werror -pedantic -Inav/include \
    nav/src/protocol.c nav/tests/test_protocol.c -lm -o /tmp/uav2_test_protocol
/tmp/uav2_test_protocol
```

CMake target 为 `navprotocol`，单元测试为 `test_protocol`。宿主机可另外以 AddressSanitizer/UndefinedBehaviorSanitizer 运行同一测试。IEEE 大端目标和目标板串口/UDP时延仍需要对应平台验证。

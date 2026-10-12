# 飞控链路和试验桥

`integration/nav_receiver.py`是标准库导航接收模块，解析93字节NAV_OUTPUT，校验CRC-16/CCITT-FALSE、85字节字段长度、有限数值、单位四元数、正的标准差、health、序列号回绕与严格递增MCU时间。每条有效输出以**本地单调接收时间**计500ms新鲜度。帧间超过500ms清解析缓存并保留统计，恢复截断的未知ID外帧；未收全合法外帧时不扫描其内部同步字。

`integration/mavlink_bridge.py`是可运行的UDP/串口MAVLink2 GPS_INPUT（ID232、CRC extra151）桥；UDP只使用Python标准库，串口另需pyserial。它保留串口批量包里的每一条有效导航帧，不重复广播最后一条。重复/倒退/不健康量测立即清当前可用值，链路到期停止注入。

```sh
# 本机SITL，MCU→Jetson的NAV_OUTPUT在UDP14560接收
python3 -m integration.mavlink_bridge \
  --listen 192.168.42.2:14560 --output-udp 127.0.0.1:14550

# 两条真实串口；须检查设备权限与飞控配置
python3 -m integration.mavlink_bridge \
  --input-serial /dev/ttyUSB0 --output-serial /dev/ttyTHS1 \
  --input-baud 921600 --output-baud 921600
```

## 字段映射

| NAV_OUTPUT | GPS_INPUT | 语义 |
|---|---|---|
| t_us | time_usec | MCU启动后微秒时间；无UTC转换 |
| lat_deg/lon_deg | lat/lon | WGS-84 ×10⁷、显式int32 |
| height_m | alt | **椭球高不能直接当MSL**；默认ignore ALT，显式`--geoid-separation N`后H=h−N |
| vel_ned | vn/ve/vd | 米/秒，地向下为正 |
| h_sigma/v_sigma | horiz_accuracy/vert_accuracy | 导航位置标准差，米；传递输出估计精度而非卫星DOP |
| 缺少DOP/速度协方差 | ignore HDOP/VDOP/SPEED_ACCURACY | 填0并设置忽略位，不虚构精度 |
| 缺少GNSS绝对时/卫星计数 | time_week=0、time_week_ms=0、satellites_visible=0 | 本桥未提供这些值，不从启动时钟或当前系统时间发明GPS周 |
| mode/health | 本地有效性门控 | 默认拒绝mask=0xa5（IMU、过大协方差、时间、滤波异常），h_sigma≤100m |

GPS_INPUT没有GPS周的ignore位；零周字段只是本桥缺少绝对GNSS时信息。必须在目标飞控版本核验其对该组合的处理。fix_type=3表示本桥提供完整三维导航位置的试验映射，**不表示真实GNSS或RTK锁定**。GPS_INPUT只提供位置/速度，无法把外部姿态、角速度和全协方差交给飛控；飞控仍运行自身EKF。常数geoid只适于已核定局部区域，长航线应换成随位置变化的可信模型。

序列号和MCU时间倒退被拒绝，MCU重启后需要明确重启接收器，避免旧会话混入新会话。接收新鲜度不测量MCU到Jetson的实际传输延迟；应在BSP发送侧丢弃过时发送队列，并通过PPS/实机测试核对链路时延。

## ArduPilot/PX4验证界限

本仓库没有把一个未经构建的AP_ExternalAHRS后端声称为可用。原生外部AHRS需在**明确固定版本**的ArduPilot源码内实现backend、参数类型、位置/速度/姿态和失效状态，并在SITL/HIL中验证，再使用実机。桥接测试只证明字节协议和失效门控，不能证明目标飞控接受这些量测。

飞控参数/GPS类型/EKF来源取决于版本，操作者应照实际版本官方文档配置。500ms停止注入不是自动飞控切换；必须检查source switching、EKF failsafe、无GNSS返航条件和自身传感器备份。试验日志须包含NAV_OUTPUT、GPS_INPUT、飞控EKF创新/健康和独立真值，核对频率、坐标、高度基准、更新时间、异常与恢复。

依据：[MAVLink GPS_INPUT/ignore flags](https://mavlink.io/en/messages/common.html#GPS_INPUT)、[MAVLink官方生成C字段布局](https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_gps_input.h)、[ArduPilot GPS_INPUT指导](https://ardupilot.org/dev/docs/mavlink-gps-input.html)。尚未运行目标ArduPilot/PX4源码或硬件在环。

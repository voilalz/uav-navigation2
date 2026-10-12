# 模块接口约定

仓库根：`/workspace/scratch/8d7fc3e7ed97/uav-navigation2`。各模块独立维护自己的文件，不修改根 CMake。

## 导航（root 实现）

`nav/include/nav/types.hpp`:

```cpp
namespace nav {
using Timestamp = uint64_t;
using Vec3 = Vec<3>; // v[0..2], Vec3{{x,y,z}}
struct ImuSample { Timestamp t_us{}; Vec3 gyro_rad_s{}, accel_m_s2{}; double temperature_c{25}; bool valid{true}; };
struct NominalState { Timestamp t_us{}; double lat_rad{},lon_rad{},height_m{}; Vec3 velocity_ned{}; Quaternion q_nb{}; Vec3 gyro_bias{},accel_bias{}; Vec<2> wind_ne{}; double baro_bias{},vo_scale{},mag_bias{}; };
enum class MeasurementKind : uint8_t {Position,VelocityNed,VelocityBody,Airspeed,BaroHeight,MagHeading,RadarHeight};
enum class FixSource : uint8_t {None,Gnss,Vision,Terrain3D,Tercom};
struct Measurement { Timestamp t_us{}; MeasurementKind kind{MeasurementKind::Position}; FixSource source{FixSource::None}; Vec3 value{}; Mat<3,3> covariance{}; uint8_t dimensions{3}; double terrain_height_m{}; Vec<2> terrain_gradient{}; };
enum class Mode : uint8_t {Gnss=0,Vision=1,Terrain=2,DeadReckoning=3,Reacquire=4};
struct NavOutput { NominalState state{}; Vec3 angular_rate_body{}; double horizontal_sigma_m{},vertical_sigma_m{}; Mode mode{Mode::DeadReckoning}; uint32_t health{}; };
class NavCore { public: void initialize(const NominalState&); bool onImu(const ImuSample&); bool onMeasurement(const Measurement&); NavOutput output() const; bool takeOutput(NavOutput&); const NominalState& state() const; };
}
```

`nav/math.hpp`: `Vec<N>`/`Mat<R,C>` 固定 `std::array<double,...>`，`operator[]` / `(r,c)`；`Quaternion{w,x,y,z}`，`fromEuler(roll,pitch,yaw)`、`matrix()`、`toEuler()`、`exp(Vec3)`、`normalized()`；`norm`、`cross`、`transpose`、`identity<N>()`。

`nav/earth.hpp`: `earth::gravity(lat,h)`、`earth::radii(lat)`（Vec<2> meridian, prime vertical）、`earth::rotation(lat)`、`earth::transport(lat,h,v)`；`displacementNed(from,to)`、`offsetPosition(state,Vec3)`。

## 协议（protocol agent 实现）

协议模块仅 C99，无需依赖 C++ types。提供 `nav_frame_encode`、`nav_parser_init/nav_parser_feed`、`nav_crc16` 及七种消息的 encode/decode 函数。`NAV_OUTPUT` 必含 t_us、lat_deg/lon_deg/height_m、vel_ned[3]、quaternion[4]、angular_rate[3]、h_sigma/v_sigma、mode/health。`VO` 与 `POSITION_FIX` 包含曝光 t_us、值、协方差（三维全矩阵）、source/dimensions。float/double 均显式序列化。

## 视觉（companion agent 实现）

使用 companion 自有轻量类型（与 nav 无依赖）；返回 `MatchResult{valid,north_m,east_m,height_m,sigma_m,...}` 与 `VelocityResult`，root 负责 NavCore 桥接。模块 CMake 在 `companion/CMakeLists.txt` 内定义 `companion_lib`、`nav_companion`、`test_companion`。使用标准 C++17 CPU 基线，可选 OpenCV；不以桩代替定位算法。

## 设备（firmware agent 实现）

只依赖上述 nav types，协议 agent 完成后再接入；先可独立测试解码与时间。自己的 `firmware/CMakeLists.txt`/`integration/CMakeLists.txt`，root 添加子目录。每个测试用不同文件/目标。

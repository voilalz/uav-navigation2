#pragma once
#include "nav/math.hpp"
#include <cstdint>
namespace nav {
using Timestamp = std::uint64_t;
struct ImuSample {
  Timestamp t_us{};
  Vec3 gyro_rad_s{}, accel_m_s2{};
  double temperature_c{25};
  bool valid{true};
};
struct NominalState {
  Timestamp t_us{};
  double lat_rad{}, lon_rad{}, height_m{};
  Vec3 velocity_ned{};
  Quaternion q_nb{};
  Vec3 gyro_bias{}, accel_bias{};
  Vec<2> wind_ne{};
  double baro_bias{}, vo_scale{}, mag_bias{};
};
enum class MeasurementKind : std::uint8_t {
  Position,
  VelocityNed,
  VelocityBody,
  Airspeed,
  BaroHeight,
  MagHeading,
  RadarHeight
};
enum class FixSource : std::uint8_t { None, Gnss, Vision, Terrain3D, Tercom };
struct Measurement {
  Timestamp t_us{};
  MeasurementKind kind{MeasurementKind::Position};
  FixSource source{FixSource::None};
  Vec3 value{};
  Mat<3, 3> covariance{};
  std::uint8_t dimensions{3};
  double terrain_height_m{};
  Vec<2> terrain_gradient{};
};
enum class Mode : std::uint8_t {
  Gnss = 0,
  Vision = 1,
  Terrain = 2,
  DeadReckoning = 3,
  Reacquire = 4
};
enum Health : std::uint32_t {
  ImuInvalid = 1u,
  VoLost = 2u,
  CovLarge = 4u,
  AbsoluteLost = 8u,
  MeasurementRejected = 16u,
  TimeInvalid = 32u,
  QueueOverflow = 64u,
  FilterInvalid = 128u
};
struct NavOutput {
  NominalState state{};
  Vec3 angular_rate_body{};
  double horizontal_sigma_m{}, vertical_sigma_m{};
  Mode mode{Mode::DeadReckoning};
  std::uint32_t health{};
};
struct Statistics {
  std::uint64_t imu_accepted{}, imu_rejected{}, measurements_accepted{},
      measurements_rejected{}, time_rejected{}, outputs{}, outliers{};
};
inline bool validState(const NominalState &s) {
  return std::isfinite(s.lat_rad) && std::abs(s.lat_rad) < 89.5 * deg &&
         std::isfinite(s.lon_rad) && std::abs(s.lon_rad) <= pi &&
         std::isfinite(s.height_m) && s.height_m > -1000 &&
         s.height_m < 100000 && finite(s.q_nb) && finite(s.velocity_ned) &&
         finite(s.gyro_bias) && finite(s.accel_bias) && finite(s.wind_ne) &&
         std::isfinite(s.baro_bias) && std::isfinite(s.vo_scale) &&
         std::abs(s.vo_scale) < 0.5 && std::isfinite(s.mag_bias);
}
} // namespace nav

#include "nav/strapdown.hpp"
namespace nav {
// 固定 dt 输入；状态仅在完整步成功后更新。
bool propagateStrapdown(NominalState &s, const ImuSample &imu, double dt) {
  if (!validState(s) || !imu.valid || !finite(imu.gyro_rad_s) ||
      !finite(imu.accel_m_s2) || !(dt > 0 && dt <= 0.1))
    return false;
  const auto old = s;
  const auto wie = earth::rotation(s.lat_rad),
             wen = earth::transport(s.lat_rad, s.height_m, s.velocity_ned);
  const auto omega_n = wie + wen;
  const auto wb = imu.gyro_rad_s - s.gyro_bias;
  const auto fb = imu.accel_m_s2 - s.accel_bias;
  const auto mid = (Quaternion::exp(omega_n * (-dt / 2)) * s.q_nb *
                    Quaternion::exp(wb * (dt / 2)))
                       .normalized();
  const Vec3 accel = mid.matrix() * fb +
                     Vec3{{0, 0, earth::gravity(s.lat_rad, s.height_m)}} -
                     cross(wie * 2 + wen, s.velocity_ned);
  s.velocity_ned = s.velocity_ned + accel * dt;
  s.q_nb =
      (Quaternion::exp(omega_n * (-dt)) * s.q_nb * Quaternion::exp(wb * dt))
          .normalized();
  offsetPosition(s, (old.velocity_ned + s.velocity_ned) * (dt / 2));
  s.t_us = imu.t_us;
  if (!validState(s)) {
    s = old;
    return false;
  }
  return true;
}
} // namespace nav

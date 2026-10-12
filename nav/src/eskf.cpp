#include "nav/eskf.hpp"
namespace nav {
void injectError(NominalState &s, const ErrorVector &d) {
  offsetPosition(s, {{d[0], d[1], d[2]}});
  s.velocity_ned = s.velocity_ned + Vec3{{d[3], d[4], d[5]}};
  s.q_nb = (Quaternion::exp({{d[6], d[7], d[8]}}) * s.q_nb).normalized();
  for (std::size_t i = 0; i < 3; ++i) {
    s.gyro_bias[i] += d[9 + i];
    s.accel_bias[i] += d[12 + i];
  }
  s.wind_ne[0] += d[15];
  s.wind_ne[1] += d[16];
  s.baro_bias += d[17];
  s.vo_scale += d[18];
  s.mag_bias = wrapAngle(s.mag_bias + d[19]);
}
Eskf::Eskf() { reset(); }
void Eskf::reset() {
  p_ = {};
  const double sigma[20] = {40,
                            40,
                            15,
                            2,
                            2,
                            1,
                            1 * deg,
                            1 * deg,
                            3 * deg,
                            20 * deg / 3600,
                            20 * deg / 3600,
                            20 * deg / 3600,
                            0.04,
                            0.04,
                            0.04,
                            8,
                            8,
                            30,
                            0.02,
                            2 * deg};
  for (std::size_t i = 0; i < 20; ++i)
    p_(i, i) = sigma[i] * sigma[i];
  last_nis_ = 0;
}
bool Eskf::predict(const NominalState &s, const Vec3 &fb, double dt) {
  if (!(dt > 0 && dt <= 0.1) || !finite(fb) || !validState(s))
    return false;
  Covariance f;
  for (std::size_t i = 0; i < 3; ++i)
    f(i, 3 + i) = 1;
  const auto c = s.q_nb.matrix();
  const auto wie = earth::rotation(s.lat_rad),
             wen = earth::transport(s.lat_rad, s.height_m, s.velocity_ned);
  const auto vv = skew(wie * 2 + wen) * (-1), vt = skew(c * fb) * (-1),
             tt = skew(wie + wen) * (-1);
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) {
      f(3 + i, 3 + j) = vv(i, j);
      f(3 + i, 6 + j) = vt(i, j);
      f(3 + i, 12 + j) = -c(i, j);
      f(6 + i, 6 + j) = tt(i, j);
      f(6 + i, 9 + j) = -c(i, j);
    }
  f(5, 2) = 2 * earth::gravity(s.lat_rad, s.height_m) / earth::a;
  f(15, 15) = f(16, 16) = -1.0 / 600;
  f(17, 17) = -1.0 / 1800;
  f(19, 19) = -1.0 / 3600;
  const auto phi = identity<20>() + f * dt;
  Covariance next = phi * p_ * transpose(phi);
  double q[20]{};
  for (std::size_t i = 0; i < 3; ++i) {
    q[3 + i] = noise.accel_noise * noise.accel_noise;
    q[6 + i] = noise.gyro_noise * noise.gyro_noise;
    q[9 + i] = noise.gyro_bias_walk * noise.gyro_bias_walk;
    q[12 + i] = noise.accel_bias_walk * noise.accel_bias_walk;
  }
  q[15] = q[16] = noise.wind_walk * noise.wind_walk;
  q[17] = noise.baro_walk * noise.baro_walk;
  q[18] = noise.scale_walk * noise.scale_walk;
  q[19] = noise.mag_walk * noise.mag_walk;
  for (std::size_t i = 0; i < 20; ++i)
    next(i, i) += q[i] * dt;
  // 加计白噪声到位置的离散积分项。
  for (std::size_t i = 0; i < 3; ++i) {
    next(i, i) += q[3 + i] * dt * dt * dt / 3;
    next(i, 3 + i) += q[3 + i] * dt * dt / 2;
    next(3 + i, i) += q[3 + i] * dt * dt / 2;
  }
  if (!finite(next))
    return false;
  p_ = next;
  return true;
}
UpdateResult Eskf::update(NominalState &s, const Vec3 &res,
                          const ObservationMatrix &h, const Mat<3, 3> &rin,
                          std::size_t dim, ErrorVector *correction) {
  if (dim < 1 || dim > 3 || !validState(s) || !finite(res) || !finite(h) ||
      !finite(rin))
    return UpdateResult::Invalid;
  Mat<3, 3> r = rin;
  for (std::size_t i = dim; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j)
      r(i, j) = r(j, i) = 0;
    r(i, i) = 1;
  }
  Mat<3, 3> l;
  if (!cholesky(r, l))
    return UpdateResult::Invalid;
  ObservationMatrix hm = h;
  Vec3 residual = res;
  for (std::size_t i = dim; i < 3; ++i) {
    residual[i] = 0;
    for (std::size_t j = 0; j < 20; ++j)
      hm(i, j) = 0;
  }
  const auto pht = p_ * transpose(hm);
  const auto innovation = hm * pht + r;
  Mat<3, 3> sinv;
  if (!cholesky(innovation, l) || !inverse(innovation, sinv))
    return UpdateResult::Invalid;
  last_nis_ = dot(residual, sinv * residual);
  const double limits[3] = {10.827566, 13.815511, 16.266236};
  if (!std::isfinite(last_nis_) || last_nis_ > limits[dim - 1])
    return UpdateResult::Outlier;
  const auto gain = pht * sinv;
  const ErrorVector dx = gain * residual;
  auto candidate = s;
  injectError(candidate, dx);
  if (!validState(candidate))
    return UpdateResult::Invalid;
  const auto a = identity<20>() - gain * hm;
  Covariance next = a * p_ * transpose(a) + gain * r * transpose(gain);
  // 左乘误差注入后的 reset：J_theta = I + 1/2 [delta theta x]。
  auto reset = identity<20>();
  const auto sr = skew({{dx[6], dx[7], dx[8]}}) * 0.5;
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      reset(6 + i, 6 + j) += sr(i, j);
  next = reset * next * transpose(reset);
  for (std::size_t i = 0; i < 20; ++i) {
    if (!(next(i, i) > 0))
      return UpdateResult::Invalid;
    for (std::size_t j = 0; j < i; ++j) {
      const double v = (next(i, j) + next(j, i)) / 2;
      next(i, j) = next(j, i) = v;
    }
  }
  if (!finite(next))
    return UpdateResult::Invalid;
  s = candidate;
  p_ = next;
  if (correction)
    *correction = dx;
  return UpdateResult::Accepted;
}
bool observationModel(const NominalState &s, const Measurement &m, Vec3 &res,
                      ObservationMatrix &h) {
  res = {};
  h = {};
  if (!finite(m.value) || m.dimensions < 1 || m.dimensions > 3)
    return false;
  const auto c = s.q_nb.matrix(), ct = transpose(c);
  switch (m.kind) {
  case MeasurementKind::Position: {
    if (m.dimensions < 2 || std::abs(m.value[0]) >= 89.5 * deg ||
        std::abs(m.value[1]) > pi)
      return false;
    auto target = s;
    target.lat_rad = m.value[0];
    target.lon_rad = m.value[1];
    target.height_m = m.dimensions == 3 ? m.value[2] : s.height_m;
    res = displacementNed(s, target);
    for (std::size_t i = 0; i < m.dimensions; ++i)
      h(i, i) = 1;
    break;
  }
  case MeasurementKind::VelocityNed:
    if (m.dimensions < 2)
      return false;
    // 二维平面视觉速度只约束 N/E，不假定或伪造向下速度。
    // 已知其尺度来自高度时仍保留 VO 尺度误差；GNSS 速度无此尺度项。
    {
      const double scale = m.source == FixSource::Vision ? 1 + s.vo_scale : 1;
      res = m.value - s.velocity_ned * scale;
      for (std::size_t i = 0; i < m.dimensions; ++i) {
        h(i, 3 + i) = scale;
        if (m.source == FixSource::Vision)
          h(i, 18) = s.velocity_ned[i];
      }
    }
    break;
  case MeasurementKind::VelocityBody: {
    if (m.dimensions < 2)
      return false;
    const auto v = ct * s.velocity_ned;
    res = m.value - v * (1 + s.vo_scale);
    const auto a = ct * skew(s.velocity_ned) * (1 + s.vo_scale);
    for (std::size_t i = 0; i < m.dimensions; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        h(i, 3 + j) = (1 + s.vo_scale) * ct(i, j);
        h(i, 6 + j) = a(i, j);
      }
      h(i, 18) = v[i];
    }
    break;
  }
  case MeasurementKind::Airspeed: {
    if (m.dimensions != 1 || m.value[0] <= 0)
      return false;
    const auto air = s.velocity_ned - Vec3{{s.wind_ne[0], s.wind_ne[1], 0}};
    const double speed = norm(air);
    if (speed < 0.1)
      return false;
    res[0] = m.value[0] - speed;
    for (std::size_t i = 0; i < 3; ++i)
      h(0, 3 + i) = air[i] / speed;
    h(0, 15) = -air[0] / speed;
    h(0, 16) = -air[1] / speed;
    break;
  }
  case MeasurementKind::BaroHeight:
    if (m.dimensions != 1)
      return false;
    res[0] = m.value[0] - (s.height_m + s.baro_bias);
    h(0, 2) = -1;
    h(0, 17) = 1;
    break;
  case MeasurementKind::MagHeading: {
    if (m.dimensions != 1)
      return false;
    const double d = c(0, 0) * c(0, 0) + c(1, 0) * c(1, 0);
    if (d < 1e-6)
      return false;
    res[0] = wrapAngle(m.value[0] - s.q_nb.toEuler()[2] - s.mag_bias);
    h(0, 6) = -c(0, 0) * c(2, 0) / d;
    h(0, 7) = -c(1, 0) * c(2, 0) / d;
    h(0, 8) = h(0, 19) = 1;
    break;
  }
  case MeasurementKind::RadarHeight:
    if (m.dimensions != 1 || m.value[0] < 0 ||
        !std::isfinite(m.terrain_height_m) || !finite(m.terrain_gradient))
      return false;
    res[0] = m.value[0] - (s.height_m - m.terrain_height_m);
    h(0, 0) = -m.terrain_gradient[0];
    h(0, 1) = -m.terrain_gradient[1];
    h(0, 2) = -1;
    break;
  default:
    return false;
  }
  return finite(res) && finite(h);
}
} // namespace nav

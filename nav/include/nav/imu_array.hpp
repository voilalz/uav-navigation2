#pragma once
#include "nav/types.hpp"
namespace nav {
struct ImuCalibration {
  Mat<3, 3> gyro_matrix{identity<3>()}, accel_matrix{identity<3>()};
  Vec3 gyro_offset{}, accel_offset{}, gyro_temp{}, accel_temp{};
  double reference_temperature_c{25};
};
template <std::size_t Count = 4> class ImuArray {
  static_assert(Count >= 2 && Count <= 32, "2..32 IMUs");

public:
  std::array<ImuCalibration, Count> calibration{};
  double gyro_gate{5 * deg}, accel_gate{2};
  std::uint32_t rejected_mask{};
  bool fuse(const std::array<ImuSample, Count> &raw, ImuSample &out) {
    std::array<ImuSample, Count> good{};
    std::array<bool, Count> valid{};
    std::size_t count = 0;
    rejected_mask = 0;
    Timestamp tmin = UINT64_MAX, tmax = 0;
    for (std::size_t i = 0; i < Count; ++i) {
      const auto &r = raw[i];
      if (!r.valid || !finite(r.gyro_rad_s) || !finite(r.accel_m_s2) ||
          !std::isfinite(r.temperature_c)) {
        rejected_mask |= 1u << i;
        continue;
      }
      const auto &c = calibration[i];
      const double dt = r.temperature_c - c.reference_temperature_c;
      good[i] = r;
      good[i].gyro_rad_s =
          c.gyro_matrix * (r.gyro_rad_s - c.gyro_offset - c.gyro_temp * dt);
      good[i].accel_m_s2 =
          c.accel_matrix * (r.accel_m_s2 - c.accel_offset - c.accel_temp * dt);
      if (!finite(good[i].gyro_rad_s) || !finite(good[i].accel_m_s2)) {
        rejected_mask |= 1u << i;
        continue;
      }
      valid[i] = true;
      ++count;
      tmin = std::min(tmin, r.t_us);
      tmax = std::max(tmax, r.t_us);
    }
    if (count < 2 || tmax - tmin > 100)
      return false;
    Vec3 median_g{}, median_a{};
    const auto sort_small = [](std::array<double, Count> &values,
                               std::size_t n) {
      for (std::size_t j = 1; j < n; ++j) {
        double value = values[j];
        std::size_t k = j;
        while (k && values[k - 1] > value) {
          values[k] = values[k - 1];
          --k;
        }
        values[k] = value;
      }
    };
    for (std::size_t k = 0; k < 3; ++k) {
      std::array<double, Count> g{}, a{};
      std::size_t n = 0;
      for (std::size_t i = 0; i < Count; ++i)
        if (valid[i]) {
          g[n] = good[i].gyro_rad_s[k];
          a[n++] = good[i].accel_m_s2[k];
        }
      sort_small(g, n);
      sort_small(a, n);
      median_g[k] = g[n / 2];
      median_a[k] = a[n / 2];
    }
    out = {};
    out.t_us = tmax;
    out.temperature_c = 0;
    count = 0;
    for (std::size_t i = 0; i < Count; ++i)
      if (valid[i]) {
        bool ok = true;
        for (std::size_t k = 0; k < 3; ++k)
          if (std::abs(good[i].gyro_rad_s[k] - median_g[k]) > gyro_gate ||
              std::abs(good[i].accel_m_s2[k] - median_a[k]) > accel_gate)
            ok = false;
        if (!ok) {
          rejected_mask |= 1u << i;
          continue;
        }
        out.gyro_rad_s = out.gyro_rad_s + good[i].gyro_rad_s;
        out.accel_m_s2 = out.accel_m_s2 + good[i].accel_m_s2;
        out.temperature_c += good[i].temperature_c;
        ++count;
      }
    if (count < 2)
      return false;
    out.gyro_rad_s = out.gyro_rad_s / double(count);
    out.accel_m_s2 = out.accel_m_s2 / double(count);
    out.temperature_c /= double(count);
    return true;
  }
};
template <std::size_t Factor = 5> class ImuDecimator {
  static_assert(Factor > 0, "nonzero factor");
  std::size_t count_{};
  ImuSample sum_{};
  Timestamp last_{};

public:
  bool add(const ImuSample &in, ImuSample &out) {
    if (!in.valid || !finite(in.gyro_rad_s) || !finite(in.accel_m_s2) ||
        in.t_us <= last_) {
      count_ = 0;
      sum_ = {};
      return false;
    }
    last_ = in.t_us;
    if (count_ == 0) {
      sum_ = {};
      sum_.temperature_c = 0;
    }
    sum_.gyro_rad_s = sum_.gyro_rad_s + in.gyro_rad_s;
    sum_.accel_m_s2 = sum_.accel_m_s2 + in.accel_m_s2;
    sum_.temperature_c += in.temperature_c;
    if (++count_ != Factor)
      return false;
    out = sum_;
    out.t_us = in.t_us;
    out.gyro_rad_s = out.gyro_rad_s / double(Factor);
    out.accel_m_s2 = out.accel_m_s2 / double(Factor);
    out.temperature_c /= double(Factor);
    count_ = 0;
    return true;
  }
};
} // namespace nav

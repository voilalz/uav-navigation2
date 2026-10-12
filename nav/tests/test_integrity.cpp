#include "nav/nav_core.hpp"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <new>
static std::size_t allocations = 0;
void *operator new(std::size_t n) {
  ++allocations;
  if (auto *p = std::malloc(n))
    return p;
  throw std::bad_alloc();
}
void *operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t) noexcept { std::free(p); }
using namespace nav;
int main() {
  int failures = 0;
  const auto before = allocations;
  NavCoreConfig config;
  config.absolute_timeout_us = config.gnss_timeout_us = 20000;
  config.vo_timeout_us = 10000;
  NavCore core(config);
  NominalState initial;
  initial.lat_rad = 31 * deg;
  initial.lon_rad = 121 * deg;
  initial.height_m = 1000;
  core.initialize(initial);
  const auto advance = [&](unsigned n) {
    for (unsigned i = 0; i < n; ++i) {
      ImuSample imu;
      imu.t_us = core.state().t_us + 2500;
      imu.gyro_rad_s = earth::rotation(initial.lat_rad);
      imu.accel_m_s2 = {
          {0, 0, -earth::gravity(initial.lat_rad, initial.height_m)}};
      if (!core.onImu(imu))
        ++failures;
    }
  };
  advance(20);
  Measurement z;
  z.t_us = core.state().t_us;
  z.value = {{initial.lat_rad, initial.lon_rad, initial.height_m}};
  z.covariance = identity<3>() * 100;
  z.source = FixSource::Terrain3D;
  if (!core.onMeasurement(z) || core.output().mode != Mode::Terrain)
    ++failures;
  advance(9);
  if (core.output().mode != Mode::DeadReckoning)
    ++failures;
  z.t_us = core.state().t_us;
  z.source = FixSource::Vision;
  if (!core.onMeasurement(z) || core.output().mode != Mode::Vision)
    ++failures;
  auto bad = z;
  bad.t_us = core.state().t_us + 1;
  if (core.onMeasurement(bad))
    ++failures;
  bad = z;
  bad.t_us = core.state().t_us;
  bad.source = FixSource::Gnss;
  bad.covariance(0, 1) = 10;
  if (core.onMeasurement(bad) || core.output().mode != Mode::Vision)
    ++failures;
  bad = z;
  bad.source = FixSource::Gnss;
  bad.covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
  if (core.onMeasurement(bad))
    ++failures;
  advance(9);
  z.t_us = core.state().t_us;
  z.source = FixSource::Gnss;
  if (!core.onMeasurement(z) || core.output().mode != Mode::Gnss)
    ++failures;
  NavCoreConfig wide;
  wide.reacquire_sigma_m = 10;
  NavCore lost(wide);
  lost.initialize(initial);
  if (lost.output().mode != Mode::Reacquire ||
      !(lost.output().health & CovLarge))
    ++failures;
  NavCoreConfig invalid;
  invalid.output_rate_hz = 0;
  NavCore invalid_core(invalid);
  invalid_core.initialize(initial);
  if (!(invalid_core.output().health & FilterInvalid))
    ++failures;
  // 未观测的高度不能通过水平二维位置量测直接置为输入值。
  auto horizontal = z;
  advance(1);
  horizontal.t_us = core.state().t_us;
  horizontal.source = FixSource::Vision;
  horizontal.dimensions = 2;
  horizontal.value[2] = 99999;
  const double altitude = core.state().height_m;
  if (!core.onMeasurement(horizontal) ||
      std::abs(core.state().height_m - altitude) > .01)
    ++failures;
  NavCore gap;
  gap.initialize(initial);
  auto first_fix = z;
  first_fix.t_us = 0;
  if (!gap.onMeasurement(first_fix) || gap.output().mode != Mode::Gnss)
    ++failures;
  ImuSample late;
  late.t_us = 200000;
  late.gyro_rad_s = earth::rotation(initial.lat_rad);
  late.accel_m_s2 = {{0, 0, -earth::gravity(initial.lat_rad, initial.height_m)}};
  if (gap.onImu(late) || !(gap.output().health & FilterInvalid) || gap.output().mode == Mode::Gnss)
    ++failures;
  late.t_us += 2500;
  if (gap.onImu(late))
    ++failures;
  auto aligned = initial;
  aligned.t_us = 200000;
  gap.initialize(aligned);
  if (!gap.onImu(late))
    ++failures;
  NavCoreConfig slow_config;
  slow_config.covariance_rate_hz = 5;
  NavCore slow(slow_config);
  slow.initialize(initial);
  if (!(slow.output().health & FilterInvalid))
    ++failures;
  Measurement planar;
  planar.kind = MeasurementKind::VelocityNed;
  planar.source = FixSource::Vision;
  planar.dimensions = 2;
  auto moving = initial;
  moving.velocity_ned = {{100, 50, -4}};
  moving.vo_scale = .01;
  Vec3 residual;
  ObservationMatrix jacobian;
  if (!observationModel(moving, planar, residual, jacobian) ||
      std::abs(residual[0] + 101) > 1e-9 || std::abs(jacobian(0, 18) - 100) > 1e-9 ||
      jacobian(0, 5) != 0 || jacobian(1, 5) != 0)
    ++failures;
  if (allocations != before)
    ++failures;
  std::printf("integrity: %d failures; navigation heap allocations %zu; "
              "sizeof(NavCore) %zu bytes\n",
              failures, allocations - before, sizeof(NavCore));
  return failures ? 1 : 0;
}

#include "nav/nav_core.hpp"
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace nav;
static int failures = 0, checks = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    ++checks;                                                                  \
    if (!(c)) {                                                                \
      ++failures;                                                              \
      std::printf("FAIL line %d: %s\n", __LINE__, #c);                         \
    }                                                                          \
  } while (0)
static bool near(double a, double b, double e) { return std::abs(a - b) < e; }
static NominalState initial() {
  NominalState s;
  s.lat_rad = 31 * deg;
  s.lon_rad = 121 * deg;
  s.height_m = 1500;
  return s;
}
static ImuSample stationary(const NominalState &s, Timestamp t) {
  return {t, transpose(s.q_nb.matrix()) * earth::rotation(s.lat_rad),
          transpose(s.q_nb.matrix()) *
              Vec3{{0, 0, -earth::gravity(s.lat_rad, s.height_m)}},
          25, true};
}
static Measurement position(const NominalState &s, Timestamp t,
                            FixSource source = FixSource::Vision) {
  Measurement m;
  m.t_us = t;
  m.source = source;
  m.value = {{s.lat_rad, s.lon_rad, s.height_m}};
  m.covariance = identity<3>() * 100;
  return m;
}
int main() {
  auto q = Quaternion::fromEuler(.2, -.3, 1.1);
  auto e = q.toEuler();
  CHECK(near(e[0], .2, 1e-12) && near(e[1], -.3, 1e-12) &&
        near(e[2], 1.1, 1e-12));
  Vec3 rv{{.3, -.2, .1}};
  CHECK(norm(Quaternion::exp(rv).log() - rv) < 1e-12);
  Mat<3, 3> a = identity<3>() * 3;
  a(0, 1) = a(1, 0) = .4;
  Mat<3, 3> b;
  CHECK(inverse(a, b));
  CHECK(near((a * b)(0, 0), 1, 1e-12));
  a(2, 2) = -1;
  CHECK(!cholesky(a, b));
  // 静止 60s：角速度是地球自转，不是零；比力不是含重力加速度。
  NavCore core;
  auto s = initial();
  core.initialize(s);
  for (int i = 1; i <= 24000; ++i)
    CHECK(core.onImu(stationary(s, Timestamp(i) * 2500)));
  CHECK(norm(core.state().velocity_ned) < .01);
  CHECK(norm(displacementNed(s, core.state())) < .05);
  CHECK(core.statistics().outputs == 1800);
  CHECK(!core.onImu(stationary(s, core.state().t_us)));
  auto bad = stationary(s, core.state().t_us + 2500);
  bad.accel_m_s2[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK(!core.onImu(bad));
  Eskf filter;
  auto state = initial();
  auto fix = state;
  offsetPosition(fix, {{20, 0, 0}});
  auto m = position(fix, 0);
  Vec3 res;
  ObservationMatrix h;
  CHECK(observationModel(state, m, res, h));
  CHECK(filter.update(state, res, h, m.covariance, 3) ==
        UpdateResult::Accepted);
  CHECK(near(displacementNed(initial(), state)[0], 20 * 1600.0 / 1700, 1e-4));
  auto remote = state;
  offsetPosition(remote, {{2000, 0, 0}});
  m = position(remote, 0);
  CHECK(observationModel(state, m, res, h));
  CHECK(filter.update(state, res, h, m.covariance, 3) == UpdateResult::Outlier);
  m.covariance(0, 0) = -1;
  CHECK(filter.update(state, res, h, m.covariance, 3) == UpdateResult::Invalid);
  // 解析雅可比与独立数值差分，检测左右乘、交叉积符号。
  state.q_nb = q;
  state.velocity_ned = {{120, 45, -3}};
  state.vo_scale = .008;
  for (auto kind :
       {MeasurementKind::MagHeading, MeasurementKind::VelocityBody}) {
    m = {};
    m.kind = kind;
    m.dimensions = kind == MeasurementKind::MagHeading ? 1 : 3;
    CHECK(observationModel(state, m, res, h));
    for (std::size_t j = 3; j < 20; ++j) {
      auto shifted = state;
      ErrorVector dx;
      dx[j] = 1e-6;
      injectError(shifted, dx);
      Vec3 r2;
      ObservationMatrix h2;
      CHECK(observationModel(shifted, m, r2, h2));
      for (std::size_t i = 0; i < m.dimensions; ++i)
        CHECK(near((res[i] - r2[i]) / 1e-6, h(i, j), 2e-4));
    }
  }
  Mat<20, 20> chol;
  CHECK(cholesky(filter.covariance(), chol));
  // 250ms 延迟必须按曝光时刻位置求新息，不能把当前的41.7m航行误认为误差。
  NavCore moving;
  s = initial();
  s.velocity_ned = {{166.6666667, 0, 0}};
  moving.initialize(s);
  auto truth = s;
  for (int i = 1; i <= 400; ++i) {
    const auto w =
        earth::rotation(truth.lat_rad) +
        earth::transport(truth.lat_rad, truth.height_m, truth.velocity_ned);
    const auto f =
        Vec3{{0, 0, -earth::gravity(truth.lat_rad, truth.height_m)}} +
        cross(earth::rotation(truth.lat_rad) * 2 +
                  earth::transport(truth.lat_rad, truth.height_m,
                                   truth.velocity_ned),
              truth.velocity_ned);
    ImuSample sample{Timestamp(i) * 2500, w, f};
    CHECK(moving.onImu(sample));
    offsetPosition(truth, truth.velocity_ned * .0025);
    truth.t_us = sample.t_us;
  }
  auto exposed = truth;
  offsetPosition(exposed, truth.velocity_ned * (-.25));
  exposed.t_us = 750000;
  m = position(exposed, 750000);
  m.dimensions = 2;
  CHECK(moving.onMeasurement(m));
  CHECK(norm(displacementNed(truth, moving.state())) < .2);
  CHECK(moving.output().mode == Mode::Vision);
  CHECK(!moving.onMeasurement(m));
  m.t_us = 1000001;
  CHECK(!moving.onMeasurement(m));
  m.t_us = 1;
  CHECK(!moving.onMeasurement(m));
  m = position(truth, 1000000, FixSource::Gnss);
  offsetPosition(exposed, {{10000, 0, 0}});
  m.value = {{exposed.lat_rad, exposed.lon_rad, exposed.height_m}};
  CHECK(!moving.onMeasurement(m));
  CHECK(moving.output().mode == Mode::Vision);
  m = position(truth, 1000000, FixSource::Gnss);
  CHECK(moving.onMeasurement(m));
  CHECK(moving.output().mode == Mode::Gnss);
  StateHistory<4> hist;
  hist.push(s);
  auto s2 = s;
  offsetPosition(s2, {{10, 0, 0}});
  s2.t_us = 10000;
  hist.push(s2);
  NominalState interpolated;
  CHECK(hist.sample(5000, interpolated));
  CHECK(near(displacementNed(s, interpolated)[0], 5, 1e-5));
  CHECK(!hist.sample(20000, interpolated));
  std::printf("core: %d checks, %d failures\n", checks, failures);
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}

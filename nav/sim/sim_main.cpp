#include "nav/nav_core.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <vector>
using namespace nav;
namespace {
constexpr double speed = 600.0 / 3.6, dt = .0025;
struct Motion {
  double yaw{}, yaw_rate{}, roll{}, roll_rate{}, height{}, vd{}, vd_rate{};
  Vec3 velocity{}, acceleration{};
  Quaternion q{};
  Vec3 gyro{};
};
Motion motion(double t) {
  Motion m;
  double yaw = .2, rate = 0, rate_dot = 0;
  const double start[3] = {150, 420, 850}, length[3] = {35, 45, 30},
               angle[3] = {.7, -1.0, .6};
  for (int i = 0; i < 3; ++i) {
    if (t >= start[i] + length[i])
      yaw += angle[i];
    else if (t > start[i]) {
      const double u = (t - start[i]) / length[i];
      yaw += angle[i] * (u - std::sin(2 * pi * u) / (2 * pi));
      rate = angle[i] / length[i] * (1 - std::cos(2 * pi * u));
      rate_dot =
          angle[i] * 2 * pi / (length[i] * length[i]) * std::sin(2 * pi * u);
    }
  }
  m.yaw = yaw;
  m.yaw_rate = rate;
  m.roll = std::atan(speed * rate / 9.8);
  m.roll_rate =
      speed * rate_dot / (9.8 * (1 + std::pow(speed * rate / 9.8, 2)));
  m.height = 1500 + 40 * std::sin(2 * pi * t / 300);
  m.vd = -40 * 2 * pi / 300 * std::cos(2 * pi * t / 300);
  m.vd_rate = 40 * std::pow(2 * pi / 300, 2) * std::sin(2 * pi * t / 300);
  m.velocity = {{speed * std::cos(yaw), speed * std::sin(yaw), m.vd}};
  m.acceleration = {
      {-speed * std::sin(yaw) * rate, speed * std::cos(yaw) * rate, m.vd_rate}};
  const double pitch = std::atan2(-m.vd, speed),
               pitch_rate =
                   -m.vd_rate / speed / (1 + std::pow(m.vd / speed, 2));
  m.q = Quaternion::fromEuler(m.roll, pitch, yaw);
  m.gyro = {{m.roll_rate - rate * std::sin(pitch),
             pitch_rate * std::cos(m.roll) +
                 rate * std::cos(pitch) * std::sin(m.roll),
             -pitch_rate * std::sin(m.roll) +
                 rate * std::cos(pitch) * std::cos(m.roll)}};
  return m;
}
struct Pending {
  Timestamp arrival{};
  Measurement measurement{};
  bool injected_outlier{};
};
void writeMeasurement(std::ostream &o, Timestamp arrival,
                      const Measurement &m) {
  o << "M," << arrival << ',' << m.t_us << ',' << unsigned(m.kind) << ','
    << unsigned(m.source) << ',' << unsigned(m.dimensions);
  for (auto x : m.value.v)
    o << ',' << x;
  for (auto x : m.covariance.v)
    o << ',' << x;
  o << ',' << m.terrain_height_m << ',' << m.terrain_gradient[0] << ','
    << m.terrain_gradient[1] << '\n';
}
bool number(const char *s, double &v) {
  char *end = nullptr;
  v = std::strtod(s, &end);
  return end != s && !*end && std::isfinite(v);
}
} // namespace
int main(int argc, char **argv) {
  double duration = 1200, seed_arg = 1;
  if ((argc > 1 && !number(argv[1], duration)) ||
      (argc > 2 && !number(argv[2], seed_arg)) || duration < 1 ||
      duration > 7200 || seed_arg < 0 || seed_arg > 4294967295.0 ||
      std::floor(seed_arg) != seed_arg || argc > 5) {
    std::cerr << "usage: nav_sim [duration_s 1..7200] [seed] [output_dir|-] "
                 "[combined|relative|inertial]\n";
    return 2;
  }
  const std::string output = argc > 3 ? argv[3] : "out/sim",
                    scenario = argc > 4 ? argv[4] : "combined";
  if (scenario != "combined" && scenario != "relative" &&
      scenario != "inertial") {
    std::cerr << "invalid scenario\n";
    return 2;
  }
  const bool log = output != "-";
  std::ofstream events, estimates, truth_log;
  if (log) {
    std::error_code ec;
    std::filesystem::create_directories(output, ec);
    if (ec) {
      std::cerr << ec.message() << '\n';
      return 3;
    }
    events.open(output + "/events.csv");
    estimates.open(output + "/estimate.csv");
    truth_log.open(output + "/truth.csv");
    if (!events || !estimates || !truth_log)
      return 3;
    events << std::setprecision(17);
    estimates << std::setprecision(17);
    truth_log << std::setprecision(17);
    estimates << "t_us,lat_rad,lon_rad,height_m,vn,ve,vd,qw,qx,qy,qz,h_sigma,v_"
                 "sigma,mode,health,error_n,error_e,error_d,nees_pos,nis\n";
    truth_log << "t_us,lat_rad,lon_rad,height_m,vn,ve,vd,qw,qx,qy,qz\n";
  }
  std::mt19937 random{std::uint32_t(seed_arg)};
  std::normal_distribution<double> normal(0, 1);
  const auto gaussian = [&](double sigma) { return normal(random) * sigma; };
  NominalState truth;
  truth.lat_rad = 31 * deg;
  truth.lon_rad = 121 * deg;
  truth.height_m = 1500;
  auto start = motion(0);
  truth.velocity_ned = start.velocity;
  truth.q_nb = start.q;
  auto init = truth;
  offsetPosition(init, {{10, -8, -5}});
  init.q_nb =
      (Quaternion::exp({{.05 * deg, -.08 * deg, .15 * deg}}) * init.q_nb)
          .normalized();
  NavCore core;
  core.initialize(init);
  if (log) {
    events << "S," << init.t_us << ',' << init.lat_rad << ',' << init.lon_rad
           << ',' << init.height_m;
    for (double x : init.velocity_ned.v)
      events << ',' << x;
    events << ',' << init.q_nb.w << ',' << init.q_nb.x << ',' << init.q_nb.y
           << ',' << init.q_nb.z << '\n';
  }
  const Vec3 bg{{8 * deg / 3600, -6 * deg / 3600, 10 * deg / 3600}},
      ba{{.012, -.015, .018}};
  std::vector<Pending> pending;
  pending.reserve(100);
  std::vector<double> errors;
  errors.reserve(std::size_t(duration * 30) + 1);
  std::uint64_t vo_next = 33333, baro_next = 20000, mag_next = 100000,
                fix_next = 10000000, terrain_next = 45000000,
                gnss_next = 1000000, outputs = 0, known_outliers = 0,
                outliers_rejected = 0;
  double square = 0, max_error = 0, nees_sum = 0, nis_sum = 0;
  std::uint64_t nis_count = 0;
  auto deliver = [&](Timestamp arrival, const Measurement &m,
                     bool bad = false) {
    if (log)
      writeMeasurement(events, arrival, m);
    const bool accepted = core.onMeasurement(m);
    if (bad) {
      ++known_outliers;
      if (!accepted)
        ++outliers_rejected;
    }
    if (accepted) {
      nis_sum += core.filter().lastNis();
      ++nis_count;
    }
  };
  const auto begin = std::chrono::steady_clock::now();
  const auto steps = std::uint64_t(std::floor(duration / dt));
  for (std::uint64_t i = 1; i <= steps; ++i) {
    const double t = double(i) * dt;
    const Timestamp now = i * 2500;
    const auto m = motion(t - dt / 2), at = motion(t);
    offsetPosition(truth, m.velocity * dt);
    truth.t_us = now;
    truth.height_m = at.height;
    truth.velocity_ned = at.velocity;
    truth.q_nb = at.q;
    const auto ct = transpose(m.q.matrix());
    const auto wie = earth::rotation(truth.lat_rad),
               wen =
                   earth::transport(truth.lat_rad, truth.height_m, m.velocity);
    const Vec3 ideal_g = m.gyro + ct * (wie + wen),
               ideal_a =
                   ct * (m.acceleration -
                         Vec3{{0, 0,
                               earth::gravity(truth.lat_rad, truth.height_m)}} +
                         cross(wie * 2 + wen, m.velocity));
    ImuSample imu;
    imu.t_us = now;
    for (unsigned k = 0; k < 3; ++k) {
      imu.gyro_rad_s[k] = ideal_g[k] * (1 + .0005) + bg[k] +
                          gaussian(.3 * deg / 60 / std::sqrt(dt));
      imu.accel_m_s2[k] =
          ideal_a[k] * (1 + .0005) + ba[k] + gaussian(.015 / std::sqrt(dt));
    }
    if (log) {
      events << "I," << now << ',' << now;
      for (double x : imu.gyro_rad_s.v)
        events << ',' << x;
      for (double x : imu.accel_m_s2.v)
        events << ',' << x;
      events << '\n';
    }
    if (!core.onImu(imu)) {
      std::cerr << "IMU propagation rejected at " << t << '\n';
      return 4;
    }
    // 量测按到达时间排序；时间戳仍为曝光时间。
    for (std::size_t j = 0; j < pending.size();) {
      if (pending[j].arrival <= now) {
        deliver(now, pending[j].measurement, pending[j].injected_outlier);
        pending.erase(pending.begin() + std::ptrdiff_t(j));
      } else
        ++j;
    }
    if (now >= baro_next) {
      baro_next += 20000;
      Measurement z;
      z.t_us = now;
      z.kind = MeasurementKind::BaroHeight;
      z.dimensions = 1;
      z.value[0] = truth.height_m + 25 + gaussian(3);
      z.covariance(0, 0) = 9;
      deliver(now, z);
      if (scenario != "inertial") {
        z.kind = MeasurementKind::Airspeed;
        z.value[0] =
            norm(truth.velocity_ned - Vec3{{4, -2, 0}}) + gaussian(1.5);
        z.covariance(0, 0) = 2.25;
        deliver(now, z);
      }
    }
    if (scenario != "inertial" && now >= mag_next) {
      mag_next += 100000;
      Measurement z;
      z.t_us = now;
      z.kind = MeasurementKind::MagHeading;
      z.dimensions = 1;
      z.value[0] = at.yaw + .5 * deg + gaussian(1.5 * deg);
      z.covariance(0, 0) = std::pow(1.5 * deg, 2);
      deliver(now, z);
    }
    if (scenario != "inertial" && now >= vo_next) {
      vo_next += 33333;
      if (!(t >= 600 && t < 610)) {
        Measurement z;
        z.t_us = now;
        z.kind = MeasurementKind::VelocityBody;
        z.source = FixSource::Vision;
        z.value =
            transpose(truth.q_nb.matrix()) * truth.velocity_ned * (1 + .008);
        for (unsigned k = 0; k < 3; ++k)
          z.value[k] += gaussian(k == 2 ? .4 : 1);
        z.covariance(0, 0) = z.covariance(1, 1) = 1;
        z.covariance(2, 2) = .16;
        pending.push_back({now + 30000, z, false});
      }
    }
    if (scenario == "combined" && now >= gnss_next && t <= 8) {
      gnss_next += 1000000;
      auto noisy = truth;
      offsetPosition(noisy, {{gaussian(2), gaussian(2), gaussian(3)}});
      Measurement z;
      z.t_us = now;
      z.source = FixSource::Gnss;
      z.value = {{noisy.lat_rad, noisy.lon_rad, noisy.height_m}};
      z.covariance = identity<3>() * 4;
      z.covariance(2, 2) = 9;
      deliver(now, z);
      z.kind = MeasurementKind::VelocityNed;
      z.value = truth.velocity_ned;
      for (auto &x : z.value.v)
        x += gaussian(.2);
      z.covariance = identity<3>() * .04;
      deliver(now, z);
    }
    if (scenario == "combined" && now >= fix_next) {
      fix_next += 30000000;
      if (!(t >= 600 && t < 720)) {
        auto noisy = truth;
        const bool outlier = (random() % 20) == 0;
        offsetPosition(
            noisy, {{gaussian(10) + (outlier ? 2000 : 0), gaussian(10), 0}});
        Measurement z;
        z.t_us = now;
        z.source = FixSource::Vision;
        z.dimensions = 2;
        z.value = {{noisy.lat_rad, noisy.lon_rad, noisy.height_m}};
        z.covariance = identity<3>() * 100;
        pending.push_back({now + 250000, z, outlier});
      }
    }
    if (scenario == "combined" && now >= terrain_next) {
      terrain_next += 60000000;
      if (!(t >= 600 && t < 720)) {
        auto noisy = truth;
        offsetPosition(noisy, {{gaussian(20), gaussian(20), gaussian(8)}});
        Measurement z;
        z.t_us = now;
        z.source = FixSource::Terrain3D;
        z.value = {{noisy.lat_rad, noisy.lon_rad, noisy.height_m}};
        z.covariance = identity<3>() * 400;
        z.covariance(2, 2) = 64;
        pending.push_back({now + 400000, z, false});
      }
    }
    NavOutput out;
    if (core.takeOutput(out)) {
      ++outputs;
      const auto err = displacementNed(out.state, truth);
      const double horizontal = std::hypot(err[0], err[1]);
      errors.push_back(horizontal);
      square += horizontal * horizontal;
      max_error = std::max(max_error, horizontal);
      Mat<3, 3> pp;
      for (unsigned r = 0; r < 3; ++r)
        for (unsigned c = 0; c < 3; ++c)
          pp(r, c) = core.filter().covariance()(r, c);
      Mat<3, 3> pinv;
      const double nees = inverse(pp, pinv) ? dot(err, pinv * err) : 0;
      nees_sum += nees;
      if (log) {
        estimates << now << ',' << out.state.lat_rad << ',' << out.state.lon_rad
                  << ',' << out.state.height_m;
        for (double x : out.state.velocity_ned.v)
          estimates << ',' << x;
        estimates << ',' << out.state.q_nb.w << ',' << out.state.q_nb.x << ','
                  << out.state.q_nb.y << ',' << out.state.q_nb.z << ','
                  << out.horizontal_sigma_m << ',' << out.vertical_sigma_m
                  << ',' << unsigned(out.mode) << ',' << out.health;
        for (double x : err.v)
          estimates << ',' << x;
        estimates << ',' << nees << ',' << core.filter().lastNis() << '\n';
        truth_log << now << ',' << truth.lat_rad << ',' << truth.lon_rad << ','
                  << truth.height_m;
        for (double x : truth.velocity_ned.v)
          truth_log << ',' << x;
        truth_log << ',' << truth.q_nb.w << ',' << truth.q_nb.x << ','
                  << truth.q_nb.y << ',' << truth.q_nb.z << '\n';
      }
    }
  }
  if (errors.empty())
    return 5;
  std::sort(errors.begin(), errors.end());
  const double rms = std::sqrt(square / outputs),
               cep = errors[errors.size() / 2],
               p95 = errors[std::min(errors.size() - 1,
                                     std::size_t(.95 * errors.size()))];
  const auto stats = core.statistics();
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
          .count();
  std::ostringstream summary;
  summary << std::setprecision(10) << "{\n  \"duration_s\": " << steps * dt
          << ", \"seed\": " << std::uint32_t(seed_arg) << ", \"scenario\": \""
          << scenario << "\",\n  \"horizontal_rms_m\": " << rms
          << ", \"cep_m\": " << cep << ", \"horizontal_p95_m\": " << p95
          << ", \"horizontal_max_m\": " << max_error
          << ",\n  \"output_hz\": " << outputs / (steps * dt)
          << ", \"output_count\": " << outputs
          << ",\n  \"position_nees_mean\": " << nees_sum / outputs
          << ", \"accepted_nis_mean\": "
          << (nis_count ? nis_sum / nis_count : 0)
          << ",\n  \"measurements_accepted\": " << stats.measurements_accepted
          << ", \"measurements_rejected\": " << stats.measurements_rejected
          << ", \"time_rejected\": " << stats.time_rejected
          << ",\n  \"injected_outliers\": " << known_outliers
          << ", \"injected_outliers_rejected\": " << outliers_rejected
          << ", \"wall_time_s\": " << elapsed << "\n}\n";
  std::cout << summary.str();
  if (log) {
    std::ofstream report(output + "/summary.json");
    report << summary.str();
    if (!report || !events || !truth_log || !estimates)
      return 3;
  }
  return std::isfinite(rms) &&
                 outputs == std::uint64_t(std::floor(steps * dt * 30 + 1e-6))
             ? 0
             : 6;
}

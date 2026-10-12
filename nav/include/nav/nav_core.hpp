#pragma once
#include "nav/state_history.hpp"
#include "nav/strapdown.hpp"
namespace nav {
struct NavCoreConfig {
  double output_rate_hz{30}, covariance_rate_hz{100}, max_imu_gap_s{0.05};
  Timestamp absolute_timeout_us{30000000}, gnss_timeout_us{2000000},
      vo_timeout_us{1000000}, max_measurement_age_us{2000000};
  double reacquire_sigma_m{300};
};
class NavCore {
public:
  explicit NavCore(const NavCoreConfig &c = {}) : config_(c) {}
  void initialize(const NominalState &);
  bool onImu(const ImuSample &);
  bool onMeasurement(const Measurement &);
  NavOutput output() const;
  bool takeOutput(NavOutput &);
  const NominalState &state() const { return state_; }
  const Eskf &filter() const { return filter_; }
  const Statistics &statistics() const { return stats_; }

private:
  NavCoreConfig config_;
  NominalState state_{};
  Eskf filter_{};
  StateHistory<> history_{};
  Statistics stats_{};
  Vec3 rate_{}, force_integral_{};
  double covariance_dt_{}, next_output_us_{};
  bool initialized_{}, ready_{};
  std::uint32_t health_{};
  Timestamp last_gnss_{}, last_vision_{}, last_terrain_{}, last_vo_{};
  bool have_gnss_{}, have_vision_{}, have_terrain_{}, have_vo_{};
  std::array<Timestamp, 35> measurement_times_{};
  std::array<bool, 35> have_measurement_{};
  Timestamp output_epoch_us_{};
};
} // namespace nav

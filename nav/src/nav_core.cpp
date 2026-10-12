#include "nav/nav_core.hpp"
namespace nav {
void NavCore::initialize(const NominalState &s) {
  state_ = s;
  state_.q_nb = s.q_nb.normalized();
  filter_.reset();
  history_.clear();
  stats_ = {};
  health_ = 0;
  ready_ = false;
  rate_ = {};
  force_integral_ = {};
  covariance_dt_ = 0;
  have_gnss_ = have_vision_ = have_terrain_ = have_vo_ = false;
  have_measurement_.fill(false);
  measurement_times_.fill(0);
  initialized_ = validState(s) && std::isfinite(config_.output_rate_hz) &&
                 config_.output_rate_hz > 0 && config_.output_rate_hz <= 400 &&
                 std::isfinite(config_.covariance_rate_hz) &&
                 config_.covariance_rate_hz >= 20 &&
                 config_.covariance_rate_hz <= 400 &&
                 config_.max_imu_gap_s > 0 && config_.max_imu_gap_s <= .1 &&
                 std::isfinite(config_.reacquire_sigma_m) && config_.reacquire_sigma_m > 0;
  if (!initialized_) {
    health_ = FilterInvalid;
    return;
  }
  history_.push(state_);
  output_epoch_us_ = s.t_us;
  next_output_us_ = double(s.t_us) + 1e6 / config_.output_rate_hz;
}
bool NavCore::onImu(const ImuSample &imu) {
  if (!initialized_ || !imu.valid || !finite(imu.gyro_rad_s) ||
      !finite(imu.accel_m_s2) || norm(imu.gyro_rad_s) > 40 ||
      norm(imu.accel_m_s2) > 500 || !std::isfinite(imu.temperature_c)) {
    ++stats_.imu_rejected;
    health_ |= ImuInvalid;
    return false;
  }
  if (imu.t_us <= state_.t_us) {
    ++stats_.imu_rejected;
    health_ |= TimeInvalid;
    return false;
  }
  const double dt = double(imu.t_us - state_.t_us) * 1e-6;
  if (dt > config_.max_imu_gap_s) {
    ++stats_.imu_rejected;
    // 没有可靠角增量时不能跨越长断流。显式锁定失效，重新对准后 initialize 恢复。
    health_ |= TimeInvalid | ImuInvalid | FilterInvalid;
    initialized_ = false;
    ready_ = false;
    return false;
  }
  auto next = state_;
  if (!propagateStrapdown(next, imu, dt)) {
    ++stats_.imu_rejected;
    health_ |= FilterInvalid;
    return false;
  }
  force_integral_ = force_integral_ + (imu.accel_m_s2 - state_.accel_bias) * dt;
  covariance_dt_ += dt;
  if (covariance_dt_ + 1e-10 >= 1 / config_.covariance_rate_hz) {
    if (!filter_.predict(next, force_integral_ / covariance_dt_,
                         covariance_dt_)) {
      ++stats_.imu_rejected;
      health_ |= FilterInvalid;
      initialized_ = false;
      ready_ = false;
      return false;
    }
    force_integral_ = {};
    covariance_dt_ = 0;
    history_.push(next);
  }
  state_ = next;
  rate_ = imu.gyro_rad_s - state_.gyro_bias;
  ++stats_.imu_accepted;
  health_ &= ~(ImuInvalid | TimeInvalid);
  if (double(state_.t_us) + 1e-6 >= next_output_us_) {
    ready_ = true;
    ++stats_.outputs;
    next_output_us_ = double(output_epoch_us_) + (double(stats_.outputs) + 1) *
                                                     1e6 /
                                                     config_.output_rate_hz;
  }
  return true;
}
bool NavCore::onMeasurement(const Measurement &m) {
  const auto kind = static_cast<unsigned>(m.kind),
             source = static_cast<unsigned>(m.source);
  if (!initialized_ || kind >= 7 || source >= 5 || !finite(m.covariance)) {
    ++stats_.measurements_rejected;
    health_ |= MeasurementRejected;
    return false;
  }
  const auto index = kind * 5 + source;
  if (m.t_us > state_.t_us ||
      state_.t_us - m.t_us > config_.max_measurement_age_us ||
      (have_measurement_[index] && m.t_us <= measurement_times_[index])) {
    ++stats_.time_rejected;
    ++stats_.measurements_rejected;
    health_ |= TimeInvalid | MeasurementRejected;
    return false;
  }
  NominalState at = state_;
  if (m.t_us != state_.t_us && !history_.sample(m.t_us, at)) {
    const auto *last = history_.latest();
    if (!last || m.t_us < last->t_us) {
      ++stats_.time_rejected;
      ++stats_.measurements_rejected;
      return false;
    }
    const double u =
        double(m.t_us - last->t_us) / double(state_.t_us - last->t_us);
    at = *last;
    offsetPosition(at, displacementNed(*last, state_) * u);
    at.q_nb = interpolate(last->q_nb, state_.q_nb, u);
    at.velocity_ned = last->velocity_ned * (1 - u) + state_.velocity_ned * u;
    at.t_us = m.t_us;
  }
  Vec3 residual;
  ObservationMatrix h;
  ErrorVector correction;
  if (!observationModel(at, m, residual, h)) {
    ++stats_.measurements_rejected;
    health_ |= MeasurementRejected;
    return false;
  }
  const auto result = filter_.update(state_, residual, h, m.covariance,
                                     m.dimensions, &correction);
  if (result != UpdateResult::Accepted) {
    ++stats_.measurements_rejected;
    health_ |= MeasurementRejected;
    if (result == UpdateResult::Outlier)
      ++stats_.outliers;
    return false;
  }
  history_.rebase(correction);
  measurement_times_[index] = m.t_us;
  have_measurement_[index] = true;
  ++stats_.measurements_accepted;
  health_ &= ~(MeasurementRejected | TimeInvalid);
  if (m.kind == MeasurementKind::Position) {
    switch (m.source) {
    case FixSource::Gnss:
      last_gnss_ = m.t_us;
      have_gnss_ = true;
      break;
    case FixSource::Vision:
      last_vision_ = m.t_us;
      have_vision_ = true;
      break;
    case FixSource::Terrain3D:
    case FixSource::Tercom:
      last_terrain_ = m.t_us;
      have_terrain_ = true;
      break;
    default:
      break;
    }
  }
  if (m.kind == MeasurementKind::VelocityBody ||
      (m.kind == MeasurementKind::VelocityNed &&
       m.source == FixSource::Vision)) {
    last_vo_ = m.t_us;
    have_vo_ = true;
  }
  return true;
}
NavOutput NavCore::output() const {
  NavOutput o;
  o.state = state_;
  o.angular_rate_body = rate_;
  const auto &p = filter_.covariance();
  o.horizontal_sigma_m = std::sqrt(std::max(0.0, p(0, 0) + p(1, 1)));
  o.vertical_sigma_m = std::sqrt(std::max(0.0, p(2, 2)));
  o.health = health_;
  const auto recent = [this](bool have, Timestamp stamp, Timestamp timeout) {
    return have && stamp <= state_.t_us && state_.t_us - stamp <= timeout;
  };
  if (!initialized_) {
    o.mode = Mode::DeadReckoning;
    o.health |= FilterInvalid | AbsoluteLost;
  } else if (o.horizontal_sigma_m > config_.reacquire_sigma_m) {
    o.mode = Mode::Reacquire;
    o.health |= CovLarge;
  } else if (recent(have_gnss_, last_gnss_, config_.gnss_timeout_us))
    o.mode = Mode::Gnss;
  else if (recent(have_vision_, last_vision_, config_.absolute_timeout_us))
    o.mode = Mode::Vision;
  else if (recent(have_terrain_, last_terrain_, config_.absolute_timeout_us))
    o.mode = Mode::Terrain;
  else {
    o.mode = Mode::DeadReckoning;
    o.health |= AbsoluteLost;
  }
  if (!recent(have_vo_, last_vo_, config_.vo_timeout_us))
    o.health |= VoLost;
  return o;
}
bool NavCore::takeOutput(NavOutput &out) {
  if (!ready_)
    return false;
  out = output();
  ready_ = false;
  return true;
}
} // namespace nav

#pragma once
#include "nav/earth.hpp"
namespace nav {
constexpr std::size_t error_dimension = 20;
using ErrorVector = Vec<error_dimension>;
using Covariance = Mat<error_dimension, error_dimension>;
using ObservationMatrix = Mat<3, error_dimension>;
enum class UpdateResult { Accepted, Outlier, Invalid };
struct NoiseConfig {
  double gyro_noise{0.3 * deg / 60}, accel_noise{0.03},
      gyro_bias_walk{0.003 * deg / 60}, accel_bias_walk{0.0003},
      wind_walk{0.15}, baro_walk{0.03}, scale_walk{0.00001},
      mag_walk{0.002 * deg};
};
void injectError(NominalState &, const ErrorVector &);
bool observationModel(const NominalState &, const Measurement &, Vec3 &residual,
                      ObservationMatrix &h);
class Eskf {
public:
  Eskf();
  void reset();
  bool predict(const NominalState &, const Vec3 &specific_force_body,
               double dt);
  UpdateResult update(NominalState &, const Vec3 &, const ObservationMatrix &,
                      const Mat<3, 3> &r, std::size_t dim,
                      ErrorVector *correction = nullptr);
  const Covariance &covariance() const { return p_; }
  Covariance &covarianceForInitialization() { return p_; }
  NoiseConfig noise{};
  double lastNis() const { return last_nis_; }

private:
  Covariance p_{};
  double last_nis_{};
};
} // namespace nav

#pragma once
#include "nav/types.hpp"
namespace nav {
namespace earth {
constexpr double a = 6378137.0, e2 = 0.0066943799901413165, omega = 7.292115e-5;
inline Vec<2> radii(double lat) {
  const double d = 1 - e2 * std::pow(std::sin(lat), 2);
  return {{a * (1 - e2) / std::pow(d, 1.5), a / std::sqrt(d)}};
}
inline double gravity(double lat, double h) {
  const double s = std::pow(std::sin(lat), 2);
  return 9.7803253359 * (1 + 0.00193185265241 * s) / std::sqrt(1 - e2 * s) *
         (1 - 2 * h / a);
}
inline Vec3 rotation(double lat) {
  return {{omega * std::cos(lat), 0, -omega * std::sin(lat)}};
}
inline Vec3 transport(double lat, double h, const Vec3 &v) {
  const auto r = radii(lat);
  return {{v[1] / (r[1] + h), -v[0] / (r[0] + h),
           -v[1] * std::tan(lat) / (r[1] + h)}};
}
} // namespace earth
inline Vec3 displacementNed(const NominalState &from, const NominalState &to) {
  const auto r = earth::radii((from.lat_rad + to.lat_rad) / 2);
  const double h = (from.height_m + to.height_m) / 2;
  return {{(to.lat_rad - from.lat_rad) * (r[0] + h),
           wrapAngle(to.lon_rad - from.lon_rad) * (r[1] + h) *
               std::cos((from.lat_rad + to.lat_rad) / 2),
           from.height_m - to.height_m}};
}
inline void offsetPosition(NominalState &s, const Vec3 &d) {
  const auto r = earth::radii(s.lat_rad);
  s.lat_rad += d[0] / (r[0] + s.height_m);
  s.lon_rad =
      wrapAngle(s.lon_rad + d[1] / ((r[1] + s.height_m) * std::cos(s.lat_rad)));
  s.height_m -= d[2];
}
} // namespace nav

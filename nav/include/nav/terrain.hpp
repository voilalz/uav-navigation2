#pragma once
#include "nav/math.hpp"
#include <limits>
namespace nav {
struct DemView {
  const double *heights{};
  std::size_t rows{}, cols{};
  double origin_north_m{}, origin_east_m{}, spacing_m{1};
  bool sample(double north, double east, double &height,
              Vec<2> *gradient = nullptr) const {
    if (!heights || rows < 2 || cols < 2 || rows > std::numeric_limits<std::size_t>::max() / cols ||
        !(spacing_m > 0) || !std::isfinite(spacing_m) || !std::isfinite(origin_north_m) ||
        !std::isfinite(origin_east_m) || !std::isfinite(north) || !std::isfinite(east))
      return false;
    const double y = (north - origin_north_m) / spacing_m,
                 x = (east - origin_east_m) / spacing_m;
    if (!std::isfinite(y) || !std::isfinite(x) || y < 0 || x < 0 || y > rows - 1 || x > cols - 1)
      return false;
    const auto r = std::min(std::size_t(y), rows - 2),
               c = std::min(std::size_t(x), cols - 2);
    const double u = y - double(r), v = x - double(c);
    const double a = heights[r * cols + c], b = heights[r * cols + c + 1],
                 d = heights[(r + 1) * cols + c],
                 e = heights[(r + 1) * cols + c + 1];
    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(d) ||
        !std::isfinite(e))
      return false;
    height = (1 - u) * ((1 - v) * a + v * b) + u * ((1 - v) * d + v * e);
    if (gradient)
      *gradient = {{((1 - v) * (d - a) + v * (e - b)) / spacing_m,
                    ((1 - u) * (b - a) + u * (e - d)) / spacing_m}};
    return std::isfinite(height) && (!gradient || finite(*gradient));
  }
};
struct TerrainPoint {
  double north_m{}, east_m{}, height_m{};
};
struct TercomResult {
  bool valid{};
  double delta_north_m{}, delta_east_m{}, cost{}, ambiguity_ratio{}, sigma_m{};
};
template <std::size_t MaxPoints = 256> class Tercom {
  std::array<TerrainPoint, MaxPoints> points_{};
  std::size_t size_{};
  bool costAt(const DemView &dem, double dn, double de, double &cost) const {
    double sum = 0, sum2 = 0;
    for (std::size_t i = 0; i < size_; ++i) {
      double h;
      if (!dem.sample(points_[i].north_m + dn, points_[i].east_m + de, h))
        return false;
      const double v = points_[i].height_m - h;
      sum += v;
      sum2 += v * v;
    }
    cost = std::max(0.0, sum2 / size_ - (sum / size_) * (sum / size_));
    return true;
  }

public:
  bool add(const TerrainPoint &p) {
    if (size_ == MaxPoints || !std::isfinite(p.north_m) ||
        !std::isfinite(p.east_m) || !std::isfinite(p.height_m))
      return false;
    points_[size_++] = p;
    return true;
  }
  void clear() { size_ = 0; }
  TercomResult match(const DemView &dem, double search_m, double step_m) const {
    TercomResult out;
    if (size_ < 8 || !std::isfinite(search_m) || !std::isfinite(step_m) ||
        search_m < step_m || step_m <= 0 || search_m / step_m > 100)
      return out;
    double mean = 0, var = 0;
    for (std::size_t i = 0; i < size_; ++i) {
      mean += points_[i].height_m;
      var += points_[i].height_m * points_[i].height_m;
    }
    if (var / size_ - std::pow(mean / size_, 2) < 4)
      return out;
    double best = 1e300, second = 1e300;
    const int steps = int(std::floor(search_m / step_m));
    for (int n = -steps; n <= steps; ++n)
      for (int e = -steps; e <= steps; ++e) {
        double cost;
        if (costAt(dem, n * step_m, e * step_m, cost) && cost < best) {
          best = cost;
          out.delta_north_m = n * step_m;
          out.delta_east_m = e * step_m;
        }
      }
    if (best == 1e300)
      return out;
    for (int n = -steps; n <= steps; ++n)
      for (int e = -steps; e <= steps; ++e) {
        if (std::hypot(n * step_m - out.delta_north_m,
                       e * step_m - out.delta_east_m) < 2 * step_m)
          continue;
        double cost;
        if (costAt(dem, n * step_m, e * step_m, cost))
          second = std::min(second, cost);
      }
    out.cost = best;
    out.ambiguity_ratio = (second + 1e-6) / (best + 1e-6);
    out.sigma_m = std::max(5.0, step_m);
    out.valid = second < 1e300 && out.ambiguity_ratio > 1.5 && best < 100;
    return out;
  }
};
} // namespace nav

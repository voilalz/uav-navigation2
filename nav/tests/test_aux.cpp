#include "nav/imu_array.hpp"
#include "nav/spsc_queue.hpp"
#include "nav/terrain.hpp"
#include <cstdio>
#include <limits>
using namespace nav;
int main() {
  int fail = 0;
  SpscQueue<int, 2> q;
  int out = 0;
  if (!q.push(1) || !q.push(2) || q.push(3) || q.dropped() != 1 ||
      !q.pop(out) || out != 1 || !q.push(4) || !q.pop(out) || out != 2 ||
      !q.pop(out) || out != 4 || q.pop(out))
    ++fail;
  ImuArray<4> a;
  std::array<ImuSample, 4> raw{};
  for (auto &r : raw) {
    r.t_us = 1000;
    r.accel_m_s2 = {{0, 0, -9.8}};
  }
  raw[3].gyro_rad_s[0] = 20;
  ImuSample fused;
  if (!a.fuse(raw, fused) || a.rejected_mask != 8 ||
      std::abs(fused.gyro_rad_s[0]) > 1e-10)
    ++fail;
  ImuDecimator<5> d;
  ImuSample decimated;
  for (int i = 0; i < 5; ++i) {
    fused.t_us = 1000 + 500 * i;
    if (d.add(fused, decimated) != (i == 4))
      ++fail;
  }
  if (decimated.t_us != 3000)
    ++fail;
  constexpr std::size_t side = 161;
  std::array<double, side * side> heights{};
  for (std::size_t n = 0; n < side; ++n)
    for (std::size_t e = 0; e < side; ++e) {
      const double x = double(n) * 10, y = double(e) * 10;
      heights[n * side + e] = 30 * std::sin(x / 127) + 20 * std::cos(y / 83) +
                              13 * std::sin((x + y) / 59);
    }
  DemView dem{heights.data(), side, side, 0, 0, 10};
  double height;
  Vec<2> gradient;
  if (!dem.sample(20, 30, height, &gradient) || dem.sample(-1, 30, height))
    ++fail;
  Tercom<64> ter;
  for (int i = 0; i < 40; ++i) {
    const double n = 400 + i * 18, e = 500 + i * 4;
    dem.sample(n + 120, e - 90, height);
    ter.add({n, e, height + 25});
  }
  auto fix = ter.match(dem, 200, 10);
  if (!fix.valid || std::abs(fix.delta_north_m - 120) > 10 ||
      std::abs(fix.delta_east_m + 90) > 10)
    ++fail;
  Tercom<32> flat;
  for (int i = 0; i < 20; ++i)
    flat.add({400. + i * 10, 500, 25});
  if (flat.match(dem, 200, 10).valid)
    ++fail;
  auto bad_dem = dem;
  bad_dem.spacing_m = std::numeric_limits<double>::infinity();
  if (bad_dem.sample(20, 30, height))
    ++fail;
  std::printf("aux: %d failures, TERCOM shift %.1f %.1f ratio %.2f\n", fail,
              fix.delta_north_m, fix.delta_east_m, fix.ambiguity_ratio);
  return fail ? 1 : 0;
}

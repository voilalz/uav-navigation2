#include "companion/companion.hpp"
#include "firmware/runtime.hpp"
#include "nav/earth.hpp"
#include <cstdio>
#include <random>
int main() {
  nav::NominalState origin;
  origin.lat_rad = 31 * nav::deg;
  origin.lon_rad = 121 * nav::deg;
  origin.height_m = 1000;
  auto initial = origin;
  nav::offsetPosition(initial, {{20, -15, 0}});
  firmware::Runtime runtime;
  runtime.initialize(initial);
  for (unsigned i = 1; i <= 400; ++i) {
    nav::ImuSample imu;
    imu.t_us = std::uint64_t(i) * 2500;
    imu.gyro_rad_s = nav::earth::rotation(initial.lat_rad);
    imu.accel_m_s2 = {
        {0, 0, -nav::earth::gravity(initial.lat_rad, initial.height_m)}};
    if (!runtime.enqueueImu(imu) || !runtime.runNavOnce())
      return 1;
    firmware::TxFrame tx;
    while (runtime.popTx(tx)) {
    }
  }
  // 合成的地图/实时图是真实输入给 CPU NCC 算法；没有手写位置量测替代 matcher。
  companion::ImageMap reference;
  reference.image.width = reference.image.height = 96;
  reference.image.pixels.resize(96 * 96);
  reference.origin_north_m = 47.5;
  reference.origin_east_m = -47.5;
  reference.gsd_m = 1;
  std::mt19937 random(42);
  for (auto &x : reference.image.pixels)
    x = float(random() % 256);
  companion::OrthoImage query;
  query.image.width = query.image.height = 32;
  query.image.pixels.resize(32 * 32);
  query.mask.assign(32 * 32, 1);
  query.center_north_m = 20;
  query.center_east_m = -15;
  query.camera_height_m = 1000;
  query.gsd_m = 1;
  query.t_us = 750000;
  for (int y = 0; y < 32; ++y)
    for (int x = 0; x < 32; ++x)
      query.image.at(x, y) = reference.image.at(x + 32, y + 32);
  const auto match = companion::matchAbsolute(query, reference, {20, -15, 80});
  if (!match.valid) {
    std::printf("matcher rejected: %s\n", companion::reasonName(match.reason));
    return 2;
  }
  auto fix = origin;
  nav::offsetPosition(fix, {{match.north_m, match.east_m, 0}});
  nav_wire_position_fix_t wire{};
  wire.t_us = match.t_us;
  wire.lat_deg = fix.lat_rad / nav::deg;
  wire.lon_deg = fix.lon_rad / nav::deg;
  wire.height_m = fix.height_m;
  wire.source = NAV_WIRE_SOURCE_VISION;
  wire.dimensions = match.dimensions;
  for (unsigned i = 0; i < 9; ++i)
    wire.covariance[i] = match.covariance[i];
  std::uint8_t payload[NAV_PROTOCOL_MAX_PAYLOAD], bytes[NAV_PROTOCOL_MAX_FRAME];
  std::size_t n = 0, k = 0;
  if (nav_encode_position_fix(&wire, payload, sizeof(payload), &n) !=
          NAV_PROTOCOL_OK ||
      nav_frame_encode(NAV_MSG_POSITION_FIX, 1, payload, n, bytes,
                       sizeof(bytes), &k) != NAV_PROTOCOL_OK)
    return 3;
  runtime.feedCompanionLink(bytes, k, 1000000);
  nav::ImuSample next;
  next.t_us = 1002500;
  next.gyro_rad_s = nav::earth::rotation(initial.lat_rad);
  next.accel_m_s2 = {{0, 0, -nav::earth::gravity(initial.lat_rad, initial.height_m)}};
  if (!runtime.enqueueImu(next))
    return 3;
  runtime.runNavOnce();
  const auto out = runtime.coreForNavTask().output();
  const auto error = nav::displacementNed(origin, out.state);
  std::printf("image -> NCC -> CRC POSITION_FIX -> MCU ESKF: shift %.3f %.3f "
              "m; residual %.3f m; mode %u\n",
              match.north_m, match.east_m, std::hypot(error[0], error[1]),
              unsigned(out.mode));
  return out.mode == nav::Mode::Vision && std::hypot(error[0], error[1]) < 1
             ? 0
             : 4;
}

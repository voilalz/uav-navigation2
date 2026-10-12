#include "nav/nav_core.hpp"
#include "nav/protocol.h"
#include <cstdio>
int main() {
  nav::NavCore core;
  nav::NominalState s;
  s.lat_rad = 31 * nav::deg;
  s.lon_rad = 121 * nav::deg;
  s.height_m = 1500;
  core.initialize(s);
  nav_parser_t parser;
  nav_parser_init(&parser);
  unsigned frames = 0;
  for (unsigned i = 1; i <= 4000; ++i) {
    nav::ImuSample imu;
    imu.t_us = std::uint64_t(i) * 2500;
    imu.gyro_rad_s = nav::earth::rotation(s.lat_rad);
    imu.accel_m_s2 = {{0, 0, -nav::earth::gravity(s.lat_rad, s.height_m)}};
    if (!core.onImu(imu))
      return 1;
    nav::NavOutput output;
    if (!core.takeOutput(output))
      continue;
    nav_wire_output_t wire{};
    wire.t_us = output.state.t_us;
    wire.lat_deg = output.state.lat_rad / nav::deg;
    wire.lon_deg = output.state.lon_rad / nav::deg;
    wire.height_m = output.state.height_m;
    wire.quaternion[0] = float(output.state.q_nb.w);
    wire.quaternion[1] = float(output.state.q_nb.x);
    wire.quaternion[2] = float(output.state.q_nb.y);
    wire.quaternion[3] = float(output.state.q_nb.z);
    for (unsigned j = 0; j < 3; ++j) {
      wire.vel_ned[j] = float(output.state.velocity_ned[j]);
      wire.angular_rate[j] = float(output.angular_rate_body[j]);
    }
    wire.h_sigma = float(output.horizontal_sigma_m);
    wire.v_sigma = float(output.vertical_sigma_m);
    wire.mode = std::uint8_t(output.mode);
    wire.health = output.health;
    std::uint8_t payload[NAV_PROTOCOL_MAX_PAYLOAD],
        buffer[NAV_PROTOCOL_MAX_FRAME];
    std::size_t payload_size = 0, frame_size = 0;
    if (nav_encode_nav_output(&wire, payload, sizeof(payload), &payload_size) !=
            NAV_PROTOCOL_OK ||
        nav_frame_encode(NAV_MSG_NAV_OUTPUT, std::uint16_t(frames), payload,
                         payload_size, buffer, sizeof(buffer),
                         &frame_size) != NAV_PROTOCOL_OK)
      return 2;
    for (std::size_t j = 0; j < frame_size; ++j) {
      nav_frame_t frame{};
      if (nav_parser_feed(&parser, buffer[j], &frame) == NAV_PARSE_FRAME) {
        nav_wire_output_t decoded{};
        if (nav_decode_nav_output(frame.payload, frame.payload_len, &decoded) !=
                NAV_PROTOCOL_OK ||
            decoded.t_us != wire.t_us)
          return 3;
        ++frames;
      }
    }
  }
  std::printf("MCU host: %u NAV_OUTPUT frames / 10 s; CRC errors %u\n", frames,
              parser.crc_errors);
  return frames == 300 && parser.crc_errors == 0 ? 0 : 4;
}

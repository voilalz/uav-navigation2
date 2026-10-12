#include "nav/nav_core.hpp"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace nav;
static bool parse(const std::string &s, double &v) {
  char *end = nullptr;
  v = std::strtod(s.c_str(), &end);
  return end != s.c_str() && !*end && std::isfinite(v);
}
int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: nav_replay events.csv replayed.csv\n";
    return 2;
  }
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) {
    std::cerr << "cannot open replay files\n";
    return 3;
  }
  output << std::setprecision(17)
         << "t_us,lat_rad,lon_rad,height_m,vn,ve,vd,qw,qx,qy,qz,h_sigma,v_"
            "sigma,mode,health\n";
  NavCore core;
  bool initialized = false, have_arrival = false;
  Timestamp last_arrival = 0;
  std::size_t row = 0, outputs = 0;
  std::string line;
  const auto flush = [&]() {
    NavOutput o;
    if (!core.takeOutput(o))
      return;
    ++outputs;
    output << o.state.t_us << ',' << o.state.lat_rad << ',' << o.state.lon_rad
           << ',' << o.state.height_m;
    for (double x : o.state.velocity_ned.v)
      output << ',' << x;
    output << ',' << o.state.q_nb.w << ',' << o.state.q_nb.x << ','
           << o.state.q_nb.y << ',' << o.state.q_nb.z << ','
           << o.horizontal_sigma_m << ',' << o.vertical_sigma_m << ','
           << unsigned(o.mode) << ',' << o.health << '\n';
  };
  const auto bad = [&]() {
    std::cerr << "invalid replay row " << row << '\n';
    return 4;
  };
  while (std::getline(input, line)) {
    ++row;
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.empty() || line[0] == '#')
      continue;
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string part;
    while (std::getline(stream, part, ','))
      fields.push_back(part);
    if (fields.empty())
      return bad();
    std::vector<double> value(fields.size());
    for (std::size_t i = 1; i < fields.size(); ++i)
      if (!parse(fields[i], value[i]))
        return bad();
    if (fields[0] == "S") {
      if (initialized || fields.size() != 12 || value[1] < 0 || value[1] > 9e15 ||
          std::floor(value[1]) != value[1])
        return bad();
      NominalState s;
      s.t_us = Timestamp(value[1]);
      s.lat_rad = value[2];
      s.lon_rad = value[3];
      s.height_m = value[4];
      s.velocity_ned = {{value[5], value[6], value[7]}};
      s.q_nb = {value[8], value[9], value[10], value[11]};
      if (!validState(s))
        return bad();
      core.initialize(s);
      initialized = true;
      continue;
    }
    if (!initialized || fields.size() < 3 || value[1] < 0 || value[2] < 0 ||
        value[1] > 9e15 || value[2] > 9e15 ||
        std::floor(value[1]) != value[1] || std::floor(value[2]) != value[2])
      return bad();
    const Timestamp arrival = Timestamp(value[1]), time = Timestamp(value[2]);
    if (have_arrival && arrival < last_arrival)
      return bad();
    if (have_arrival && arrival > last_arrival)
      flush();
    last_arrival = arrival;
    have_arrival = true;
    if (fields[0] == "I") {
      if (fields.size() != 9 || time != arrival)
        return bad();
      ImuSample imu;
      imu.t_us = time;
      imu.gyro_rad_s = {{value[3], value[4], value[5]}};
      imu.accel_m_s2 = {{value[6], value[7], value[8]}};
      if (!core.onImu(imu))
        return bad();
    } else if (fields[0] == "M") {
      if (fields.size() != 21 || value[3] < 0 || value[3] > 6 || value[4] < 0 ||
          value[4] > 4 || value[5] < 1 || value[5] > 3 ||
          std::floor(value[3]) != value[3] ||
          std::floor(value[4]) != value[4] || std::floor(value[5]) != value[5])
        return bad();
      Measurement m;
      m.t_us = time;
      m.kind = MeasurementKind(unsigned(value[3]));
      m.source = FixSource(unsigned(value[4]));
      m.dimensions = std::uint8_t(value[5]);
      m.value = {{value[6], value[7], value[8]}};
      for (unsigned i = 0; i < 9; ++i)
        m.covariance.v[i] = value[9 + i];
      m.terrain_height_m = value[18];
      m.terrain_gradient = {{value[19], value[20]}};
      core.onMeasurement(m);
    } else
      return bad();
  }
  flush();
  if (!initialized || !input.eof() || !output || outputs == 0)
    return 5;
  const auto stats = core.statistics();
  std::cout << "replay: " << outputs << " outputs, "
            << stats.measurements_accepted << " accepted, "
            << stats.measurements_rejected << " rejected\n";
  return 0;
}

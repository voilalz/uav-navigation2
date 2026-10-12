#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace firmware {
enum class DecodeStatus : uint8_t { Ok, Length, Checksum, Diagnostic, Invalid, Duplicate, Bus, NotReady };
struct ImuReading {
  uint64_t t_us{};
  std::array<double,3> gyro_rad_s{}, accel_m_s2{};
  double temperature_c{};
  uint16_t diagnostic{}, sample_counter{};
};
struct MagReading { uint64_t t_us{}; std::array<int32_t,3> counts{}; std::array<double,3> field_uT{}; };
struct GnssReading {
  uint64_t t_us{}; uint32_t tow_ms{};
  double lat_rad{},lon_rad{},height_ellipsoid_m{},height_msl_m{};
  std::array<double,3> velocity_ned{};
  double horizontal_accuracy_m{},vertical_accuracy_m{},speed_accuracy_m_s{},pdop{};
  uint16_t year{}; uint8_t month{},day{},hour{},minute{},second{},satellites{},fix_type{},flags{},utc_valid{};
  bool valid{};
};
// ADIS 数据为 SPI MSB 优先的20字节响应，不含命令周期的空返回。
DecodeStatus decodeAdis16470(const uint8_t*,std::size_t,uint64_t,ImuReading&);
DecodeStatus decodeRm3100(const uint8_t*,std::size_t,uint64_t,const std::array<double,3>&,MagReading&);
// F9P旧版手册将offset78列为reserved；只有核实协议支持flags3时才启用该校验。
DecodeStatus decodeUbxNavPvt(const uint8_t*,std::size_t,uint64_t,GnssReading&,bool has_flags3=false);
class UbxParser {
 public:
  explicit UbxParser(bool has_flags3=false):has_flags3_(has_flags3){}
  bool feed(uint8_t byte,uint64_t receive_t_us,GnssReading&);
  uint32_t checksumErrors() const {return checksum_errors_;}
  uint32_t discardedFrames() const {return discarded_;}
 private:
  std::array<uint8_t,264> buffer_{}; std::size_t size_{},expected_{};
  uint32_t checksum_errors_{},discarded_{};bool has_flags3_{};
};
class SpiBus {
 public:
  virtual ~SpiBus() = default;
  // 一次 transfer 在完整字节串期间保持CS；BSP负责DMA缓存一致性。
  virtual bool transfer(uint8_t device,uint8_t mode,uint32_t max_hz,const uint8_t* tx,uint8_t* rx,std::size_t size)=0;
  virtual void delayUs(uint32_t us)=0;
};
struct DriverStats { uint32_t reads{},bad_checksum{},diagnostic{},bus_errors{},duplicates{},lost_samples{}; };
class Adis16470 {
 public:
  Adis16470(SpiBus& bus,uint8_t device):bus_(bus),device_(device){}
  bool configureDirectSync();
  DecodeStatus read(uint64_t t_us,ImuReading&);
  // DMA完成路径同样可以调用此入口；只支持DATA_CNTR模式，不支持scaled-sync TIME_STMP。
  DecodeStatus acceptBurst(const uint8_t*,std::size_t,uint64_t,ImuReading&);
  const DriverStats& stats() const {return stats_;}
 private:
  bool readRegister(uint8_t address,uint16_t&);
  bool writeRegister(uint8_t address,uint16_t);
  SpiBus& bus_; uint8_t device_; bool have_counter_{}; uint16_t last_counter_{}; DriverStats stats_{};
};
class Rm3100 {
 public:
  Rm3100(SpiBus& bus,uint8_t device):bus_(bus),device_(device){}
  bool configure(uint16_t cycle_count=200,uint8_t rate_register=0x98);
  DecodeStatus read(uint64_t t_us,MagReading&);
 private:
  SpiBus& bus_; uint8_t device_; std::array<double,3> gain_{{75,75,75}};
};
} // namespace firmware

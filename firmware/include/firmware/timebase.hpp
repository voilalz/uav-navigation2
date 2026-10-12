#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace firmware {
// 单一时钟拥有者；必须在半个32位周期内采样。过时ISR捕获使用near()扩展。
class Counter32 {
 public:
  bool sample(uint32_t ticks,uint64_t& extended);
  bool near(uint32_t captured,uint64_t& extended) const;
 private: bool initialized_{}; uint32_t last_{}; uint64_t extended_{};
};
struct PpsStatus {bool locked{}; uint64_t last_t_us{}; double frequency_error_ppm{}; uint32_t rejected{},edges{};};
class PpsMonitor {
 public:
  bool edge(uint64_t t_us);
  PpsStatus status(uint64_t now_us) const;
 private: PpsStatus status_{}; uint8_t consecutive_{};
};
struct ExposureStamp {uint32_t frame_id{}; uint64_t trigger_us{},mid_us{}; uint32_t exposure_us{};};
class ExposureRegistry {
 public:
  bool trigger(uint32_t frame_id,uint64_t trigger_us,uint32_t exposure_us,uint32_t trigger_to_exposure_us=0);
  bool consume(uint32_t frame_id,uint64_t now_us,ExposureStamp&);
  uint32_t lost() const {return lost_;}
  uint32_t rejected() const {return rejected_;}
 private:
  struct Entry {ExposureStamp stamp{};bool live{};};
  std::array<Entry,64> entries_{}; std::size_t next_{}; uint32_t lost_{},rejected_{};
  bool have_last_{}; uint32_t last_frame_{}; uint64_t last_t_{};
};
} // namespace firmware

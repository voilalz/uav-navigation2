#pragma once
#include "firmware/fixed_queue.hpp"
#include "firmware/timebase.hpp"
#include "firmware/sensors.hpp"
#include "nav/nav_core.hpp"
#include "nav/protocol.h"
#include <array>
namespace firmware {
enum class Destination:uint8_t {FlightController=1,Companion=2,Both=3};
struct TxFrame {Destination destination{Destination::Both};uint16_t length{};std::array<uint8_t,NAV_PROTOCOL_MAX_FRAME> bytes{};};
struct RuntimeStats {uint32_t link_invalid{},link_time_invalid{},link_duplicate{},tx_invalid{},sensor_time_invalid{};};
class Runtime {
 public:
  explicit Runtime(const nav::NavCoreConfig& config={}):core_(config){nav_parser_init(&parser_);}
  // initialize仅在调度器启动前调用；NavTask是core_和输出序列号的唯一写者。
  void initialize(const nav::NominalState& state);
  bool enqueueImu(const nav::ImuSample&);
  bool enqueueSensorMeasurement(const nav::Measurement&);
  bool enqueueCamera(const ExposureStamp&);
  bool enqueuePps(uint64_t t_us);
  void feedCompanionLink(const uint8_t*,std::size_t,uint64_t mcu_now_us);
  // 仅LinkTask/NavTask全部暂停后调用；保留惯导/滤波状态，接入新的companion发送会话。
  void resetCompanionSessionQuiescent();
  bool runNavOnce();
  bool popTx(TxFrame& out){return tx_.pop(out);}
  RuntimeStats stats() const {return {link_invalid_.load(),link_time_invalid_.load(),link_duplicate_.load(),tx_invalid_.load(),sensor_time_invalid_.load()};}
  const nav::NavCore& coreForNavTask() const {return core_;}
  uint32_t queueOverflows() const;
 private:
  bool send(uint8_t id,const uint8_t*,std::size_t,Destination);
  void measurements(FixedQueue<nav::Measurement,64>&,std::array<nav::Measurement,64>&,std::size_t&);
  void publish(const nav::NavOutput&);
  nav::NavCore core_;nav_parser_t parser_{};
  std::atomic<uint32_t> link_invalid_{0},link_time_invalid_{0},link_duplicate_{0},tx_invalid_{0},sensor_time_invalid_{0};
  FixedQueue<nav::ImuSample,256> imu_;FixedQueue<nav::Measurement,64> sensor_,link_;
  FixedQueue<ExposureStamp,64> camera_;FixedQueue<uint64_t,8> pps_;FixedQueue<TxFrame,64> tx_;
  std::array<nav::Measurement,64> sensor_pending_{},link_pending_{};std::size_t sensor_count_{},link_count_{};bool have_seq_{};
  uint16_t tx_seq_{},rx_seq_{};uint64_t last_prior_us_{},last_link_byte_us_{};bool have_link_byte_{};PpsMonitor pps_monitor_{};
};
// 空速/磁航向/气压/雷达协议由具体设备定义；BSP仅可提交带明确SI和R的量测。
bool gnssMeasurements(const GnssReading&,nav::Measurement& position,nav::Measurement& velocity);
}

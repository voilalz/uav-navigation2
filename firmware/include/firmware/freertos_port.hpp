#pragma once
#include "firmware/imu_pipeline.hpp"
namespace firmware {
// BSP负责真实外设、中断队列、DMA缓存、初始对准和地址配置。回调均不得阻塞。
struct BoardPort {
  void* context{};
  bool (*initializeSensors)(void*){};
  bool (*initialState)(void*,nav::NominalState&){};
  uint64_t (*monotonicUs)(void*){};
  bool (*readImuGroup)(void*,std::array<nav::ImuSample,4>&){};
  std::size_t (*pollMeasurements)(void*,nav::Measurement*,std::size_t){};
  std::size_t (*receiveCompanion)(void*,uint8_t*,std::size_t){};
  bool (*transmit)(void*,Destination,const uint8_t*,std::size_t){};
  void (*serviceWatchdog)(void*,bool healthy){};
  void (*reportHealth)(void*,uint32_t overflows,RuntimeStats link_stats,uint32_t tx_errors){};
  void (*configureCalibration)(void*,nav::ImuArray<4>&){};
};
// 在CubeMX完成时钟/缓存/外设初始化后，调度器启动前调用；不得二次启动。
bool startFreeRtosPort(const BoardPort&);
// 相机和PPS各有唯一ISR生产者；t_us必须来自同一个MCU时钟。
bool cameraCaptureFromIsr(const ExposureStamp&);
bool ppsCaptureFromIsr(uint64_t);
}

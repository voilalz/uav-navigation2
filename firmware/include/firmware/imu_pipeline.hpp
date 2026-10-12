#pragma once
#include "firmware/runtime.hpp"
#include "nav/imu_array.hpp"
namespace firmware {
// 在ImuTask而非ISR内做阵列标定/野值排除和5点平均；DMA ISR只投递完整采样组。
class ImuPipeline {
 public:
  nav::ImuArray<4> array{};
  bool add(const std::array<nav::ImuSample,4>& samples,Runtime& runtime){
    nav::ImuSample fused,decimated;
    if(!array.fuse(samples,fused))return false;
    if(!decimator_.add(fused,decimated))return true;
    return runtime.enqueueImu(decimated);
  }
 private:nav::ImuDecimator<5> decimator_{};
};
}

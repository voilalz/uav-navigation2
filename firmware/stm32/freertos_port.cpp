#include "firmware/freertos_port.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <atomic>
#ifndef NAV_NAVTASK_STACK_WORDS
#define NAV_NAVTASK_STACK_WORDS 16384
#endif

namespace firmware {
namespace {
static_assert(configSUPPORT_STATIC_ALLOCATION==1,"必须启用FreeRTOS静态任务");
static_assert(configTICK_RATE_HZ>=1000,"轮询任务需要至少1kHz RTOS tick");
static_assert(configMAX_PRIORITIES>=6,"任务优先级需要至少6级");
Runtime runtime;
ImuPipeline pipeline;
BoardPort board;
std::atomic<bool> started{false};
std::atomic<uint32_t> nav_beats{0},link_beats{0},sensor_beats{0},tx_errors{0};
StaticTask_t nav_tcb,imu_tcb,link_tcb,sensor_tcb,health_tcb;
StackType_t nav_stack[NAV_NAVTASK_STACK_WORDS],imu_stack[1024],link_stack[1024],sensor_stack[768],health_stack[512];
void navTask(void*){
  for(;;){while(runtime.runNavOnce())++nav_beats;vTaskDelay(1);}
}
void imuTask(void*){
  std::array<nav::ImuSample,4> group;
  for(;;){for(unsigned n=0;n<32 && board.readImuGroup(board.context,group);++n)pipeline.add(group,runtime);vTaskDelay(1);}
}
void linkTask(void*){
  uint8_t bytes[NAV_PROTOCOL_MAX_FRAME];TxFrame pending;bool have_pending=false;
  for(;;){
    for(unsigned k=0;k<32;++k){const auto n=board.receiveCompanion(board.context,bytes,sizeof(bytes));if(!n)break;if(n>sizeof(bytes)){++tx_errors;break;}runtime.feedCompanionLink(bytes,n,board.monotonicUs(board.context));}
    for(unsigned k=0;k<32;++k){
      if(!have_pending)have_pending=runtime.popTx(pending);
      if(!have_pending)break;
      if(!board.transmit(board.context,pending.destination,pending.bytes.data(),pending.length)){++tx_errors;break;}
      have_pending=false;
    }
    ++link_beats;vTaskDelay(1);
  }
}
void sensorTask(void*){
  nav::Measurement samples[8];TickType_t wake=xTaskGetTickCount();
  for(;;){
    const auto count=board.pollMeasurements(board.context,samples,8);
    for(std::size_t n=0;n<count && n<8;++n)runtime.enqueueSensorMeasurement(samples[n]);
    ++sensor_beats;vTaskDelayUntil(&wake,pdMS_TO_TICKS(20));
  }
}
void healthTask(void*){
  uint32_t nav_before=0,link_before=0,sensor_before=0;
  for(;;){
    vTaskDelay(pdMS_TO_TICKS(50));const auto nv=nav_beats.load(),lk=link_beats.load(),sn=sensor_beats.load();
    // 100ms看门狗的喂狗入口只在所有任务继续推进时调用。
    board.serviceWatchdog(board.context,nv!=nav_before && lk!=link_before && sn!=sensor_before);
    if(board.reportHealth)board.reportHealth(board.context,runtime.queueOverflows(),runtime.stats(),tx_errors.load());
    nav_before=nv;link_before=lk;sensor_before=sn;
  }
}
}
bool startFreeRtosPort(const BoardPort& p){
  if(started || !p.initializeSensors || !p.initialState || !p.monotonicUs || !p.readImuGroup || !p.pollMeasurements || !p.receiveCompanion || !p.transmit || !p.serviceWatchdog)return false;
  board=p;nav::NominalState initial;
  if(!board.initializeSensors(board.context) || !board.initialState(board.context,initial) || !nav::validState(initial))return false;
  if(board.configureCalibration)board.configureCalibration(board.context,pipeline.array);
  runtime.initialize(initial);
  TaskHandle_t tasks[5]{
    xTaskCreateStatic(navTask,"Nav",NAV_NAVTASK_STACK_WORDS,nullptr,configMAX_PRIORITIES-1,nav_stack,&nav_tcb),
    xTaskCreateStatic(imuTask,"Imu",1024,nullptr,configMAX_PRIORITIES-2,imu_stack,&imu_tcb),
    xTaskCreateStatic(linkTask,"Link",1024,nullptr,3,link_stack,&link_tcb),
    xTaskCreateStatic(sensorTask,"Sensor",768,nullptr,2,sensor_stack,&sensor_tcb),
    xTaskCreateStatic(healthTask,"Health",512,nullptr,1,health_stack,&health_tcb)};
  for(auto task:tasks)if(!task){for(auto created:tasks)if(created)vTaskDelete(created);return false;}
  started=true;return true;
}
bool cameraCaptureFromIsr(const ExposureStamp& c){return started && runtime.enqueueCamera(c);}
bool ppsCaptureFromIsr(uint64_t t){return started && runtime.enqueuePps(t);}
}

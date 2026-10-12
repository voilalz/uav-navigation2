#include "firmware/timebase.hpp"
#include <limits>
namespace firmware {
bool Counter32::sample(uint32_t ticks,uint64_t& out){
  if(!initialized_){initialized_=true;last_=ticks;extended_=ticks;out=extended_;return true;}
  const uint32_t delta=ticks-last_;if(delta>=0x80000000)return false;
  extended_+=delta;last_=ticks;out=extended_;return true;
}
bool Counter32::near(uint32_t ticks,uint64_t& out) const{
  if(!initialized_)return false;
  const uint32_t delta=ticks-last_;
  if(delta<0x80000000){out=extended_+delta;return true;}
  const uint64_t back=4294967296ull-delta;if(back>extended_)return false;out=extended_-back;return true;
}
bool PpsMonitor::edge(uint64_t t){
  if(!status_.edges){status_.last_t_us=t;++status_.edges;consecutive_=1;return true;}
  if(t<=status_.last_t_us){++status_.rejected;status_.locked=false;consecutive_=0;return false;}
  const uint64_t dt=t-status_.last_t_us;
  if(dt<900000 || dt>1100000){++status_.rejected;status_.locked=false;consecutive_=0;if(dt>1100000)status_.last_t_us=t;return false;}
  const double ppm=(double(dt)-1000000);status_.frequency_error_ppm=consecutive_<2?ppm:.5*(status_.frequency_error_ppm+ppm);
  status_.last_t_us=t;++status_.edges;if(consecutive_<3)++consecutive_;status_.locked=consecutive_>=3;return true;
}
PpsStatus PpsMonitor::status(uint64_t t) const{
  auto v=status_;if(t<v.last_t_us || t-v.last_t_us>2500000)v.locked=false;return v;
}
bool ExposureRegistry::trigger(uint32_t id,uint64_t t,uint32_t exposure,uint32_t offset){
  if(exposure==0 || exposure>1000000 || offset>1000000 || t>std::numeric_limits<uint64_t>::max()-offset-exposure/2){++rejected_;return false;}
  if(have_last_){const uint32_t delta=id-last_frame_;if(delta==0 || delta>=0x80000000 || t<=last_t_){++rejected_;return false;}lost_+=delta-1;}
  auto& e=entries_[next_];if(e.live)++lost_;e.stamp={id,t,t+offset+exposure/2,exposure};e.live=true;next_=(next_+1)%entries_.size();
  have_last_=true;last_frame_=id;last_t_=t;return true;
}
bool ExposureRegistry::consume(uint32_t id,uint64_t now,ExposureStamp& out){
  for(auto& e:entries_)if(e.live && e.stamp.frame_id==id){
    if(now<e.stamp.mid_us){++rejected_;return false;}
    if(now-e.stamp.mid_us>2560000){e.live=false;++lost_;++rejected_;return false;}
    out=e.stamp;e.live=false;return true;
  }
  ++rejected_;return false;
}
}

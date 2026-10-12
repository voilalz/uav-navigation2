#include "firmware/sensors.hpp"
#include <cmath>
#include <cstring>
namespace firmware {
namespace {
constexpr double kPi=3.1415926535897932384626433832795;
uint16_t be16(const uint8_t* p){return uint16_t((uint16_t(p[0])<<8)|p[1]);}
int32_t signed16(const uint8_t* p){const uint16_t u=be16(p);return u<0x8000?int32_t(u):int32_t(u)-65536;}
uint32_t le32(const uint8_t* p){return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
int32_t sle32(const uint8_t* p){const uint32_t u=le32(p);return u<=0x7fffffff?int32_t(u):int32_t(int64_t(u)-4294967296ll);}
bool ubxChecksum(const uint8_t* p,std::size_t size){
  uint8_t a=0,b=0;for(std::size_t n=2;n<size-2;++n){a=uint8_t(a+p[n]);b=uint8_t(b+a);}
  return p[size-2]==a && p[size-1]==b;
}
}
DecodeStatus decodeAdis16470(const uint8_t* p,std::size_t size,uint64_t t,ImuReading& out){
  if(!p || size!=20)return DecodeStatus::Length;
  uint16_t sum=0;for(std::size_t n=0;n<18;++n)sum=uint16_t(sum+p[n]);
  if(be16(p+18)!=sum)return DecodeStatus::Checksum;
  ImuReading v;v.t_us=t;v.diagnostic=be16(p);v.sample_counter=be16(p+16);
  for(std::size_t n=0;n<3;++n){v.gyro_rad_s[n]=signed16(p+2+n*2)*(.1*kPi/180);v.accel_m_s2[n]=signed16(p+8+n*2)*(.00125*9.80665);}
  v.temperature_c=signed16(p+14)*.1;out=v;
  return v.diagnostic?DecodeStatus::Diagnostic:DecodeStatus::Ok;
}
DecodeStatus decodeRm3100(const uint8_t* p,std::size_t size,uint64_t t,const std::array<double,3>& gain,MagReading& out){
  if(!p || size!=9)return DecodeStatus::Length;
  MagReading v;v.t_us=t;
  for(std::size_t n=0;n<3;++n){
    if(!std::isfinite(gain[n]) || gain[n]<=0)return DecodeStatus::Invalid;
    const uint32_t u=(uint32_t(p[n*3])<<16)|(uint32_t(p[n*3+1])<<8)|p[n*3+2];
    v.counts[n]=(u&0x800000)?int32_t(u)-16777216:int32_t(u);v.field_uT[n]=v.counts[n]/gain[n];
  }
  out=v;return DecodeStatus::Ok;
}
DecodeStatus decodeUbxNavPvt(const uint8_t* frame,std::size_t size,uint64_t t,GnssReading& out,bool has_flags3){
  if(!frame || size!=100 || frame[4]!=92 || frame[5]!=0)return DecodeStatus::Length;
  if(frame[0]!=0xb5 || frame[1]!=0x62 || frame[2]!=1 || frame[3]!=7)return DecodeStatus::Invalid;
  if(!ubxChecksum(frame,size))return DecodeStatus::Checksum;
  const uint8_t* p=frame+6;GnssReading v;v.t_us=t;v.tow_ms=le32(p);
  v.year=uint16_t(p[4]|(uint16_t(p[5])<<8));v.month=p[6];v.day=p[7];v.hour=p[8];v.minute=p[9];v.second=p[10];v.utc_valid=p[11]&7;
  v.fix_type=p[20];v.flags=p[21];v.satellites=p[23];
  v.lon_rad=sle32(p+24)*1e-7*kPi/180;v.lat_rad=sle32(p+28)*1e-7*kPi/180;
  v.height_ellipsoid_m=sle32(p+32)*.001;v.height_msl_m=sle32(p+36)*.001;
  v.horizontal_accuracy_m=le32(p+40)*.001;v.vertical_accuracy_m=le32(p+44)*.001;
  for(std::size_t n=0;n<3;++n)v.velocity_ned[n]=sle32(p+48+n*4)*.001;
  v.speed_accuracy_m_s=le32(p+68)*.001;v.pdop=uint16_t(p[76]|(uint16_t(p[77])<<8))*.01;
  if(std::abs(v.lat_rad)>kPi/2 || std::abs(v.lon_rad)>kPi || v.tow_ms>=604800000)return DecodeStatus::Invalid;
  // invalidLlh 位属于 flags3；地理坐标无效时不允许入导航核。
  v.valid=(v.fix_type==3 || v.fix_type==4) && (v.flags&1) && (!has_flags3 || !(p[78]&1)) && v.horizontal_accuracy_m>0 && v.vertical_accuracy_m>0 && v.speed_accuracy_m_s>0;
  out=v;return DecodeStatus::Ok;
}
bool UbxParser::feed(uint8_t c,uint64_t t,GnssReading& out){
  if(size_==0){if(c==0xb5)buffer_[size_++]=c;return false;}
  if(size_==1){if(c==0x62)buffer_[size_++]=c;else if(c!=0xb5)size_=0;return false;}
  buffer_[size_++]=c;
  if(size_==6){
    expected_=8+std::size_t(buffer_[4])+(std::size_t(buffer_[5])<<8);
    if(expected_>buffer_.size()){++discarded_;size_=c==0xb5?1:0;buffer_[0]=c;expected_=0;return false;}
  }
  if(!expected_ || size_<expected_)return false;
  bool got=false;
  if(!ubxChecksum(buffer_.data(),size_))++checksum_errors_;
  else if(buffer_[2]==1 && buffer_[3]==7 && decodeUbxNavPvt(buffer_.data(),size_,t,out,has_flags3_)==DecodeStatus::Ok)got=true;
  else ++discarded_;
  if(got){size_=0;expected_=0;return true;}
  // 坏帧内仍可能含下一帧同步头，保留后缀继续解析。
  std::size_t prefix=size_;
  for(std::size_t n=2;n+1<size_;++n)if(buffer_[n]==0xb5 && buffer_[n+1]==0x62){prefix=n;break;}
  if(prefix<size_){size_-=prefix;std::memmove(buffer_.data(),buffer_.data()+prefix,size_);expected_=size_>=6?8+std::size_t(buffer_[4])+(std::size_t(buffer_[5])<<8):0;
    if(expected_>buffer_.size() || (expected_ && size_>=expected_)){size_=0;expected_=0;}
  }else{size_=c==0xb5?1:0;buffer_[0]=c;expected_=0;}
  return false;
}
bool Adis16470::readRegister(uint8_t address,uint16_t& value){
  uint8_t tx[]={address,0},rx[2]{};
  if(!bus_.transfer(device_,3,1000000,tx,rx,2))return false;
  bus_.delayUs(16);tx[0]=0;
  if(!bus_.transfer(device_,3,1000000,tx,rx,2))return false;
  bus_.delayUs(16);value=be16(rx);return true;
}
bool Adis16470::writeRegister(uint8_t address,uint16_t value){
  uint8_t rx[2]{},tx[]={uint8_t(address|0x80),uint8_t(value)};
  if(!bus_.transfer(device_,3,1000000,tx,rx,2))return false;
  bus_.delayUs(16);tx[0]=uint8_t((address+1)|0x80);tx[1]=uint8_t(value>>8);
  if(!bus_.transfer(device_,3,1000000,tx,rx,2))return false;
  bus_.delayUs(16);return true;
}
bool Adis16470::configureDirectSync(){
  uint16_t product=0,msc=0;
  if(!readRegister(0x72,product) || product!=16470 || !readRegister(0x60,msc))return false;
  const uint16_t configured=uint16_t((msc&~uint16_t(0x1c))|4);
  if(!writeRegister(0x60,configured) || !writeRegister(0x64,0))return false;
  uint16_t dec=1,verify=0;
  if(!readRegister(0x60,verify) || verify!=configured || !readRegister(0x64,dec) || dec!=0)return false;
  have_counter_=false;return true;
}
DecodeStatus Adis16470::read(uint64_t t,ImuReading& out){
  // 命令2字节+响应20字节，整个176位期间CS保持低电平。
  uint8_t tx[22]{0x68,0},rx[22]{};
  if(!bus_.transfer(device_,3,1000000,tx,rx,22)){++stats_.bus_errors;return DecodeStatus::Bus;}
  return acceptBurst(rx+2,20,t,out);
}
DecodeStatus Adis16470::acceptBurst(const uint8_t* p,std::size_t size,uint64_t t,ImuReading& out){
  ++stats_.reads;const auto result=decodeAdis16470(p,size,t,out);
  if(result==DecodeStatus::Checksum)++stats_.bad_checksum;
  if(result==DecodeStatus::Diagnostic)++stats_.diagnostic;
  if(result!=DecodeStatus::Ok)return result;
  if(have_counter_){const uint16_t delta=uint16_t(out.sample_counter-last_counter_);
    if(delta==0 || delta>=0x8000){++stats_.duplicates;return DecodeStatus::Duplicate;}
    stats_.lost_samples+=uint32_t(delta-1);
  }
  have_counter_=true;last_counter_=out.sample_counter;return DecodeStatus::Ok;
}
bool Rm3100::configure(uint16_t cycle,uint8_t rate){
  if(cycle<30 || cycle>400 || rate<0x92 || rate>0x9f)return false;
  uint8_t rx[7]{},tx[7]{0x04};for(int n=0;n<3;++n){tx[1+n*2]=uint8_t(cycle>>8);tx[2+n*2]=uint8_t(cycle);}
  if(!bus_.transfer(device_,0,1000000,tx,rx,7))return false;
  tx[0]=0x0b;tx[1]=rate;if(!bus_.transfer(device_,0,1000000,tx,rx,2))return false;
  tx[0]=1;tx[1]=0x71;if(!bus_.transfer(device_,0,1000000,tx,rx,2))return false;
  // PNI典型增益线性近似；精确增益和硬/软铁矩阵应由板级标定配置。
  gain_.fill(.37*cycle+1);return true;
}
DecodeStatus Rm3100::read(uint64_t t,MagReading& out){
  uint8_t tx[10]{0xb4},rx[10]{};
  if(!bus_.transfer(device_,0,1000000,tx,rx,2))return DecodeStatus::Bus;
  if(!(rx[1]&0x80))return DecodeStatus::NotReady;
  tx[0]=0xa4;if(!bus_.transfer(device_,0,1000000,tx,rx,10))return DecodeStatus::Bus;
  return decodeRm3100(rx+1,9,t,gain_,out);
}
}

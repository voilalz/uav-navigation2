#include "firmware/sensors.hpp"
#include "firmware/timebase.hpp"
#include "firmware/fixed_queue.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
static void require(bool ok,const char* why){if(!ok){std::fprintf(stderr,"FAIL: %s\n",why);std::exit(1);}}
static bool close(double a,double b){return std::abs(a-b)<1e-9;}
static std::array<uint8_t,20> burst(uint16_t counter=7,uint16_t diag=0){
  const uint16_t words[]={diag,100,uint16_t(-100),0,800,uint16_t(-800),1,250,counter,0};
  std::array<uint8_t,20> b{};uint16_t sum=0;
  for(int i=0;i<9;++i){b[i*2]=uint8_t(words[i]>>8);b[i*2+1]=uint8_t(words[i]);sum+=b[i*2]+b[i*2+1];}
  b[18]=uint8_t(sum>>8);b[19]=uint8_t(sum);return b;
}
static void put32(std::vector<uint8_t>& b,int at,uint32_t x){for(int i=0;i<4;++i)b[at+i]=uint8_t(x>>(i*8));}
static std::vector<uint8_t> pvt(){
  std::vector<uint8_t> b(100);b[0]=0xb5;b[1]=0x62;b[2]=1;b[3]=7;b[4]=92;
  b[6+20]=3;b[6+21]=1;b[6+23]=12;b[6+11]=7;
  put32(b,6,123456);put32(b,6+24,1234567890);put32(b,6+28,uint32_t(-345678901));
  put32(b,6+32,1250123);put32(b,6+36,1210123);put32(b,6+40,1500);put32(b,6+44,2500);
  put32(b,6+48,167000);put32(b,6+52,uint32_t(-1000));put32(b,6+56,500);put32(b,6+68,300);
  uint8_t a=0,c=0;for(int i=2;i<98;++i){a=uint8_t(a+b[i]);c=uint8_t(c+a);}b[98]=a;b[99]=c;return b;
}
class NoBus final:public firmware::SpiBus {
 public:bool transfer(uint8_t,uint8_t,uint32_t,const uint8_t*,uint8_t*,std::size_t)override{return false;}void delayUs(uint32_t)override{}
};
int main(){
  using namespace firmware; ImuReading i;auto b=burst();
  require(decodeAdis16470(b.data(),b.size(),500,i)==DecodeStatus::Ok,"valid ADIS burst accepted");
  require(i.t_us==500 && close(i.gyro_rad_s[0],10*3.14159265358979323846/180) && close(i.gyro_rad_s[1],-10*3.14159265358979323846/180),"signed gyroscope SI scale");
  require(close(i.accel_m_s2[0],9.80665) && close(i.accel_m_s2[1],-9.80665) && close(i.temperature_c,25),"ADIS acceleration and zero-reference temperature");
  b[4]^=1;require(decodeAdis16470(b.data(),20,500,i)==DecodeStatus::Checksum,"ADIS checksum failure");
  b=burst(8,2);require(decodeAdis16470(b.data(),20,500,i)==DecodeStatus::Diagnostic,"diagnostic rejects corrupt sensor");
  require(decodeAdis16470(b.data(),19,500,i)==DecodeStatus::Length,"short ADIS burst");
  NoBus bus;Adis16470 driver(bus,0);b=burst(65535);require(driver.acceptBurst(b.data(),20,1,i)==DecodeStatus::Ok,"initial counter");
  b=burst(0);require(driver.acceptBurst(b.data(),20,2,i)==DecodeStatus::Ok,"16bit counter wrap");
  require(driver.acceptBurst(b.data(),20,3,i)==DecodeStatus::Duplicate,"duplicate ADIS rejected");
  b=burst(3);require(driver.acceptBurst(b.data(),20,4,i)==DecodeStatus::Ok && driver.stats().lost_samples==2,"missing samples counted");
  require(driver.read(5,i)==DecodeStatus::Bus && driver.stats().bus_errors==1,"bus failure propagated");
  const uint8_t m[]={0x7f,0xff,0xff,0x80,0,0,0xff,0xff,0xff};MagReading mag;
  require(decodeRm3100(m,9,600,{{75,75,75}},mag)==DecodeStatus::Ok && mag.counts[0]==8388607 && mag.counts[1]==-8388608 && mag.counts[2]==-1,"RM3100 signed 24bit endpoints");
  require(close(mag.field_uT[2],-1.0/75) && decodeRm3100(m,9,600,{{0,75,75}},mag)==DecodeStatus::Invalid,"RM3100 calibrated gain validation");
  auto u=pvt();GnssReading gnss;require(decodeUbxNavPvt(u.data(),u.size(),700,gnss)==DecodeStatus::Ok,"UBX PVT valid");
  require(gnss.valid && gnss.tow_ms==123456 && close(gnss.height_ellipsoid_m,1250.123) && close(gnss.height_msl_m,1210.123),"PVT uses ellipsoid height explicitly");
  require(close(gnss.velocity_ned[0],167) && close(gnss.velocity_ned[1],-1) && close(gnss.speed_accuracy_m_s,.3),"PVT velocity scale/sign");
  UbxParser parser;bool got=false;for(uint8_t c:{0x00,0xb5,0xb5})parser.feed(c,700,gnss);for(uint8_t c:u)got|=parser.feed(c,700,gnss);require(got,"UBX noise and overlapping sync");
  u[30]^=1;got=false;for(auto c:u)got|=parser.feed(c,800,gnss);require(!got && parser.checksumErrors()==1,"UBX checksum corrupt");
  u=pvt();got=false;for(auto c:u)got|=parser.feed(c,900,gnss);require(got,"UBX recovery after bad frame");
  u[6+78]=1;{uint8_t a=0,c=0;for(int n=2;n<98;++n){a=uint8_t(a+u[n]);c=uint8_t(c+a);}u[98]=a;u[99]=c;}
  require(decodeUbxNavPvt(u.data(),100,900,gnss,false)==DecodeStatus::Ok && gnss.valid,"F9P legacy PVT reserved bits ignored");
  require(decodeUbxNavPvt(u.data(),100,900,gnss,true)==DecodeStatus::Ok && !gnss.valid,"modern PVT invalidLlh bit rejects fix");
  Counter32 clock;uint64_t t=0;require(clock.sample(0xfffffff0,t)&&t==0xfffffff0ull,"clock initial extension");
  require(clock.sample(0x20,t)&&t==0x100000020ull,"32bit microsecond wrap");
  require(clock.near(0xfffffff8,t)&&t==0xfffffff8ull,"timestamp older capture across wrap");
  require(!clock.sample(0x10,t),"clock rejects out of order sample");
  PpsMonitor pps;require(pps.edge(1000000)&&pps.edge(2000020)&&pps.edge(3000040),"PPS normal period");
  require(pps.status(3000100).locked && close(pps.status(3000100).frequency_error_ppm,20),"PPS clock drift estimate");
  require(!pps.status(5500041).locked && !pps.edge(3000041),"PPS expiry and implausible period");
  ExposureRegistry frames;ExposureStamp stamp;
  require(frames.trigger(42,1000000,400,50)&&frames.consume(42,1001000,stamp)&&stamp.mid_us==1000250,"exposure midpoint association");
  require(!frames.consume(42,1001100,stamp),"one trigger cannot timestamp repeated frame");
  require(frames.trigger(44,1100000,400)&&frames.lost()==1,"lost camera frame id count");
  require(!frames.trigger(43,1100100,400),"out of order trigger rejected");
  require(!frames.consume(44,4100001,stamp),"stale exposure rejected");
  FixedQueue<uint32_t,4> q;for(uint32_t k=1;k<=4;++k)require(q.push(k),"queue full capacity usable");
  require(!q.push(5)&&q.dropped()==1&&q.size()==4,"queue overrun count");
  uint32_t v=0;for(uint32_t k=1;k<=4;++k)require(q.pop(v)&&v==k,"queue FIFO");require(!q.pop(v),"empty queue");
  for(uint32_t k=0;k<100;++k){require(q.push(k)&&q.pop(v)&&v==k,"ring wrap");}
  std::puts("firmware sensor/time/queue tests passed");
}

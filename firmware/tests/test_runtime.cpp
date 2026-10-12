#include "firmware/sensors.hpp"
#include "firmware/runtime.hpp"
#include "nav/earth.hpp"
#include <cstdio>
#include <cstdlib>
static void require(bool ok,const char* why){if(!ok){std::fprintf(stderr,"FAIL: %s\n",why);std::exit(1);}}
static std::size_t vo(uint64_t t,uint16_t seq,uint8_t* out){nav_wire_vo_velocity_t v{};v.t_us=t;v.source=NAV_WIRE_SOURCE_VISION;v.dimensions=3;v.covariance[0]=v.covariance[4]=v.covariance[8]=1;uint8_t p[NAV_PAYLOAD_VO_VELOCITY];std::size_t n=0,size=0;require(nav_encode_vo_velocity(&v,p,sizeof(p),&n)==NAV_PROTOCOL_OK,"VO payload fixture");require(nav_frame_encode(NAV_MSG_VO_VELOCITY,seq,p,n,out,NAV_PROTOCOL_MAX_FRAME,&size)==NAV_PROTOCOL_OK,"VO frame fixture");return size;}
int main(){
  using namespace firmware;Runtime rt;nav::NominalState state;state.t_us=1000000;state.lat_rad=.6;state.lon_rad=1;state.height_m=100;rt.initialize(state);
  nav::ImuSample imu;imu.t_us=1002500;imu.gyro_rad_s=nav::earth::rotation(state.lat_rad);imu.accel_m_s2={{0,0,-nav::earth::gravity(state.lat_rad,state.height_m)}};
  require(rt.enqueueImu(imu),"runtime enqueues IMU");require(rt.runNavOnce()&&rt.coreForNavTask().state().t_us==imu.t_us,"NavTask advances core");
  uint8_t frame[NAV_PROTOCOL_MAX_FRAME];auto size=vo(1002500,1,frame);rt.feedCompanionLink(frame,size,1002600);
  imu.t_us+=2500;require(rt.enqueueImu(imu)&&rt.runNavOnce()&&rt.coreForNavTask().statistics().measurements_accepted==1,"link measurement routed to only core writer");
  rt.feedCompanionLink(frame,size,1005100);require(rt.stats().link_duplicate==1,"link duplicate sequence discarded");
  size=vo(1100000,2,frame);rt.feedCompanionLink(frame,size,1005200);require(rt.stats().link_time_invalid==1,"future exposure rejected");
  size=vo(1,3,frame);rt.feedCompanionLink(frame,size,3000002);require(rt.stats().link_time_invalid==2,"stale exposure rejected");
  require(rt.enqueueCamera({10,1000000,1000200,400})&&rt.enqueuePps(1000000),"IRQ events enqueued");
  for(int k=0;k<20;++k){imu.t_us+=2500;require(rt.enqueueImu(imu)&&rt.runNavOnce(),"runtime periodic advance");}
  TxFrame tx;bool output=false,camera=false,time=false,prior=false;nav_parser_t parser{};nav_parser_init(&parser);nav_frame_t decoded{};
  while(rt.popTx(tx)){for(std::size_t n=0;n<tx.length;++n)if(nav_parser_feed(&parser,tx.bytes[n],&decoded)==NAV_PARSE_FRAME){
    if(decoded.msg_id==NAV_MSG_NAV_OUTPUT){nav_wire_output_t o{};require(nav_decode_nav_output(decoded.payload,decoded.payload_len,&o)==NAV_PROTOCOL_OK,"runtime nav output serializes fields");output=true;require(o.lat_deg>34&&o.lat_deg<35,"runtime rad to degrees conversion");}
    camera|=decoded.msg_id==NAV_MSG_CAM_TRIGGER;time|=decoded.msg_id==NAV_MSG_TIME_SYNC;prior|=decoded.msg_id==NAV_MSG_NAV_PRIOR;
  }}require(output&&camera&&time&&prior,"all output paths emit valid protocol frames");
  GnssReading g;g.valid=true;g.t_us=1;g.lat_rad=.5;g.lon_rad=1;g.height_ellipsoid_m=123;g.horizontal_accuracy_m=2;g.vertical_accuracy_m=3;g.speed_accuracy_m_s=.2;
  nav::Measurement p,v;require(gnssMeasurements(g,p,v)&&p.source==nav::FixSource::Gnss&&p.value[2]==123&&p.covariance(0,0)==4&&v.covariance(0,0)>.039,"GNSS SI covariance mapping");
  g.valid=false;require(!gnssMeasurements(g,p,v),"invalid GNSS fix never fused");
  Runtime broken;broken.initialize(state);const uint8_t truncated[]={0xa5,0x5a,0x7f,0xff,0,0,5,6};
  broken.feedCompanionLink(truncated,sizeof(truncated),1000000);size=vo(1002500,1,frame);broken.feedCompanionLink(frame,size,1600001);
  imu.t_us=1002500;require(broken.enqueueImu(imu)&&broken.runNavOnce()&&broken.coreForNavTask().statistics().measurements_accepted==1,"transport timeout resets truncated outer frame");
  Runtime buffered;buffered.initialize(state);uint8_t wrapped[236]{0xa5,0x5a,0x7f,228,0,0};
  size=vo(1002500,1,wrapped+6);require(size==115,"buffered fixture first size");size=vo(1003000,2,wrapped+121);
  buffered.feedCompanionLink(wrapped,sizeof(wrapped),1005100);
  imu.t_us=1002500;require(buffered.enqueueImu(imu)&&buffered.runNavOnce(),"buffered first IMU");imu.t_us=1005000;
  require(buffered.enqueueImu(imu)&&buffered.runNavOnce()&&buffered.coreForNavTask().statistics().measurements_accepted==2,"CRC recovery drains multiple cached measurement frames");
  Runtime future_sensor;future_sensor.initialize(state);g.valid=true;g.t_us=1002500;require(gnssMeasurements(g,p,v),"sensor fixture");p.value={{state.lat_rad,state.lon_rad,state.height_m}};
  auto future=p;future.t_us=100000000;
  require(future_sensor.enqueueSensorMeasurement(future)&&future_sensor.enqueueSensorMeasurement(p),"future sensor queue fixture");imu.t_us=1002500;
  require(future_sensor.enqueueImu(imu)&&future_sensor.runNavOnce()&&future_sensor.coreForNavTask().statistics().measurements_accepted==1,"future sensor cannot block valid following measurement");
  Runtime reset;reset.initialize(state);imu.t_us=1005000;require(reset.enqueueImu(imu),"reinitialize fixture old IMU");size=vo(1002500,5,frame);reset.feedCompanionLink(frame,size,1005100);
  reset.initialize(state);require(!reset.runNavOnce(),"initialize clears old queued IMU");size=vo(1002500,1,frame);reset.feedCompanionLink(frame,size,1005100);imu.t_us=1002500;
  require(reset.enqueueImu(imu)&&reset.runNavOnce()&&reset.coreForNavTask().statistics().measurements_accepted==1,"initialize resets previous link sequence and queued measurements");
  reset.resetCompanionSessionQuiescent();size=vo(1002501,0,frame);reset.feedCompanionLink(frame,size,1005100);imu.t_us=1005000;
  require(reset.enqueueImu(imu)&&reset.runNavOnce()&&reset.coreForNavTask().statistics().measurements_accepted==2,"explicit companion reboot reset preserves filter and accepts fresh sequence");
  Runtime horizontal;auto tilted=state;tilted.q_nb=nav::Quaternion::fromEuler(.3,.5,.7);tilted.velocity_ned={{0,0,5}};horizontal.initialize(tilted);
  nav_wire_vo_velocity_t hvo{};hvo.t_us=1002500;hvo.dimensions=2;hvo.source=NAV_WIRE_SOURCE_VISION;hvo.frame=NAV_WIRE_VELOCITY_NED;hvo.covariance[0]=hvo.covariance[4]=1;
  uint8_t payload[NAV_PAYLOAD_VO_VELOCITY];std::size_t length=0;require(nav_encode_vo_velocity(&hvo,payload,sizeof(payload),&length)==NAV_PROTOCOL_OK,"NED2 codec fixture");
  require(nav_frame_encode(NAV_MSG_VO_VELOCITY,0,payload,length,frame,sizeof(frame),&size)==NAV_PROTOCOL_OK,"NED2 frame fixture");horizontal.feedCompanionLink(frame,size,1002600);
  imu.t_us=1002500;const auto body_from_ned=nav::transpose(tilted.q_nb.matrix());imu.gyro_rad_s=body_from_ned*nav::earth::rotation(tilted.lat_rad);imu.accel_m_s2=body_from_ned*nav::Vec3{{0,0,-nav::earth::gravity(tilted.lat_rad,tilted.height_m)}};
  require(horizontal.enqueueImu(imu)&&horizontal.runNavOnce()&&horizontal.coreForNavTask().statistics().measurements_accepted==1,"horizontal NED2 accepted at tilted attitude");
  require(std::abs(horizontal.coreForNavTask().state().velocity_ned[2]-5)<.01,"NED2 does not invent observed vertical velocity or rotate into body axes");
  std::puts("firmware runtime tests passed");
}

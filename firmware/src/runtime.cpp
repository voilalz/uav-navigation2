#include "firmware/runtime.hpp"
#include "firmware/sensors.hpp"
namespace firmware {
void Runtime::initialize(const nav::NominalState& state){
  imu_.resetQuiescent();sensor_.resetQuiescent();link_.resetQuiescent();camera_.resetQuiescent();pps_.resetQuiescent();tx_.resetQuiescent();
  sensor_count_=link_count_=0;have_seq_=have_link_byte_=false;rx_seq_=tx_seq_=0;last_prior_us_=last_link_byte_us_=0;pps_monitor_={};
  link_invalid_=0;link_time_invalid_=0;link_duplicate_=0;tx_invalid_=0;sensor_time_invalid_=0;nav_parser_init(&parser_);core_.initialize(state);
}
bool Runtime::enqueueImu(const nav::ImuSample& s){return imu_.push(s);}
void Runtime::resetCompanionSessionQuiescent(){
  nav_parser_reset_stream(&parser_);link_.resetQuiescent();link_count_=0;rx_seq_=0;have_seq_=have_link_byte_=false;last_link_byte_us_=0;
}
bool Runtime::enqueueSensorMeasurement(const nav::Measurement& m){return sensor_.push(m);}
bool Runtime::enqueueCamera(const ExposureStamp& c){return camera_.push(c);}
bool Runtime::enqueuePps(uint64_t t){return pps_.push(t);}
void Runtime::feedCompanionLink(const uint8_t* data,std::size_t size,uint64_t now){
  if(!data && size){++link_invalid_;return;}
  if(size){
    if(have_link_byte_ && (now<last_link_byte_us_ || now-last_link_byte_us_>500000))nav_parser_reset_stream(&parser_);
    last_link_byte_us_=now;have_link_byte_=true;
  }
  nav_frame_t frame{};
  const auto accept=[this,now](const nav_frame_t& frame){
    if(have_seq_){const uint16_t delta=uint16_t(frame.seq-rx_seq_);if(delta==0 || delta>=0x8000){++link_duplicate_;return;}}
    nav::Measurement m;bool valid=false;
    if(frame.msg_id==NAV_MSG_VO_VELOCITY){
      nav_wire_vo_velocity_t v{};
      if(nav_decode_vo_velocity(frame.payload,frame.payload_len,&v)==NAV_PROTOCOL_OK && v.source==NAV_WIRE_SOURCE_VISION){
        m.t_us=v.t_us;m.kind=v.frame==NAV_WIRE_VELOCITY_NED?nav::MeasurementKind::VelocityNed:nav::MeasurementKind::VelocityBody;m.source=nav::FixSource::Vision;m.dimensions=v.dimensions;
        for(std::size_t k=0;k<3;++k)m.value[k]=v.velocity_body[k];
        for(std::size_t k=0;k<9;++k)m.covariance.v[k]=v.covariance[k];
        valid=true;
      }
    }else if(frame.msg_id==NAV_MSG_POSITION_FIX){
      nav_wire_position_fix_t v{};
      if(nav_decode_position_fix(frame.payload,frame.payload_len,&v)==NAV_PROTOCOL_OK && v.source>=NAV_WIRE_SOURCE_VISION && v.source<=NAV_WIRE_SOURCE_TERCOM){
        m.t_us=v.t_us;m.kind=nav::MeasurementKind::Position;m.source=static_cast<nav::FixSource>(v.source);m.dimensions=v.dimensions;
        m.value={{v.lat_deg*nav::deg,v.lon_deg*nav::deg,v.height_m}};
        for(std::size_t k=0;k<9;++k)m.covariance.v[k]=v.covariance[k];
        valid=true;
      }
    }else return;
    if(!valid){++link_invalid_;return;}
    if(m.t_us>now || now-m.t_us>2000000){++link_time_invalid_;return;}
    if(link_.push(m)){rx_seq_=frame.seq;have_seq_=true;}
  };
  for(std::size_t n=0;n<size;++n){
    if(nav_parser_feed(&parser_,data[n],&frame)!=NAV_PARSE_FRAME)continue;
    do {accept(frame);}while(nav_parser_next(&parser_,&frame)==NAV_PARSE_FRAME);
  }
}
bool Runtime::send(uint8_t id,const uint8_t* payload,std::size_t size,Destination to){
  TxFrame frame;frame.destination=to;std::size_t length=0;
  if(nav_frame_encode(id,tx_seq_,payload,size,frame.bytes.data(),frame.bytes.size(),&length)!=NAV_PROTOCOL_OK){++tx_invalid_;return false;}
  frame.length=uint16_t(length);if(!tx_.push(frame))return false;++tx_seq_;return true;
}
void Runtime::measurements(FixedQueue<nav::Measurement,64>& q,std::array<nav::Measurement,64>& pending,std::size_t& count){
  while(count<pending.size() && q.pop(pending[count]))++count;
  // 有界等待表允许合法稍后时间等待下一IMU，同时不阻塞其后的到期量测。
  std::size_t remain=0;
  for(std::size_t n=0;n<count;++n){
    if(pending[n].t_us>core_.state().t_us){
      if(pending[n].t_us-core_.state().t_us>50000){++sensor_time_invalid_;continue;}
      pending[remain++]=pending[n];continue;
    }
    core_.onMeasurement(pending[n]);
  }
  count=remain;
}
bool Runtime::runNavOnce(){
  uint8_t payload[NAV_PROTOCOL_MAX_PAYLOAD];std::size_t size=0;uint64_t edge=0;
  while(pps_.pop(edge)){
    pps_monitor_.edge(edge);const auto ps=pps_monitor_.status(edge);nav_wire_time_sync_t t{edge,ps.edges,uint8_t(ps.locked)};
    if(nav_encode_time_sync(&t,payload,sizeof(payload),&size)==NAV_PROTOCOL_OK)send(NAV_MSG_TIME_SYNC,payload,size,Destination::Companion);
  }
  ExposureStamp camera;
  while(camera_.pop(camera)){
    nav_wire_cam_trigger_t c{camera.frame_id,camera.mid_us,camera.exposure_us,0};
    if(nav_encode_cam_trigger(&c,payload,sizeof(payload),&size)==NAV_PROTOCOL_OK)send(NAV_MSG_CAM_TRIGGER,payload,size,Destination::Companion);
  }
  nav::ImuSample sample;if(!imu_.pop(sample))return false;
  const bool accepted=core_.onImu(sample);
  measurements(sensor_,sensor_pending_,sensor_count_);measurements(link_,link_pending_,link_count_);
  nav::NavOutput output;if(core_.takeOutput(output))publish(output);return accepted;
}
uint32_t Runtime::queueOverflows() const{return imu_.dropped()+sensor_.dropped()+link_.dropped()+camera_.dropped()+pps_.dropped()+tx_.dropped();}
void Runtime::publish(const nav::NavOutput& o){
  nav_wire_output_t v{};v.t_us=o.state.t_us;v.lat_deg=o.state.lat_rad/nav::deg;v.lon_deg=o.state.lon_rad/nav::deg;v.height_m=o.state.height_m;
  for(std::size_t k=0;k<3;++k){v.vel_ned[k]=float(o.state.velocity_ned[k]);v.angular_rate[k]=float(o.angular_rate_body[k]);}
  v.quaternion[0]=float(o.state.q_nb.w);v.quaternion[1]=float(o.state.q_nb.x);v.quaternion[2]=float(o.state.q_nb.y);v.quaternion[3]=float(o.state.q_nb.z);
  v.h_sigma=float(o.horizontal_sigma_m);v.v_sigma=float(o.vertical_sigma_m);v.mode=uint8_t(o.mode);v.health=o.health|(queueOverflows()?uint32_t(nav::QueueOverflow):0u);
  uint8_t payload[NAV_PROTOCOL_MAX_PAYLOAD];std::size_t size=0;
  if(nav_encode_nav_output(&v,payload,sizeof(payload),&size)==NAV_PROTOCOL_OK)send(NAV_MSG_NAV_OUTPUT,payload,size,Destination::Both);else ++tx_invalid_;
  if(!last_prior_us_ || v.t_us-last_prior_us_>=100000){
    nav_wire_prior_t p{};p.t_us=v.t_us;p.lat_deg=v.lat_deg;p.lon_deg=v.lon_deg;p.height_m=v.height_m;p.h_sigma=v.h_sigma;p.v_sigma=v.v_sigma;p.mode=v.mode;
    for(std::size_t k=0;k<3;++k)p.vel_ned[k]=v.vel_ned[k];
    for(std::size_t k=0;k<4;++k)p.quaternion[k]=v.quaternion[k];
    if(nav_encode_nav_prior(&p,payload,sizeof(payload),&size)==NAV_PROTOCOL_OK && send(NAV_MSG_NAV_PRIOR,payload,size,Destination::Companion))last_prior_us_=v.t_us;
  }
}
bool gnssMeasurements(const GnssReading& g,nav::Measurement& position,nav::Measurement& velocity){
  if(!g.valid || !std::isfinite(g.lat_rad) || !std::isfinite(g.lon_rad) || !std::isfinite(g.height_ellipsoid_m) || std::abs(g.lat_rad)>=89.5*nav::deg || std::abs(g.lon_rad)>nav::pi)return false;
  for(const double sigma:{g.horizontal_accuracy_m,g.vertical_accuracy_m,g.speed_accuracy_m_s})if(!std::isfinite(sigma) || sigma<=0)return false;
  position={};velocity={};position.t_us=velocity.t_us=g.t_us;position.source=velocity.source=nav::FixSource::Gnss;
  position.kind=nav::MeasurementKind::Position;position.value={{g.lat_rad,g.lon_rad,g.height_ellipsoid_m}};
  velocity.kind=nav::MeasurementKind::VelocityNed;
  for(std::size_t k=0;k<3;++k){if(!std::isfinite(g.velocity_ned[k]))return false;velocity.value[k]=g.velocity_ned[k];position.covariance(k,k)=k==2?g.vertical_accuracy_m*g.vertical_accuracy_m:g.horizontal_accuracy_m*g.horizontal_accuracy_m;velocity.covariance(k,k)=g.speed_accuracy_m_s*g.speed_accuracy_m_s;}
  return true;
}
}

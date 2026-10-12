#ifndef NAV_PROTOCOL_H
#define NAV_PROTOCOL_H

/* 独立 C99 通信接口。禁止直接把以下结构体的内存作为线协议发送。 */
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_PROTOCOL_MAX_PAYLOAD 255u
#define NAV_PROTOCOL_FRAME_OVERHEAD 8u
#define NAV_PROTOCOL_MAX_FRAME (NAV_PROTOCOL_MAX_PAYLOAD + NAV_PROTOCOL_FRAME_OVERHEAD)

#define NAV_MSG_NAV_OUTPUT 0x01u
#define NAV_MSG_TIME_SYNC 0x02u
#define NAV_MSG_CAM_TRIGGER 0x03u
#define NAV_MSG_NAV_PRIOR 0x04u
#define NAV_MSG_VO_VELOCITY 0x10u
#define NAV_MSG_POSITION_FIX 0x11u
#define NAV_MSG_HEALTH 0x20u

#define NAV_PAYLOAD_NAV_OUTPUT 85u
#define NAV_PAYLOAD_TIME_SYNC 13u
#define NAV_PAYLOAD_CAM_TRIGGER 17u
#define NAV_PAYLOAD_NAV_PRIOR 69u
#define NAV_PAYLOAD_VO_VELOCITY 107u
#define NAV_PAYLOAD_POSITION_FIX 106u
#define NAV_PAYLOAD_HEALTH 29u

typedef enum {
    NAV_PROTOCOL_OK = 0,
    NAV_PROTOCOL_BAD_ARGUMENT = -1,
    NAV_PROTOCOL_BAD_LENGTH = -2,
    NAV_PROTOCOL_BAD_VALUE = -3
} nav_protocol_status_t;

typedef enum {
    NAV_WIRE_MODE_GNSS = 0,
    NAV_WIRE_MODE_VISION = 1,
    NAV_WIRE_MODE_TERRAIN = 2,
    NAV_WIRE_MODE_DEAD_RECKONING = 3,
    NAV_WIRE_MODE_REACQUIRE = 4
} nav_wire_mode_t;

typedef enum {
    NAV_WIRE_SOURCE_NONE = 0,
    NAV_WIRE_SOURCE_GNSS = 1,
    NAV_WIRE_SOURCE_VISION = 2,
    NAV_WIRE_SOURCE_TERRAIN_3D = 3,
    NAV_WIRE_SOURCE_TERCOM = 4
} nav_wire_source_t;

typedef enum {
    NAV_WIRE_VELOCITY_BODY = 0,
    NAV_WIRE_VELOCITY_NED = 1
} nav_wire_velocity_frame_t;

/* 位置为 deg/deg/椭球高 m；速度 NED；q_nb 顺序 w,x,y,z；角速度 FRD。 */
typedef struct {
    uint64_t t_us;
    double lat_deg;
    double lon_deg;
    double height_m;
    float vel_ned[3];
    float quaternion[4];
    float angular_rate[3];
    float h_sigma;
    float v_sigma;
    uint8_t mode;
    uint32_t health;
} nav_wire_output_t;

typedef struct {
    uint64_t t_us;
    uint32_t pulse_count;
    uint8_t pps_valid;
} nav_wire_time_sync_t;

typedef struct {
    uint32_t frame_id;
    uint64_t t_mid_exposure_us;
    uint32_t exposure_us;
    uint8_t camera_id;
} nav_wire_cam_trigger_t;

typedef struct {
    uint64_t t_us;
    double lat_deg;
    double lon_deg;
    double height_m;
    float vel_ned[3];
    float quaternion[4];
    float h_sigma;
    float v_sigma;
    uint8_t mode;
} nav_wire_prior_t;

/* covariance 为行主序完整 3x3；仅 dimensions 指定的左上子矩阵参与正定检查。 */
typedef struct {
    uint64_t t_us;
    /* 成员名保留兼容；值和协方差实际坐标系由 frame 决定。 */
    double velocity_body[3];
    double covariance[9];
    uint8_t source;
    uint8_t dimensions;
    uint8_t frame;
} nav_wire_vo_velocity_t;

/* 经纬高单位 deg/deg/m；协方差属于局部 NED，单位 m²。 */
typedef struct {
    uint64_t t_us;
    double lat_deg;
    double lon_deg;
    double height_m;
    double covariance[9];
    uint8_t source;
    uint8_t dimensions;
} nav_wire_position_fix_t;

typedef struct {
    uint64_t t_us;
    uint32_t uptime_ms;
    uint32_t health;
    uint32_t rx_crc_errors;
    uint32_t rx_length_errors;
    uint32_t queue_overflows;
    uint8_t component_id;
} nav_wire_health_t;

typedef struct {
    uint8_t msg_id;
    uint8_t payload_len;
    uint16_t seq;
    uint8_t payload[NAV_PROTOCOL_MAX_PAYLOAD];
} nav_frame_t;

typedef struct {
    uint8_t buffer[NAV_PROTOCOL_MAX_FRAME];
    uint16_t buffered;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t discarded_bytes;
} nav_parser_t;

typedef enum {
    NAV_PARSE_BAD_ARGUMENT = -1,
    NAV_PARSE_INCOMPLETE = 0,
    NAV_PARSE_FRAME = 1
} nav_parse_result_t;

/* CRC-16/CCITT-FALSE：poly=0x1021，init=0xffff，refin/refout=false，xorout=0。 */
/* length=0 允许 data=NULL，返回 0xffff；NULL 且 length>0 返回 0。 */
uint16_t nav_crc16(const uint8_t *data, size_t length);
/* 已知消息返回固定 payload 长度；未知 ID 返回 0，允许扩展消息。 */
size_t nav_message_payload_size(uint8_t msg_id);
/* CRC 覆盖 msg_id,len,seq,payload；帧总长 payload_len+8。失败不修改输出。 */
nav_protocol_status_t nav_frame_encode(uint8_t msg_id, uint16_t seq,
                                      const uint8_t *payload, size_t payload_len,
                                      uint8_t *out, size_t capacity, size_t *out_len);
void nav_parser_init(nav_parser_t *parser);
/* 运输层确认帧间超时后丢弃未完成数据；保留累计计数，增加 discarded_bytes。 */
void nav_parser_reset_stream(nav_parser_t *parser);
/* 每次输入一字节。仅 NAV_PARSE_FRAME 修改 frame；无需 malloc。 */
nav_parse_result_t nav_parser_feed(nav_parser_t *parser, uint8_t byte, nav_frame_t *frame);
/* 恢复坏帧后可能积存多个有效帧，用此函数取尽已缓存帧，不输入新字节。 */
nav_parse_result_t nav_parser_next(nav_parser_t *parser, nav_frame_t *frame);

/* 编码器写 payload，随后用 nav_frame_encode 包帧；失败不修改 payload/out_len。 */
nav_protocol_status_t nav_encode_nav_output(const nav_wire_output_t *msg, uint8_t *payload, size_t capacity, size_t *out_len);
nav_protocol_status_t nav_decode_nav_output(const uint8_t *payload, size_t length, nav_wire_output_t *msg);
nav_protocol_status_t nav_encode_time_sync(const nav_wire_time_sync_t *msg, uint8_t *payload, size_t capacity, size_t *out_len);
nav_protocol_status_t nav_decode_time_sync(const uint8_t *payload, size_t length, nav_wire_time_sync_t *msg);
nav_protocol_status_t nav_encode_cam_trigger(const nav_wire_cam_trigger_t *msg, uint8_t *payload, size_t capacity, size_t *out_len);
nav_protocol_status_t nav_decode_cam_trigger(const uint8_t *payload, size_t length, nav_wire_cam_trigger_t *msg);
nav_protocol_status_t nav_encode_nav_prior(const nav_wire_prior_t *msg, uint8_t *payload, size_t capacity, size_t *out_len);
nav_protocol_status_t nav_decode_nav_prior(const uint8_t *payload, size_t length, nav_wire_prior_t *msg);
nav_protocol_status_t nav_encode_vo_velocity(const nav_wire_vo_velocity_t *msg, uint8_t *payload, size_t capacity, size_t *out_len);
nav_protocol_status_t nav_decode_vo_velocity(const uint8_t *payload, size_t length, nav_wire_vo_velocity_t *msg);
nav_protocol_status_t nav_encode_position_fix(const nav_wire_position_fix_t *msg, uint8_t *payload, size_t capacity, size_t *out_len);
nav_protocol_status_t nav_decode_position_fix(const uint8_t *payload, size_t length, nav_wire_position_fix_t *msg);
nav_protocol_status_t nav_encode_health(const nav_wire_health_t *msg, uint8_t *payload, size_t capacity, size_t *out_len);
nav_protocol_status_t nav_decode_health(const uint8_t *payload, size_t length, nav_wire_health_t *msg);

#ifdef __cplusplus
}
#endif
#endif

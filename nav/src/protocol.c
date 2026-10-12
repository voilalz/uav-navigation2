#include "nav/protocol.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

/* 线协议浮点格式为 IEEE-754 binary32/binary64，拒绝不兼容的平台。 */
#if CHAR_BIT != 8 || FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128 || DBL_MANT_DIG != 53 || DBL_MAX_EXP != 1024
#error "nav protocol requires 8-bit bytes and IEEE-754 binary32/binary64"
#endif
#ifdef __FAST_MATH__
#error "nav protocol finite-value validation must not use fast-math"
#endif
typedef char nav_float_must_be_four_bytes[(sizeof(float) == 4) ? 1 : -1];
typedef char nav_double_must_be_eight_bytes[(sizeof(double) == 8) ? 1 : -1];

static void write_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void write_u32(uint8_t *out, uint32_t value)
{
    size_t i;
    for (i = 0; i < 4; ++i) out[i] = (uint8_t)(value >> (8 * i));
}

static void write_u64(uint8_t *out, uint64_t value)
{
    size_t i;
    for (i = 0; i < 8; ++i) out[i] = (uint8_t)(value >> (8 * i));
}

static uint16_t read_u16(const uint8_t *in)
{
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static uint32_t read_u32(const uint8_t *in)
{
    uint32_t value = 0;
    size_t i;
    for (i = 0; i < 4; ++i) value |= (uint32_t)in[i] << (8 * i);
    return value;
}

static uint64_t read_u64(const uint8_t *in)
{
    uint64_t value = 0;
    size_t i;
    for (i = 0; i < 8; ++i) value |= (uint64_t)in[i] << (8 * i);
    return value;
}

static void write_f32(uint8_t *out, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    write_u32(out, bits);
}

static void write_f64(uint8_t *out, double value)
{
    uint64_t bits;
    memcpy(&bits, &value, sizeof bits);
    write_u64(out, bits);
}

static float read_f32(const uint8_t *in)
{
    const uint32_t bits = read_u32(in);
    float value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static double read_f64(const uint8_t *in)
{
    const uint64_t bits = read_u64(in);
    double value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static void write_float_array(uint8_t *out, const float *value, size_t length)
{
    size_t i;
    for (i = 0; i < length; ++i) write_f32(out + 4 * i, value[i]);
}

static void read_float_array(const uint8_t *in, float *value, size_t length)
{
    size_t i;
    for (i = 0; i < length; ++i) value[i] = read_f32(in + 4 * i);
}

static void write_double_array(uint8_t *out, const double *value, size_t length)
{
    size_t i;
    for (i = 0; i < length; ++i) write_f64(out + 8 * i, value[i]);
}

static void read_double_array(const uint8_t *in, double *value, size_t length)
{
    size_t i;
    for (i = 0; i < length; ++i) value[i] = read_f64(in + 8 * i);
}

static int finite_float_array(const float *value, size_t length)
{
    size_t i;
    for (i = 0; i < length; ++i) if (!isfinite(value[i])) return 0;
    return 1;
}

static int finite_double_array(const double *value, size_t length)
{
    size_t i;
    for (i = 0; i < length; ++i) if (!isfinite(value[i])) return 0;
    return 1;
}

static int valid_position(double lat, double lon, double height)
{
    return isfinite(lat) && isfinite(lon) && isfinite(height) &&
           lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;
}

static int valid_attitude(const float q[4])
{
    double norm_squared = 0;
    size_t i;
    if (!finite_float_array(q, 4)) return 0;
    for (i = 0; i < 4; ++i) norm_squared += (double)q[i] * q[i];
    return norm_squared > 0;
}

static int valid_sigma(float horizontal, float vertical)
{
    return isfinite(horizontal) && isfinite(vertical) && horizontal >= 0 && vertical >= 0;
}

static int valid_output(const nav_wire_output_t *msg)
{
    return valid_position(msg->lat_deg, msg->lon_deg, msg->height_m) &&
           finite_float_array(msg->vel_ned, 3) && finite_float_array(msg->angular_rate, 3) &&
           valid_attitude(msg->quaternion) && valid_sigma(msg->h_sigma, msg->v_sigma) &&
           msg->mode <= NAV_WIRE_MODE_REACQUIRE;
}

static int valid_prior(const nav_wire_prior_t *msg)
{
    return valid_position(msg->lat_deg, msg->lon_deg, msg->height_m) &&
           finite_float_array(msg->vel_ned, 3) && valid_attitude(msg->quaternion) &&
           valid_sigma(msg->h_sigma, msg->v_sigma) && msg->mode <= NAV_WIRE_MODE_REACQUIRE;
}

/* 活跃子矩阵必须对称正定；Cholesky 无堆分配，检查运算溢出。 */
static int valid_covariance(const double covariance[9], uint8_t dimensions)
{
    double lower[9] = {0};
    size_t i;
    size_t j;
    size_t k;
    if (dimensions < 1 || dimensions > 3 || !finite_double_array(covariance, 9)) return 0;
    for (i = 0; i < dimensions; ++i) {
        for (j = 0; j < i; ++j) {
            const double a = covariance[3 * i + j];
            const double b = covariance[3 * j + i];
            const double scale = fmax(1.0, fmax(fabs(a), fabs(b)));
            if (fabs(a - b) > 1e-9 * scale) return 0;
        }
    }
    for (i = 0; i < dimensions; ++i) {
        for (j = 0; j <= i; ++j) {
            double pivot = covariance[3 * i + j];
            /* 容差内仍按对称平均检查；保持原始九个传输元素不变。 */
            if (i != j) pivot += (covariance[3 * j + i] - pivot) * 0.5;
            for (k = 0; k < j; ++k) pivot -= lower[3 * i + k] * lower[3 * j + k];
            if (!isfinite(pivot)) return 0;
            if (i == j) {
                if (!(pivot > 0)) return 0;
                lower[3 * i + j] = sqrt(pivot);
            } else {
                lower[3 * i + j] = pivot / lower[3 * j + j];
                if (!isfinite(lower[3 * i + j])) return 0;
            }
        }
    }
    return 1;
}

static int valid_vo(const nav_wire_vo_velocity_t *msg)
{
    return msg->frame <= NAV_WIRE_VELOCITY_NED && msg->source <= NAV_WIRE_SOURCE_TERCOM &&
           finite_double_array(msg->velocity_body, 3) &&
           valid_covariance(msg->covariance, msg->dimensions);
}

static int valid_fix(const nav_wire_position_fix_t *msg)
{
    return msg->source <= NAV_WIRE_SOURCE_TERCOM && valid_position(msg->lat_deg, msg->lon_deg, msg->height_m) &&
           valid_covariance(msg->covariance, msg->dimensions);
}

static nav_protocol_status_t encode_arguments(const void *msg, uint8_t *payload,
                                              size_t capacity, size_t *out_len, size_t required)
{
    if (!msg || !payload || !out_len) return NAV_PROTOCOL_BAD_ARGUMENT;
    if (capacity < required) return NAV_PROTOCOL_BAD_LENGTH;
    return NAV_PROTOCOL_OK;
}

static nav_protocol_status_t decode_arguments(const uint8_t *payload, size_t length,
                                              void *msg, size_t required)
{
    if (!payload || !msg) return NAV_PROTOCOL_BAD_ARGUMENT;
    if (length != required) return NAV_PROTOCOL_BAD_LENGTH;
    return NAV_PROTOCOL_OK;
}

uint16_t nav_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffff;
    size_t i;
    unsigned bit;
    if (!data && length) return 0;
    for (i = 0; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; ++bit) {
            crc = (uint16_t)((crc & 0x8000) ? ((uint32_t)crc << 1) ^ 0x1021u : (uint32_t)crc << 1);
        }
    }
    return crc;
}

size_t nav_message_payload_size(uint8_t msg_id)
{
    switch (msg_id) {
    case NAV_MSG_NAV_OUTPUT: return NAV_PAYLOAD_NAV_OUTPUT;
    case NAV_MSG_TIME_SYNC: return NAV_PAYLOAD_TIME_SYNC;
    case NAV_MSG_CAM_TRIGGER: return NAV_PAYLOAD_CAM_TRIGGER;
    case NAV_MSG_NAV_PRIOR: return NAV_PAYLOAD_NAV_PRIOR;
    case NAV_MSG_VO_VELOCITY: return NAV_PAYLOAD_VO_VELOCITY;
    case NAV_MSG_POSITION_FIX: return NAV_PAYLOAD_POSITION_FIX;
    case NAV_MSG_HEALTH: return NAV_PAYLOAD_HEALTH;
    default: return 0;
    }
}

nav_protocol_status_t nav_frame_encode(uint8_t msg_id, uint16_t seq,
                                      const uint8_t *payload, size_t payload_len,
                                      uint8_t *out, size_t capacity, size_t *out_len)
{
    const size_t expected = nav_message_payload_size(msg_id);
    if (!out || !out_len || (!payload && payload_len)) return NAV_PROTOCOL_BAD_ARGUMENT;
    if (payload_len > NAV_PROTOCOL_MAX_PAYLOAD || (expected && expected != payload_len)) return NAV_PROTOCOL_BAD_LENGTH;
    if (capacity < payload_len + NAV_PROTOCOL_FRAME_OVERHEAD) return NAV_PROTOCOL_BAD_LENGTH;
    /* 先移动 payload，再写帧头，允许 payload/out 重叠和原地包帧。 */
    if (payload_len) memmove(out + 6, payload, payload_len);
    out[0] = 0xa5;
    out[1] = 0x5a;
    out[2] = msg_id;
    out[3] = (uint8_t)payload_len;
    write_u16(out + 4, seq);
    write_u16(out + 6 + payload_len, nav_crc16(out + 2, payload_len + 4));
    *out_len = payload_len + NAV_PROTOCOL_FRAME_OVERHEAD;
    return NAV_PROTOCOL_OK;
}

static void saturating_add(uint32_t *counter, uint32_t value)
{
    *counter = UINT32_MAX - *counter < value ? UINT32_MAX : *counter + value;
}

static void parser_remove(nav_parser_t *parser, size_t count, int discarded)
{
    const size_t remaining = parser->buffered - count;
    if (remaining) memmove(parser->buffer, parser->buffer + count, remaining);
    parser->buffered = (uint16_t)remaining;
    if (discarded) saturating_add(&parser->discarded_bytes, (uint32_t)count);
}

static int frame_crc_matches(const uint8_t *bytes, size_t length)
{
    return read_u16(bytes + length - 2) == nav_crc16(bytes + 2, length - 4);
}

void nav_parser_init(nav_parser_t *parser)
{
    if (parser) memset(parser, 0, sizeof *parser);
}

void nav_parser_reset_stream(nav_parser_t *parser)
{
    if (!parser) return;
    saturating_add(&parser->discarded_bytes, parser->buffered);
    parser->buffered = 0;
}

nav_parse_result_t nav_parser_feed(nav_parser_t *parser, uint8_t byte, nav_frame_t *frame)
{
    if (!parser || !frame || parser->buffered > NAV_PROTOCOL_MAX_FRAME) return NAV_PARSE_BAD_ARGUMENT;
    if (parser->buffered == NAV_PROTOCOL_MAX_FRAME) parser_remove(parser, 1, 1);
    parser->buffer[parser->buffered++] = byte;
    return nav_parser_next(parser, frame);
}

nav_parse_result_t nav_parser_next(nav_parser_t *parser, nav_frame_t *frame)
{
    if (!parser || !frame || parser->buffered > NAV_PROTOCOL_MAX_FRAME) return NAV_PARSE_BAD_ARGUMENT;
    for (;;) {
        size_t expected;
        size_t total;
        if (parser->buffered < 2) {
            if (parser->buffered == 1 && parser->buffer[0] != 0xa5) parser_remove(parser, 1, 1);
            return NAV_PARSE_INCOMPLETE;
        }
        if (parser->buffer[0] != 0xa5 || parser->buffer[1] != 0x5a) {
            parser_remove(parser, 1, 1);
            continue;
        }
        if (parser->buffered < 4) return NAV_PARSE_INCOMPLETE;
        expected = nav_message_payload_size(parser->buffer[2]);
        if (expected && expected != parser->buffer[3]) {
            saturating_add(&parser->length_errors, 1);
            parser_remove(parser, 1, 1);
            continue;
        }
        total = parser->buffer[3] + NAV_PROTOCOL_FRAME_OVERHEAD;
        if (parser->buffered >= total) {
            if (!frame_crc_matches(parser->buffer, total)) {
                saturating_add(&parser->crc_errors, 1);
                parser_remove(parser, 1, 1);
                continue;
            }
            frame->msg_id = parser->buffer[2];
            frame->payload_len = parser->buffer[3];
            frame->seq = read_u16(parser->buffer + 4);
            if (frame->payload_len) memcpy(frame->payload, parser->buffer + 6, frame->payload_len);
            parser_remove(parser, total, 0);
            saturating_add(&parser->frames_ok, 1);
            return NAV_PARSE_FRAME;
        }
        /* 完整有效外帧可包含任意字节，故外帧未收全时不能扫描内嵌同步字。 */
        return NAV_PARSE_INCOMPLETE;
    }
}

nav_protocol_status_t nav_encode_nav_output(const nav_wire_output_t *msg, uint8_t *payload, size_t capacity, size_t *out_len)
{
    const nav_protocol_status_t status = encode_arguments(msg, payload, capacity, out_len, NAV_PAYLOAD_NAV_OUTPUT);
    if (status != NAV_PROTOCOL_OK) return status;
    if (!valid_output(msg)) return NAV_PROTOCOL_BAD_VALUE;
    write_u64(payload, msg->t_us);
    write_f64(payload + 8, msg->lat_deg);
    write_f64(payload + 16, msg->lon_deg);
    write_f64(payload + 24, msg->height_m);
    write_float_array(payload + 32, msg->vel_ned, 3);
    write_float_array(payload + 44, msg->quaternion, 4);
    write_float_array(payload + 60, msg->angular_rate, 3);
    write_f32(payload + 72, msg->h_sigma);
    write_f32(payload + 76, msg->v_sigma);
    payload[80] = msg->mode;
    write_u32(payload + 81, msg->health);
    *out_len = NAV_PAYLOAD_NAV_OUTPUT;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_decode_nav_output(const uint8_t *payload, size_t length, nav_wire_output_t *msg)
{
    nav_wire_output_t decoded;
    const nav_protocol_status_t status = decode_arguments(payload, length, msg, NAV_PAYLOAD_NAV_OUTPUT);
    if (status != NAV_PROTOCOL_OK) return status;
    memset(&decoded, 0, sizeof decoded);
    decoded.t_us = read_u64(payload);
    decoded.lat_deg = read_f64(payload + 8);
    decoded.lon_deg = read_f64(payload + 16);
    decoded.height_m = read_f64(payload + 24);
    read_float_array(payload + 32, decoded.vel_ned, 3);
    read_float_array(payload + 44, decoded.quaternion, 4);
    read_float_array(payload + 60, decoded.angular_rate, 3);
    decoded.h_sigma = read_f32(payload + 72);
    decoded.v_sigma = read_f32(payload + 76);
    decoded.mode = payload[80];
    decoded.health = read_u32(payload + 81);
    if (!valid_output(&decoded)) return NAV_PROTOCOL_BAD_VALUE;
    *msg = decoded;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_encode_time_sync(const nav_wire_time_sync_t *msg, uint8_t *payload, size_t capacity, size_t *out_len)
{
    const nav_protocol_status_t status = encode_arguments(msg, payload, capacity, out_len, NAV_PAYLOAD_TIME_SYNC);
    if (status != NAV_PROTOCOL_OK) return status;
    if (msg->pps_valid > 1) return NAV_PROTOCOL_BAD_VALUE;
    write_u64(payload, msg->t_us);
    write_u32(payload + 8, msg->pulse_count);
    payload[12] = msg->pps_valid;
    *out_len = NAV_PAYLOAD_TIME_SYNC;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_decode_time_sync(const uint8_t *payload, size_t length, nav_wire_time_sync_t *msg)
{
    nav_wire_time_sync_t decoded;
    const nav_protocol_status_t status = decode_arguments(payload, length, msg, NAV_PAYLOAD_TIME_SYNC);
    if (status != NAV_PROTOCOL_OK) return status;
    memset(&decoded, 0, sizeof decoded);
    decoded.t_us = read_u64(payload);
    decoded.pulse_count = read_u32(payload + 8);
    decoded.pps_valid = payload[12];
    if (decoded.pps_valid > 1) return NAV_PROTOCOL_BAD_VALUE;
    *msg = decoded;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_encode_cam_trigger(const nav_wire_cam_trigger_t *msg, uint8_t *payload, size_t capacity, size_t *out_len)
{
    const nav_protocol_status_t status = encode_arguments(msg, payload, capacity, out_len, NAV_PAYLOAD_CAM_TRIGGER);
    if (status != NAV_PROTOCOL_OK) return status;
    write_u32(payload, msg->frame_id);
    write_u64(payload + 4, msg->t_mid_exposure_us);
    write_u32(payload + 12, msg->exposure_us);
    payload[16] = msg->camera_id;
    *out_len = NAV_PAYLOAD_CAM_TRIGGER;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_decode_cam_trigger(const uint8_t *payload, size_t length, nav_wire_cam_trigger_t *msg)
{
    nav_wire_cam_trigger_t decoded;
    const nav_protocol_status_t status = decode_arguments(payload, length, msg, NAV_PAYLOAD_CAM_TRIGGER);
    if (status != NAV_PROTOCOL_OK) return status;
    memset(&decoded, 0, sizeof decoded);
    decoded.frame_id = read_u32(payload);
    decoded.t_mid_exposure_us = read_u64(payload + 4);
    decoded.exposure_us = read_u32(payload + 12);
    decoded.camera_id = payload[16];
    *msg = decoded;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_encode_nav_prior(const nav_wire_prior_t *msg, uint8_t *payload, size_t capacity, size_t *out_len)
{
    const nav_protocol_status_t status = encode_arguments(msg, payload, capacity, out_len, NAV_PAYLOAD_NAV_PRIOR);
    if (status != NAV_PROTOCOL_OK) return status;
    if (!valid_prior(msg)) return NAV_PROTOCOL_BAD_VALUE;
    write_u64(payload, msg->t_us);
    write_f64(payload + 8, msg->lat_deg);
    write_f64(payload + 16, msg->lon_deg);
    write_f64(payload + 24, msg->height_m);
    write_float_array(payload + 32, msg->vel_ned, 3);
    write_float_array(payload + 44, msg->quaternion, 4);
    write_f32(payload + 60, msg->h_sigma);
    write_f32(payload + 64, msg->v_sigma);
    payload[68] = msg->mode;
    *out_len = NAV_PAYLOAD_NAV_PRIOR;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_decode_nav_prior(const uint8_t *payload, size_t length, nav_wire_prior_t *msg)
{
    nav_wire_prior_t decoded;
    const nav_protocol_status_t status = decode_arguments(payload, length, msg, NAV_PAYLOAD_NAV_PRIOR);
    if (status != NAV_PROTOCOL_OK) return status;
    memset(&decoded, 0, sizeof decoded);
    decoded.t_us = read_u64(payload);
    decoded.lat_deg = read_f64(payload + 8);
    decoded.lon_deg = read_f64(payload + 16);
    decoded.height_m = read_f64(payload + 24);
    read_float_array(payload + 32, decoded.vel_ned, 3);
    read_float_array(payload + 44, decoded.quaternion, 4);
    decoded.h_sigma = read_f32(payload + 60);
    decoded.v_sigma = read_f32(payload + 64);
    decoded.mode = payload[68];
    if (!valid_prior(&decoded)) return NAV_PROTOCOL_BAD_VALUE;
    *msg = decoded;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_encode_vo_velocity(const nav_wire_vo_velocity_t *msg, uint8_t *payload, size_t capacity, size_t *out_len)
{
    const nav_protocol_status_t status = encode_arguments(msg, payload, capacity, out_len, NAV_PAYLOAD_VO_VELOCITY);
    if (status != NAV_PROTOCOL_OK) return status;
    if (!valid_vo(msg)) return NAV_PROTOCOL_BAD_VALUE;
    write_u64(payload, msg->t_us);
    write_double_array(payload + 8, msg->velocity_body, 3);
    write_double_array(payload + 32, msg->covariance, 9);
    payload[104] = msg->source;
    payload[105] = msg->dimensions;
    payload[106] = msg->frame;
    *out_len = NAV_PAYLOAD_VO_VELOCITY;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_decode_vo_velocity(const uint8_t *payload, size_t length, nav_wire_vo_velocity_t *msg)
{
    nav_wire_vo_velocity_t decoded;
    const nav_protocol_status_t status = decode_arguments(payload, length, msg, NAV_PAYLOAD_VO_VELOCITY);
    if (status != NAV_PROTOCOL_OK) return status;
    memset(&decoded, 0, sizeof decoded);
    decoded.t_us = read_u64(payload);
    read_double_array(payload + 8, decoded.velocity_body, 3);
    read_double_array(payload + 32, decoded.covariance, 9);
    decoded.source = payload[104];
    decoded.dimensions = payload[105];
    decoded.frame = payload[106];
    if (!valid_vo(&decoded)) return NAV_PROTOCOL_BAD_VALUE;
    *msg = decoded;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_encode_position_fix(const nav_wire_position_fix_t *msg, uint8_t *payload, size_t capacity, size_t *out_len)
{
    const nav_protocol_status_t status = encode_arguments(msg, payload, capacity, out_len, NAV_PAYLOAD_POSITION_FIX);
    if (status != NAV_PROTOCOL_OK) return status;
    if (!valid_fix(msg)) return NAV_PROTOCOL_BAD_VALUE;
    write_u64(payload, msg->t_us);
    write_f64(payload + 8, msg->lat_deg);
    write_f64(payload + 16, msg->lon_deg);
    write_f64(payload + 24, msg->height_m);
    write_double_array(payload + 32, msg->covariance, 9);
    payload[104] = msg->source;
    payload[105] = msg->dimensions;
    *out_len = NAV_PAYLOAD_POSITION_FIX;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_decode_position_fix(const uint8_t *payload, size_t length, nav_wire_position_fix_t *msg)
{
    nav_wire_position_fix_t decoded;
    const nav_protocol_status_t status = decode_arguments(payload, length, msg, NAV_PAYLOAD_POSITION_FIX);
    if (status != NAV_PROTOCOL_OK) return status;
    memset(&decoded, 0, sizeof decoded);
    decoded.t_us = read_u64(payload);
    decoded.lat_deg = read_f64(payload + 8);
    decoded.lon_deg = read_f64(payload + 16);
    decoded.height_m = read_f64(payload + 24);
    read_double_array(payload + 32, decoded.covariance, 9);
    decoded.source = payload[104];
    decoded.dimensions = payload[105];
    if (!valid_fix(&decoded)) return NAV_PROTOCOL_BAD_VALUE;
    *msg = decoded;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_encode_health(const nav_wire_health_t *msg, uint8_t *payload, size_t capacity, size_t *out_len)
{
    const nav_protocol_status_t status = encode_arguments(msg, payload, capacity, out_len, NAV_PAYLOAD_HEALTH);
    if (status != NAV_PROTOCOL_OK) return status;
    write_u64(payload, msg->t_us);
    write_u32(payload + 8, msg->uptime_ms);
    write_u32(payload + 12, msg->health);
    write_u32(payload + 16, msg->rx_crc_errors);
    write_u32(payload + 20, msg->rx_length_errors);
    write_u32(payload + 24, msg->queue_overflows);
    payload[28] = msg->component_id;
    *out_len = NAV_PAYLOAD_HEALTH;
    return NAV_PROTOCOL_OK;
}

nav_protocol_status_t nav_decode_health(const uint8_t *payload, size_t length, nav_wire_health_t *msg)
{
    nav_wire_health_t decoded;
    const nav_protocol_status_t status = decode_arguments(payload, length, msg, NAV_PAYLOAD_HEALTH);
    if (status != NAV_PROTOCOL_OK) return status;
    memset(&decoded, 0, sizeof decoded);
    decoded.t_us = read_u64(payload);
    decoded.uptime_ms = read_u32(payload + 8);
    decoded.health = read_u32(payload + 12);
    decoded.rx_crc_errors = read_u32(payload + 16);
    decoded.rx_length_errors = read_u32(payload + 20);
    decoded.queue_overflows = read_u32(payload + 24);
    decoded.component_id = payload[28];
    *msg = decoded;
    return NAV_PROTOCOL_OK;
}

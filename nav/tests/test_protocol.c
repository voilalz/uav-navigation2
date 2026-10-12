#include "nav/protocol.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;
static unsigned checks;

#define CHECK(expr) do { \
    ++checks; \
    if (!(expr)) { \
        fprintf(stderr, "%s:%d: FAILED: %s\n", __FILE__, __LINE__, #expr); \
        ++failures; \
    } \
} while (0)

/* 固定参考字节独立于实现，覆盖字段顺序、大小、小端、float/double 与符号。 */
static const uint8_t output_golden[NAV_PAYLOAD_NAV_OUTPUT] = {
    0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x46, 0x40,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x56, 0xc0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x40,
    0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0xc0,
    0x00, 0x00, 0x00, 0x3f,
    0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0x3e, 0x00, 0x00, 0x00, 0xbf,
    0x00, 0x00, 0x00, 0x40,
    0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80, 0x40,
    0x02, 0x12, 0x34, 0x56, 0x78
};

static const uint8_t time_golden[NAV_PAYLOAD_TIME_SYNC] = {
    0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    0x44, 0x33, 0x22, 0x11, 0x01
};

static const uint8_t cam_golden[NAV_PAYLOAD_CAM_TRIGGER] = {
    0x44, 0x33, 0x22, 0x11,
    0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    0x10, 0x27, 0x00, 0x00, 0x02
};

static const uint8_t covariance_golden[72] = {
    0, 0, 0, 0, 0, 0, 0x10, 0x40,
    0, 0, 0, 0, 0, 0, 0xf0, 0x3f,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0xf0, 0x3f,
    0, 0, 0, 0, 0, 0, 0x22, 0x40,
    0, 0, 0, 0, 0, 0, 0xe0, 0x3f,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0xe0, 0x3f,
    0, 0, 0, 0, 0, 0, 0x30, 0x40
};

static const double covariance[9] = {4, 1, 0, 1, 9, 0.5, 0, 0.5, 16};

static nav_wire_output_t sample_output(void)
{
    nav_wire_output_t msg;
    memset(&msg, 0, sizeof msg);
    msg.t_us = UINT64_C(0x0102030405060708);
    msg.lat_deg = 45;
    msg.lon_deg = -90;
    msg.height_m = 100;
    msg.vel_ned[0] = 1;
    msg.vel_ned[1] = -2;
    msg.vel_ned[2] = 0.5f;
    msg.quaternion[0] = 1;
    msg.angular_rate[0] = 0.25f;
    msg.angular_rate[1] = -0.5f;
    msg.angular_rate[2] = 2;
    msg.h_sigma = 3;
    msg.v_sigma = 4;
    msg.mode = NAV_WIRE_MODE_TERRAIN;
    msg.health = UINT32_C(0x78563412);
    return msg;
}

static void test_crc_and_frame_golden(void)
{
    static const uint8_t text[] = "123456789";
    /* 此帧 CRC 0xe3d4 来自 Python binascii.crc_hqx(init=0xffff)。 */
    static const uint8_t golden[] = {0xa5, 0x5a, 0x7e, 3, 0x34, 0x12, 0, 0xa5, 0x5a, 0xd4, 0xe3};
    const uint8_t payload[] = {0, 0xa5, 0x5a};
    uint8_t out[32];
    size_t length = 999;
    CHECK(nav_crc16(text, 9) == 0x29b1);
    CHECK(nav_crc16(NULL, 0) == 0xffff);
    CHECK(nav_frame_encode(0x7e, 0x1234, payload, sizeof payload, out, sizeof out, &length) == NAV_PROTOCOL_OK);
    CHECK(length == sizeof golden);
    CHECK(memcmp(out, golden, sizeof golden) == 0);
}

static void test_output_layout(void)
{
    nav_wire_output_t msg = sample_output();
    nav_wire_output_t decoded;
    uint8_t payload[255];
    size_t length = 999;
    size_t i;
    memset(&decoded, 0, sizeof decoded);
    CHECK(nav_encode_nav_output(&msg, payload, sizeof payload, &length) == NAV_PROTOCOL_OK);
    CHECK(length == NAV_PAYLOAD_NAV_OUTPUT);
    CHECK(memcmp(payload, output_golden, sizeof output_golden) == 0);
    CHECK(nav_decode_nav_output(output_golden, sizeof output_golden, &decoded) == NAV_PROTOCOL_OK);
    CHECK(decoded.t_us == msg.t_us);
    CHECK(decoded.lat_deg == 45 && decoded.lon_deg == -90 && decoded.height_m == 100);
    for (i = 0; i < 3; ++i) {
        CHECK(decoded.vel_ned[i] == msg.vel_ned[i]);
        CHECK(decoded.angular_rate[i] == msg.angular_rate[i]);
    }
    for (i = 0; i < 4; ++i) CHECK(decoded.quaternion[i] == msg.quaternion[i]);
    CHECK(decoded.h_sigma == 3 && decoded.v_sigma == 4);
    CHECK(decoded.mode == NAV_WIRE_MODE_TERRAIN && decoded.health == msg.health);
}

static void test_time_camera_prior_health(void)
{
    nav_wire_time_sync_t time_msg = {UINT64_C(0x0102030405060708), UINT32_C(0x11223344), 1};
    nav_wire_cam_trigger_t cam_msg = {UINT32_C(0x11223344), UINT64_C(0x0102030405060708), 10000, 2};
    nav_wire_time_sync_t time_decoded;
    nav_wire_cam_trigger_t cam_decoded;
    nav_wire_prior_t prior;
    nav_wire_prior_t prior_decoded;
    nav_wire_health_t health = {UINT64_C(0x0102030405060708), 1, UINT32_C(0x11223344), 2, 3, 4, 5};
    nav_wire_health_t health_decoded;
    const uint8_t health_golden[NAV_PAYLOAD_HEALTH] = {
        8, 7, 6, 5, 4, 3, 2, 1, 1, 0, 0, 0,
        0x44, 0x33, 0x22, 0x11, 2, 0, 0, 0,
        3, 0, 0, 0, 4, 0, 0, 0, 5
    };
    uint8_t prior_golden[NAV_PAYLOAD_NAV_PRIOR];
    uint8_t payload[255];
    size_t length = 0;
    nav_wire_output_t output = sample_output();
    memset(&prior, 0, sizeof prior);
    memset(&prior_decoded, 0, sizeof prior_decoded);
    memset(&time_decoded, 0, sizeof time_decoded);
    memset(&cam_decoded, 0, sizeof cam_decoded);
    memset(&health_decoded, 0, sizeof health_decoded);
    prior.t_us = output.t_us;
    prior.lat_deg = output.lat_deg;
    prior.lon_deg = output.lon_deg;
    prior.height_m = output.height_m;
    memcpy(prior.vel_ned, output.vel_ned, sizeof prior.vel_ned);
    memcpy(prior.quaternion, output.quaternion, sizeof prior.quaternion);
    prior.h_sigma = output.h_sigma;
    prior.v_sigma = output.v_sigma;
    prior.mode = output.mode;
    memcpy(prior_golden, output_golden, 60);
    memcpy(prior_golden + 60, output_golden + 72, 9);
    CHECK(nav_encode_time_sync(&time_msg, payload, sizeof payload, &length) == NAV_PROTOCOL_OK);
    CHECK(length == sizeof time_golden && memcmp(payload, time_golden, length) == 0);
    CHECK(nav_decode_time_sync(time_golden, sizeof time_golden, &time_decoded) == NAV_PROTOCOL_OK);
    CHECK(time_decoded.t_us == time_msg.t_us && time_decoded.pulse_count == time_msg.pulse_count && time_decoded.pps_valid == 1);
    CHECK(nav_encode_cam_trigger(&cam_msg, payload, sizeof payload, &length) == NAV_PROTOCOL_OK);
    CHECK(length == sizeof cam_golden && memcmp(payload, cam_golden, length) == 0);
    CHECK(nav_decode_cam_trigger(cam_golden, sizeof cam_golden, &cam_decoded) == NAV_PROTOCOL_OK);
    CHECK(cam_decoded.frame_id == cam_msg.frame_id && cam_decoded.t_mid_exposure_us == cam_msg.t_mid_exposure_us);
    CHECK(cam_decoded.exposure_us == 10000 && cam_decoded.camera_id == 2);
    CHECK(nav_encode_nav_prior(&prior, payload, sizeof payload, &length) == NAV_PROTOCOL_OK);
    CHECK(length == sizeof prior_golden && memcmp(payload, prior_golden, length) == 0);
    CHECK(nav_decode_nav_prior(prior_golden, sizeof prior_golden, &prior_decoded) == NAV_PROTOCOL_OK);
    CHECK(prior_decoded.t_us == prior.t_us && prior_decoded.lat_deg == 45 && prior_decoded.lon_deg == -90);
    CHECK(prior_decoded.height_m == 100 && prior_decoded.vel_ned[1] == -2 && prior_decoded.quaternion[0] == 1);
    CHECK(prior_decoded.h_sigma == 3 && prior_decoded.v_sigma == 4 && prior_decoded.mode == 2);
    CHECK(nav_encode_health(&health, payload, sizeof payload, &length) == NAV_PROTOCOL_OK);
    CHECK(length == sizeof health_golden && memcmp(payload, health_golden, length) == 0);
    CHECK(nav_decode_health(health_golden, sizeof health_golden, &health_decoded) == NAV_PROTOCOL_OK);
    CHECK(health_decoded.t_us == health.t_us && health_decoded.uptime_ms == 1 && health_decoded.health == health.health);
    CHECK(health_decoded.rx_crc_errors == 2 && health_decoded.rx_length_errors == 3 && health_decoded.queue_overflows == 4 && health_decoded.component_id == 5);
}

static void test_measurement_layout(void)
{
    const uint8_t velocity_golden[24] = {
        0, 0, 0, 0, 0, 0, 0xf0, 0x3f,
        0, 0, 0, 0, 0, 0, 0, 0xc0,
        0, 0, 0, 0, 0, 0, 0xe0, 0x3f
    };
    nav_wire_vo_velocity_t vo;
    nav_wire_vo_velocity_t vo_decoded;
    nav_wire_position_fix_t pos;
    nav_wire_position_fix_t pos_decoded;
    uint8_t vo_golden[NAV_PAYLOAD_VO_VELOCITY];
    uint8_t pos_golden[NAV_PAYLOAD_POSITION_FIX];
    uint8_t payload[255];
    size_t length = 0;
    size_t i;
    memset(&vo, 0, sizeof vo);
    memset(&vo_decoded, 0, sizeof vo_decoded);
    memset(&pos, 0, sizeof pos);
    memset(&pos_decoded, 0, sizeof pos_decoded);
    vo.t_us = UINT64_C(0x0102030405060708);
    vo.velocity_body[0] = 1;
    vo.velocity_body[1] = -2;
    vo.velocity_body[2] = 0.5;
    memcpy(vo.covariance, covariance, sizeof covariance);
    vo.source = NAV_WIRE_SOURCE_VISION;
    vo.dimensions = 3;
    pos.t_us = vo.t_us;
    pos.lat_deg = 45;
    pos.lon_deg = -90;
    pos.height_m = 100;
    memcpy(pos.covariance, covariance, sizeof covariance);
    pos.source = NAV_WIRE_SOURCE_TERRAIN_3D;
    pos.dimensions = 3;
    memcpy(vo_golden, output_golden, 8);
    memcpy(vo_golden + 8, velocity_golden, 24);
    memcpy(vo_golden + 32, covariance_golden, sizeof covariance_golden);
    vo_golden[104] = 2;
    vo_golden[105] = 3;
    vo_golden[106] = NAV_WIRE_VELOCITY_BODY;
    memcpy(pos_golden, output_golden, 32);
    memcpy(pos_golden + 32, covariance_golden, sizeof covariance_golden);
    pos_golden[104] = 3;
    pos_golden[105] = 3;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_OK);
    CHECK(length == sizeof vo_golden && memcmp(payload, vo_golden, length) == 0);
    CHECK(nav_decode_vo_velocity(vo_golden, sizeof vo_golden, &vo_decoded) == NAV_PROTOCOL_OK);
    CHECK(vo_decoded.t_us == vo.t_us && vo_decoded.source == 2 && vo_decoded.dimensions == 3 && vo_decoded.frame == NAV_WIRE_VELOCITY_BODY);
    for (i = 0; i < 3; ++i) CHECK(vo_decoded.velocity_body[i] == vo.velocity_body[i]);
    CHECK(nav_encode_position_fix(&pos, payload, sizeof payload, &length) == NAV_PROTOCOL_OK);
    CHECK(length == sizeof pos_golden && memcmp(payload, pos_golden, length) == 0);
    CHECK(nav_decode_position_fix(pos_golden, sizeof pos_golden, &pos_decoded) == NAV_PROTOCOL_OK);
    CHECK(pos_decoded.t_us == pos.t_us && pos_decoded.lat_deg == 45 && pos_decoded.lon_deg == -90 && pos_decoded.height_m == 100);
    CHECK(pos_decoded.source == 3 && pos_decoded.dimensions == 3);
    for (i = 0; i < 9; ++i) {
        CHECK(vo_decoded.covariance[i] == covariance[i]);
        CHECK(pos_decoded.covariance[i] == covariance[i]);
    }
}

static unsigned feed_bytes(nav_parser_t *parser, const uint8_t *bytes, size_t length, nav_frame_t *frame)
{
    unsigned frames = 0;
    size_t i;
    for (i = 0; i < length; ++i) {
        nav_parse_result_t result = nav_parser_feed(parser, bytes[i], frame);
        CHECK(result != NAV_PARSE_BAD_ARGUMENT);
        if (result == NAV_PARSE_FRAME) ++frames;
    }
    return frames;
}

static void test_parser_stream(void)
{
    nav_parser_t parser;
    nav_frame_t frame;
    nav_frame_t untouched;
    uint8_t wire[263];
    uint8_t corrupt[263];
    const uint8_t noise[] = {0, 0x5a, 0xa5, 0xa5};
    size_t length = 0;
    size_t i;
    nav_parser_init(&parser);
    memset(&frame, 0x3c, sizeof frame);
    untouched = frame;
    CHECK(nav_frame_encode(NAV_MSG_NAV_OUTPUT, 65535, output_golden, sizeof output_golden, wire, sizeof wire, &length) == NAV_PROTOCOL_OK);
    CHECK(length == 93);
    if (length == 0) return;
    CHECK(feed_bytes(&parser, noise, sizeof noise, &frame) == 0);
    CHECK(memcmp(&frame, &untouched, sizeof frame) == 0);
    for (i = 0; i + 1 < length; ++i) CHECK(nav_parser_feed(&parser, wire[i], &frame) == NAV_PARSE_INCOMPLETE);
    CHECK(nav_parser_feed(&parser, wire[length - 1], &frame) == NAV_PARSE_FRAME);
    CHECK(frame.msg_id == NAV_MSG_NAV_OUTPUT && frame.seq == 65535 && frame.payload_len == 85);
    CHECK(memcmp(frame.payload, output_golden, 85) == 0);
    CHECK(parser.frames_ok == 1 && parser.discarded_bytes == sizeof noise);
    CHECK(feed_bytes(&parser, wire, length, &frame) == 1);
    memcpy(corrupt, wire, length);
    corrupt[30] ^= 0x40;
    CHECK(feed_bytes(&parser, corrupt, length, &frame) == 0);
    CHECK(parser.crc_errors == 1);
    CHECK(feed_bytes(&parser, wire, length, &frame) == 1);
    CHECK(parser.frames_ok == 3);
    nav_parser_init(&parser);
    /* 已知 ID 的错误长度必须早拒绝，不能等到填满伪长度。 */
    {
        const uint8_t bad_length[] = {0xa5, 0x5a, NAV_MSG_NAV_OUTPUT, 255, 0, 0, 0xa5};
        CHECK(feed_bytes(&parser, bad_length, sizeof bad_length, &frame) == 0);
        CHECK(parser.length_errors == 1);
        CHECK(feed_bytes(&parser, wire, length, &frame) == 1);
    }
    nav_parser_init(&parser);
    /* 未知 ID 长帧先等收全，CRC 失败后才扫描有效后续帧。 */
    {
        const uint8_t truncated[] = {0xa5, 0x5a, 0x7e, 255, 0, 0, 1, 2, 3};
        uint8_t padding[NAV_PROTOCOL_MAX_FRAME - sizeof truncated - 93];
        memset(padding, 0x33, sizeof padding);
        CHECK(feed_bytes(&parser, truncated, sizeof truncated, &frame) == 0);
        CHECK(feed_bytes(&parser, wire, length, &frame) == 0);
        CHECK(feed_bytes(&parser, padding, sizeof padding, &frame) == 1);
        CHECK(parser.crc_errors == 1 && parser.length_errors == 0 && parser.discarded_bytes == sizeof truncated);
        nav_parser_reset_stream(&parser);
        CHECK(parser.buffered == 0 && parser.frames_ok == 1 && parser.crc_errors == 1);
        CHECK(parser.discarded_bytes == sizeof truncated + sizeof padding);
    }
    nav_parser_init(&parser);
    /* 运输层超时丢弃截断数据，累计统计保留，后续短帧可正常接收。 */
    {
        uint8_t long_truncated[200];
        uint8_t short_wire[8];
        size_t short_length = 0;
        memset(long_truncated, 0x33, sizeof long_truncated);
        long_truncated[0] = 0xa5; long_truncated[1] = 0x5a;
        long_truncated[2] = 0x7e; long_truncated[3] = 255;
        CHECK(nav_frame_encode(0x7f, 12, NULL, 0, short_wire, sizeof short_wire, &short_length) == NAV_PROTOCOL_OK);
        CHECK(feed_bytes(&parser, long_truncated, sizeof long_truncated, &frame) == 0);
        CHECK(feed_bytes(&parser, short_wire, short_length, &frame) == 0);
        nav_parser_reset_stream(&parser);
        CHECK(parser.buffered == 0 && parser.discarded_bytes == sizeof long_truncated + short_length);
        CHECK(feed_bytes(&parser, short_wire, short_length, &frame) == 1);
        CHECK(frame.msg_id == 0x7f && frame.seq == 12);
    }
}

static void test_maximum_and_overlap(void)
{
    uint8_t payload[255];
    uint8_t wire[263];
    nav_parser_t parser;
    nav_frame_t frame;
    size_t length = 0;
    size_t i;
    memset(&frame, 0, sizeof frame);
    for (i = 0; i < sizeof payload; ++i) payload[i] = (uint8_t)i;
    /* 有效 payload 中出现同步字节不会被简单同步扫描误截断。 */
    payload[10] = 0xa5;
    payload[11] = 0x5a;
    nav_parser_init(&parser);
    CHECK(nav_frame_encode(0x7f, 0, payload, sizeof payload, wire, sizeof wire, &length) == NAV_PROTOCOL_OK);
    CHECK(length == sizeof wire);
    CHECK(feed_bytes(&parser, wire, length, &frame) == 1);
    CHECK(frame.msg_id == 0x7f && frame.payload_len == 255 && memcmp(frame.payload, payload, sizeof payload) == 0);
    CHECK(parser.crc_errors == 0 && parser.length_errors == 0);
    CHECK(nav_frame_encode(0x7f, 0x1234, NULL, 0, wire, sizeof wire, &length) == NAV_PROTOCOL_OK);
    CHECK(length == 8 && feed_bytes(&parser, wire, length, &frame) == 1 && frame.payload_len == 0);
    memcpy(wire, time_golden, sizeof time_golden);
    CHECK(nav_frame_encode(NAV_MSG_TIME_SYNC, 1, wire, sizeof time_golden, wire, sizeof wire, &length) == NAV_PROTOCOL_OK);
    CHECK(memcmp(wire + 6, time_golden, sizeof time_golden) == 0);
    CHECK(feed_bytes(&parser, wire, length, &frame) == 1);
}

static void test_preserves_embedded_complete_frame(void)
{
    uint8_t payload[255];
    uint8_t wire[263];
    uint8_t nested[8];
    nav_parser_t parser;
    nav_frame_t frame;
    size_t length = 0;
    size_t nested_length = 0;
    size_t i;
    memset(&frame, 0, sizeof frame);
    memset(payload, 0x33, sizeof payload);
    nav_parser_init(&parser);
    CHECK(nav_frame_encode(0x7f, 12, NULL, 0, nested, sizeof nested, &nested_length) == NAV_PROTOCOL_OK);
    memcpy(payload + 31, nested, nested_length);
    CHECK(nav_frame_encode(0x7e, 77, payload, sizeof payload, wire, sizeof wire, &length) == NAV_PROTOCOL_OK);
    for (i = 0; i + 1 < length; ++i) CHECK(nav_parser_feed(&parser, wire[i], &frame) == NAV_PARSE_INCOMPLETE);
    CHECK(nav_parser_feed(&parser, wire[length - 1], &frame) == NAV_PARSE_FRAME);
    CHECK(frame.msg_id == 0x7e && frame.seq == 77 && frame.payload_len == 255);
    CHECK(memcmp(frame.payload, payload, sizeof payload) == 0);
    CHECK(parser.frames_ok == 1 && parser.crc_errors == 0 && parser.length_errors == 0);
}

static void test_drains_recovered_frames(void)
{
    uint8_t stream[263];
    uint8_t wire[21];
    nav_parser_t parser;
    nav_frame_t frame;
    size_t length = 0;
    uint16_t crc;
    memset(&frame, 0, sizeof frame);
    memset(stream, 0x33, sizeof stream);
    stream[0] = 0xa5; stream[1] = 0x5a;
    stream[2] = 0x7e; stream[3] = 255;
    CHECK(nav_frame_encode(NAV_MSG_TIME_SYNC, 10, time_golden, sizeof time_golden, wire, sizeof wire, &length) == NAV_PROTOCOL_OK);
    memcpy(stream + 10, wire, length);
    CHECK(nav_frame_encode(NAV_MSG_TIME_SYNC, 11, time_golden, sizeof time_golden, wire, sizeof wire, &length) == NAV_PROTOCOL_OK);
    memcpy(stream + 10 + length, wire, length);
    crc = nav_crc16(stream + 2, sizeof stream - 4);
    stream[261] = (uint8_t)(crc ^ 1); stream[262] = (uint8_t)(crc >> 8);
    nav_parser_init(&parser);
    CHECK(feed_bytes(&parser, stream, sizeof stream, &frame) == 1);
    CHECK(frame.msg_id == NAV_MSG_TIME_SYNC && frame.seq == 10);
    CHECK(nav_parser_next(&parser, &frame) == NAV_PARSE_FRAME);
    CHECK(frame.msg_id == NAV_MSG_TIME_SYNC && frame.seq == 11);
    CHECK(nav_parser_next(&parser, &frame) == NAV_PARSE_INCOMPLETE);
    CHECK(parser.frames_ok == 2 && parser.crc_errors == 1);
}

static void test_corruption_and_truncation_sweep(void)
{
    uint8_t wire[93];
    uint8_t corrupt[93];
    nav_parser_t parser;
    nav_frame_t frame;
    size_t length = 0;
    size_t offset;
    unsigned bit;
    CHECK(nav_frame_encode(NAV_MSG_NAV_OUTPUT, 42, output_golden, sizeof output_golden, wire, sizeof wire, &length) == NAV_PROTOCOL_OK);
    for (offset = 0; offset < length; ++offset) {
        for (bit = 0; bit < 8; ++bit) {
            memcpy(corrupt, wire, length);
            corrupt[offset] ^= (uint8_t)(1u << bit);
            nav_parser_init(&parser);
            CHECK(feed_bytes(&parser, corrupt, length, &frame) == 0);
            CHECK(feed_bytes(&parser, wire, length, &frame) == 1);
            CHECK(frame.msg_id == NAV_MSG_NAV_OUTPUT && frame.seq == 42 && frame.payload_len == 85);
            CHECK(memcmp(frame.payload, output_golden, sizeof output_golden) == 0);
            CHECK(parser.buffered <= NAV_PROTOCOL_MAX_FRAME);
        }
    }
    /* 每个截断位置后接一帧，验证重同步；不完整伪长头允许等到其长度收全。 */
    for (offset = 0; offset < length; ++offset) {
        unsigned recovered;
        nav_parser_init(&parser);
        CHECK(feed_bytes(&parser, wire, offset, &frame) == 0);
        recovered = feed_bytes(&parser, wire, length, &frame);
        if (recovered == 0) {
            uint8_t padding[NAV_PROTOCOL_MAX_FRAME];
            memset(padding, 0x33, sizeof padding);
            recovered += feed_bytes(&parser, padding, sizeof padding, &frame);
        }
        CHECK(recovered == 1);
        CHECK(frame.seq == 42 && memcmp(frame.payload, output_golden, sizeof output_golden) == 0);
    }
}

static void test_wire_numeric_rejection_and_partial_dimensions(void)
{
    nav_wire_vo_velocity_t vo;
    nav_wire_vo_velocity_t vo_decoded;
    nav_wire_position_fix_t fix;
    nav_wire_position_fix_t fix_decoded;
    nav_wire_prior_t prior;
    nav_wire_prior_t prior_decoded;
    nav_wire_time_sync_t time_decoded;
    uint8_t valid[255];
    uint8_t invalid[255];
    size_t length = 0;
    memset(&vo, 0, sizeof vo);
    memset(&vo_decoded, 0x33, sizeof vo_decoded);
    memset(&fix, 0, sizeof fix);
    memset(&fix_decoded, 0x33, sizeof fix_decoded);
    memset(&prior, 0, sizeof prior);
    memset(&prior_decoded, 0x33, sizeof prior_decoded);
    vo.dimensions = 3;
    vo.source = NAV_WIRE_SOURCE_VISION;
    memcpy(vo.covariance, covariance, sizeof covariance);
    CHECK(nav_encode_vo_velocity(&vo, valid, sizeof valid, &length) == NAV_PROTOCOL_OK);
    memcpy(invalid, valid, length); invalid[105] = 0;
    CHECK(nav_decode_vo_velocity(invalid, length, &vo_decoded) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(invalid, valid, length); invalid[104] = 255;
    CHECK(nav_decode_vo_velocity(invalid, length, &vo_decoded) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(invalid, valid, length); memset(invalid + 96, 0, 8);
    CHECK(nav_decode_vo_velocity(invalid, length, &vo_decoded) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(invalid, valid, length); memset(invalid + 96, 0, 8); invalid[102] = 0xf8; invalid[103] = 0x7f;
    CHECK(nav_decode_vo_velocity(invalid, length, &vo_decoded) == NAV_PROTOCOL_BAD_VALUE);
    /* 三个正对角元仍不能替代完整矩阵的正定检查。 */
    vo.covariance[0] = 1; vo.covariance[4] = 1; vo.covariance[8] = 1;
    vo.covariance[1] = 0; vo.covariance[3] = 0;
    vo.covariance[2] = 1; vo.covariance[6] = 1;
    vo.covariance[5] = 1; vo.covariance[7] = 1;
    CHECK(nav_encode_vo_velocity(&vo, invalid, sizeof invalid, &length) == NAV_PROTOCOL_BAD_VALUE);
    /* 容差内的不对称也不能绕过微小方差矩阵的 SPD 检查。 */
    memset(vo.covariance, 0, sizeof vo.covariance);
    vo.covariance[0] = 1e-12; vo.covariance[4] = 1e-12;
    vo.covariance[1] = 1e-10;
    vo.dimensions = 2;
    CHECK(nav_encode_vo_velocity(&vo, invalid, sizeof invalid, &length) == NAV_PROTOCOL_BAD_VALUE);
    memset(vo.covariance, 0, sizeof vo.covariance);
    vo.covariance[0] = 1;
    vo.dimensions = 1;
    CHECK(nav_encode_vo_velocity(&vo, valid, sizeof valid, &length) == NAV_PROTOCOL_OK);
    CHECK(nav_decode_vo_velocity(valid, length, &vo_decoded) == NAV_PROTOCOL_OK && vo_decoded.dimensions == 1);
    vo.covariance[4] = 2; vo.dimensions = 2;
    CHECK(nav_encode_vo_velocity(&vo, valid, sizeof valid, &length) == NAV_PROTOCOL_OK);
    CHECK(nav_decode_vo_velocity(valid, length, &vo_decoded) == NAV_PROTOCOL_OK && vo_decoded.dimensions == 2);
    fix.dimensions = 3;
    memcpy(fix.covariance, covariance, sizeof covariance);
    CHECK(nav_encode_position_fix(&fix, valid, sizeof valid, &length) == NAV_PROTOCOL_OK);
    memcpy(invalid, valid, length); memset(invalid + 8, 0, 8); invalid[14] = 0xf8; invalid[15] = 0x7f;
    CHECK(nav_decode_position_fix(invalid, length, &fix_decoded) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(invalid, valid, length); invalid[105] = 4;
    CHECK(nav_decode_position_fix(invalid, length, &fix_decoded) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(invalid, valid, length); memset(invalid + 32, 0, 8);
    CHECK(nav_decode_position_fix(invalid, length, &fix_decoded) == NAV_PROTOCOL_BAD_VALUE);
    prior.quaternion[0] = 1;
    CHECK(nav_encode_nav_prior(&prior, valid, sizeof valid, &length) == NAV_PROTOCOL_OK);
    memcpy(invalid, valid, length); invalid[68] = 255;
    CHECK(nav_decode_nav_prior(invalid, length, &prior_decoded) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(invalid, valid, length); memset(invalid + 44, 0, 16);
    CHECK(nav_decode_nav_prior(invalid, length, &prior_decoded) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(invalid, valid, length); invalid[62] = 0xc0; invalid[63] = 0x7f;
    CHECK(nav_decode_nav_prior(invalid, length, &prior_decoded) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(invalid, time_golden, sizeof time_golden); invalid[12] = 2;
    CHECK(nav_decode_time_sync(invalid, sizeof time_golden, &time_decoded) == NAV_PROTOCOL_BAD_VALUE);
    /* 无对齐要求：串口缓冲或以太网包中的 payload 可从奇地址开始。 */
    memcpy(invalid + 1, output_golden, sizeof output_golden);
    {
        nav_wire_output_t decoded;
        CHECK(nav_decode_nav_output(invalid + 1, sizeof output_golden, &decoded) == NAV_PROTOCOL_OK);
        CHECK(decoded.lat_deg == 45 && decoded.lon_deg == -90 && decoded.health == UINT32_C(0x78563412));
    }
}

static void test_vo_ned_2d_and_unknown_frame(void)
{
    nav_wire_vo_velocity_t vo;
    nav_wire_vo_velocity_t decoded;
    nav_wire_vo_velocity_t unchanged;
    uint8_t payload[NAV_PAYLOAD_VO_VELOCITY];
    uint8_t original[NAV_PAYLOAD_VO_VELOCITY];
    size_t length = 0;
    memset(&vo, 0, sizeof vo);
    memset(&decoded, 0x33, sizeof decoded);
    memset(payload, 0xcc, sizeof payload);
    vo.t_us = 12345678;
    vo.velocity_body[0] = 1.25;
    vo.velocity_body[1] = -2.5;
    vo.covariance[0] = 4; vo.covariance[1] = 1;
    vo.covariance[3] = 1; vo.covariance[4] = 9;
    vo.source = NAV_WIRE_SOURCE_VISION;
    vo.dimensions = 2;
    vo.frame = NAV_WIRE_VELOCITY_NED;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_OK);
    CHECK(length == 107 && payload[106] == NAV_WIRE_VELOCITY_NED);
    /* 手工补独立参考尾字段，以便编码和解码错误不会互相抵消。 */
    payload[106] = NAV_WIRE_VELOCITY_NED;
    CHECK(nav_decode_vo_velocity(payload, sizeof payload, &decoded) == NAV_PROTOCOL_OK);
    CHECK(decoded.frame == NAV_WIRE_VELOCITY_NED && decoded.dimensions == 2 && decoded.source == NAV_WIRE_SOURCE_VISION);
    CHECK(decoded.t_us == vo.t_us && decoded.velocity_body[0] == 1.25 && decoded.velocity_body[1] == -2.5);
    CHECK(decoded.covariance[1] == 1 && decoded.covariance[3] == 1 && decoded.covariance[8] == 0);
    memcpy(original, payload, sizeof payload);
    vo.frame = 2;
    length = 12345;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    CHECK(length == 12345 && memcmp(payload, original, sizeof payload) == 0);
    unchanged = decoded;
    payload[106] = 2;
    CHECK(nav_decode_vo_velocity(payload, sizeof payload, &decoded) == NAV_PROTOCOL_BAD_VALUE);
    CHECK(memcmp(&decoded, &unchanged, sizeof decoded) == 0);
}

#define CHECK_CODEC_ARGUMENTS(name, type, payload_size) do { \
    type msg; \
    uint8_t bytes[payload_size]; \
    size_t length = 12345; \
    memset(&msg, 0, sizeof msg); \
    memset(bytes, 0x33, sizeof bytes); \
    CHECK(nav_encode_##name(&msg, NULL, sizeof bytes, &length) == NAV_PROTOCOL_BAD_ARGUMENT); \
    CHECK(nav_encode_##name(&msg, bytes, sizeof bytes, NULL) == NAV_PROTOCOL_BAD_ARGUMENT); \
    CHECK(nav_encode_##name(&msg, bytes, sizeof bytes - 1, &length) == NAV_PROTOCOL_BAD_LENGTH); \
    CHECK(length == 12345 && bytes[0] == 0x33 && bytes[sizeof bytes - 1] == 0x33); \
    CHECK(nav_decode_##name(NULL, sizeof bytes, &msg) == NAV_PROTOCOL_BAD_ARGUMENT); \
    CHECK(nav_decode_##name(bytes, sizeof bytes, NULL) == NAV_PROTOCOL_BAD_ARGUMENT); \
} while (0)

static void test_all_codec_arguments_and_saturating_counters(void)
{
    nav_parser_t parser;
    nav_frame_t frame;
    uint8_t wire[93];
    size_t length = 0;
    CHECK_CODEC_ARGUMENTS(nav_output, nav_wire_output_t, NAV_PAYLOAD_NAV_OUTPUT);
    CHECK_CODEC_ARGUMENTS(time_sync, nav_wire_time_sync_t, NAV_PAYLOAD_TIME_SYNC);
    CHECK_CODEC_ARGUMENTS(cam_trigger, nav_wire_cam_trigger_t, NAV_PAYLOAD_CAM_TRIGGER);
    CHECK_CODEC_ARGUMENTS(nav_prior, nav_wire_prior_t, NAV_PAYLOAD_NAV_PRIOR);
    CHECK_CODEC_ARGUMENTS(vo_velocity, nav_wire_vo_velocity_t, NAV_PAYLOAD_VO_VELOCITY);
    CHECK_CODEC_ARGUMENTS(position_fix, nav_wire_position_fix_t, NAV_PAYLOAD_POSITION_FIX);
    CHECK_CODEC_ARGUMENTS(health, nav_wire_health_t, NAV_PAYLOAD_HEALTH);
    CHECK(nav_parser_next(NULL, &frame) == NAV_PARSE_BAD_ARGUMENT);
    CHECK(nav_crc16(NULL, 1) == 0);
    nav_parser_init(NULL);
    nav_parser_reset_stream(NULL);
    nav_parser_init(&parser);
    parser.discarded_bytes = UINT32_MAX;
    parser.frames_ok = UINT32_MAX;
    parser.crc_errors = UINT32_MAX;
    parser.length_errors = UINT32_MAX;
    CHECK(nav_parser_feed(&parser, 0, &frame) == NAV_PARSE_INCOMPLETE);
    CHECK(parser.discarded_bytes == UINT32_MAX);
    CHECK(nav_frame_encode(NAV_MSG_NAV_OUTPUT, 0, output_golden, sizeof output_golden, wire, sizeof wire, &length) == NAV_PROTOCOL_OK);
    CHECK(feed_bytes(&parser, wire, length, &frame) == 1 && parser.frames_ok == UINT32_MAX);
    wire[92] ^= 1;
    CHECK(feed_bytes(&parser, wire, length, &frame) == 0 && parser.crc_errors == UINT32_MAX);
    wire[3] = 84;
    CHECK(feed_bytes(&parser, wire, length, &frame) == 0 && parser.length_errors == UINT32_MAX);
    nav_parser_reset_stream(&parser);
    CHECK(parser.discarded_bytes == UINT32_MAX);
    nav_parser_init(&parser);
    CHECK(parser.frames_ok == 0 && parser.discarded_bytes == 0 && parser.crc_errors == 0 && parser.length_errors == 0);
}

static void test_argument_and_value_rejection(void)
{
    nav_wire_output_t output = sample_output();
    nav_wire_output_t decoded = sample_output();
    nav_wire_output_t unchanged = decoded;
    nav_wire_time_sync_t time_msg = {0, 0, 2};
    nav_wire_vo_velocity_t vo;
    nav_wire_position_fix_t pos;
    nav_wire_health_t health = {0, 0, 0, 0, 0, 0, 0};
    nav_wire_cam_trigger_t cam = {0, 0, 0, 0};
    nav_wire_prior_t prior;
    uint8_t payload[255];
    uint8_t original[255];
    size_t length = 777;
    memset(payload, 0x6e, sizeof payload);
    memcpy(original, payload, sizeof payload);
    memset(&vo, 0, sizeof vo);
    vo.dimensions = 3;
    vo.source = NAV_WIRE_SOURCE_VISION;
    memcpy(vo.covariance, covariance, sizeof covariance);
    memset(&pos, 0, sizeof pos);
    pos.dimensions = 3;
    memcpy(pos.covariance, covariance, sizeof covariance);
    memset(&prior, 0, sizeof prior);
    prior.quaternion[0] = 1;
    CHECK(nav_frame_encode(0x7e, 0, payload, 256, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_frame_encode(NAV_MSG_NAV_OUTPUT, 0, payload, 84, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_frame_encode(0x7e, 0, payload, 1, payload, 8, &length) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_frame_encode(0x7e, 0, NULL, 1, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_ARGUMENT);
    CHECK(length == 777 && memcmp(payload, original, sizeof payload) == 0);
    CHECK(nav_encode_nav_output(&output, payload, 84, &length) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_encode_nav_output(NULL, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_ARGUMENT);
    CHECK(nav_decode_nav_output(output_golden, 84, &decoded) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(memcmp(&decoded, &unchanged, sizeof decoded) == 0);
    output.lat_deg = 91;
    CHECK(nav_encode_nav_output(&output, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    output = sample_output(); output.lon_deg = -181;
    CHECK(nav_encode_nav_output(&output, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    output = sample_output(); output.height_m = INFINITY;
    CHECK(nav_encode_nav_output(&output, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    output = sample_output(); output.vel_ned[2] = NAN;
    CHECK(nav_encode_nav_output(&output, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    output = sample_output(); output.h_sigma = -1;
    CHECK(nav_encode_nav_output(&output, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    output = sample_output(); output.mode = 5;
    CHECK(nav_encode_nav_output(&output, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    output = sample_output(); memset(output.quaternion, 0, sizeof output.quaternion);
    CHECK(nav_encode_nav_output(&output, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    CHECK(nav_encode_time_sync(&time_msg, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    CHECK(length == 777 && memcmp(payload, original, sizeof payload) == 0);
    memcpy(payload, output_golden, sizeof output_golden);
    /* IEEE-754 NaN 写入纬度，CRC 正确也不能变成可用导航量测。 */
    memset(payload + 8, 0, 8); payload[14] = 0xf8; payload[15] = 0x7f;
    CHECK(nav_decode_nav_output(payload, sizeof output_golden, &decoded) == NAV_PROTOCOL_BAD_VALUE);
    CHECK(memcmp(&decoded, &unchanged, sizeof decoded) == 0);
    vo.covariance[1] = 2;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(vo.covariance, covariance, sizeof covariance); vo.covariance[4] = 0;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(vo.covariance, covariance, sizeof covariance); vo.covariance[1] = 10; vo.covariance[3] = 10;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(vo.covariance, covariance, sizeof covariance); vo.covariance[8] = NAN;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    memcpy(vo.covariance, covariance, sizeof covariance); vo.dimensions = 0;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    vo.dimensions = 4;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    vo.dimensions = 3; vo.source = 5;
    CHECK(nav_encode_vo_velocity(&vo, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    pos.lat_deg = NAN;
    CHECK(nav_encode_position_fix(&pos, payload, sizeof payload, &length) == NAV_PROTOCOL_BAD_VALUE);
    CHECK(nav_decode_time_sync(time_golden, sizeof time_golden - 1, &time_msg) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_decode_cam_trigger(cam_golden, sizeof cam_golden + 1, &cam) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_decode_nav_prior(output_golden, NAV_PAYLOAD_NAV_PRIOR - 1, &prior) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_decode_vo_velocity(payload, NAV_PAYLOAD_VO_VELOCITY - 1, &vo) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_decode_position_fix(payload, NAV_PAYLOAD_POSITION_FIX - 1, &pos) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_decode_health(payload, NAV_PAYLOAD_HEALTH - 1, &health) == NAV_PROTOCOL_BAD_LENGTH);
    CHECK(nav_parser_feed(NULL, 0, NULL) == NAV_PARSE_BAD_ARGUMENT);
    CHECK(nav_message_payload_size(NAV_MSG_NAV_OUTPUT) == 85);
    CHECK(nav_message_payload_size(NAV_MSG_TIME_SYNC) == 13);
    CHECK(nav_message_payload_size(NAV_MSG_CAM_TRIGGER) == 17);
    CHECK(nav_message_payload_size(NAV_MSG_NAV_PRIOR) == 69);
    CHECK(nav_message_payload_size(NAV_MSG_VO_VELOCITY) == 107);
    CHECK(nav_message_payload_size(NAV_MSG_POSITION_FIX) == 106);
    CHECK(nav_message_payload_size(NAV_MSG_HEALTH) == 29);
    CHECK(nav_message_payload_size(0x7e) == 0);
}

int main(void)
{
    test_crc_and_frame_golden();
    test_output_layout();
    test_time_camera_prior_health();
    test_measurement_layout();
    test_parser_stream();
    test_maximum_and_overlap();
    test_preserves_embedded_complete_frame();
    test_drains_recovered_frames();
    test_corruption_and_truncation_sweep();
    test_wire_numeric_rejection_and_partial_dimensions();
    test_vo_ned_2d_and_unknown_frame();
    test_all_codec_arguments_and_saturating_counters();
    test_argument_and_value_rejection();
    if (failures) {
        fprintf(stderr, "protocol: %d failed / %u checks\n", failures, checks);
        return 1;
    }
    printf("protocol: %u checks passed\n", checks);
    return 0;
}

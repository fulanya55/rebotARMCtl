/*
 * Behaviour tests for the controller core, run against the simulated plant in
 * virtual time. Each test drives the real byte protocol through
 * rac_core_on_host_bytes and judges what the simulated motors actually did.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "drv_fashionstar.h"
#include "drv_robstride.h"
#include "rac_core.h"
#include "rac_wire.h"
#include "sim_plant.h"

#define J RAC_JOINTS

static int failures;
static int checks;
#define CHECK(cond)                                                       \
    do {                                                                  \
        checks++;                                                         \
        if (!(cond)) {                                                    \
            failures++;                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                 \
    } while (0)
#define CHECK_NEAR(a, b, tol)                                             \
    do {                                                                  \
        checks++;                                                         \
        if (fabs((double)(a) - (double)(b)) > (double)(tol)) {            \
            failures++;                                                   \
            printf("  FAIL %s:%d: %s=%f, expected %f within %f\n", __FILE__, __LINE__, #a, \
                   (double)(a), (double)(b), (double)(tol));              \
        }                                                                 \
    } while (0)

/* ---------------------------------------------------------- harness */

static uint64_t now_us;
static uint32_t seq;
static uint32_t generation;

#define MAX_RX 8192
static rac_deframer_t rx_deframer;
static uint8_t rx_payload[RAC_WIRE_MAX_PAYLOAD + 2];

static rac_ack_t last_ack;
static int ack_count;
static int event_counts[32];
static rac_arm_state_t last_state[RAC_MAX_ARMS];
static uint32_t state_count[RAC_MAX_ARMS];
static uint32_t state_gaps[RAC_MAX_ARMS];
static uint32_t last_mcu_generation;
static rac_controller_status_t last_status;
static rac_port_status_t last_port[RAC_MAX_PORTS];
static rac_device_info_t last_info;
static rac_time_sync_reply_t last_sync;

static void sink(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        size_t n = rac_deframer_push(&rx_deframer, data[i], rx_payload, sizeof(rx_payload));
        if (!n) continue;
        rac_header_t h;
        memcpy(&h, rx_payload, sizeof(h));
        last_mcu_generation = h.generation;
        const uint8_t *body = rx_payload + sizeof(h);
        switch (h.msg_type) {
        case RAC_MSG_ACK:
            memcpy(&last_ack, body, sizeof(last_ack));
            ack_count++;
            break;
        case RAC_MSG_EVENT: {
            rac_event_t e;
            memcpy(&e, body, sizeof(e));
            if (e.code < 32) event_counts[e.code]++;
            break;
        }
        case RAC_MSG_ARM_STATE:
            if (h.arm_id < RAC_MAX_ARMS) {
                rac_arm_state_t s;
                memcpy(&s, body, sizeof(s));
                if (state_count[h.arm_id] && s.sample_seq != last_state[h.arm_id].sample_seq + 1) {
                    state_gaps[h.arm_id]++;
                }
                last_state[h.arm_id] = s;
                state_count[h.arm_id]++;
            }
            break;
        case RAC_MSG_CONTROLLER_STATUS:
            memcpy(&last_status, body, sizeof(last_status));
            break;
        case RAC_MSG_PORT_STATUS: {
            rac_port_status_t p;
            memcpy(&p, body, sizeof(p));
            if (p.port < RAC_MAX_PORTS) last_port[p.port] = p;
            break;
        }
        case RAC_MSG_DEVICE_INFO:
            memcpy(&last_info, body, sizeof(last_info));
            break;
        case RAC_MSG_TIME_SYNC_REPLY:
            memcpy(&last_sync, body, sizeof(last_sync));
            break;
        default:
            break;
        }
    }
}

static void send_msg(uint8_t type, uint8_t arm, const void *body, size_t body_len)
{
    static uint8_t payload[RAC_WIRE_MAX_PAYLOAD];
    static uint8_t frame[RAC_WIRE_MAX_FRAME];
    rac_header_t h;
    memset(&h, 0, sizeof(h));
    h.version = RAC_PROTO_VERSION;
    h.msg_type = type;
    h.arm_id = arm;
    h.seq = ++seq;
    h.generation = generation;
    h.timestamp_us = now_us;
    memcpy(payload, &h, sizeof(h));
    if (body_len) memcpy(payload + sizeof(h), body, body_len);
    size_t n = rac_frame_build(payload, sizeof(h) + body_len, frame, sizeof(frame));
    rac_core_on_host_bytes(frame, n, now_us);
}

static void run_ms(uint32_t ms)
{
    uint64_t end = now_us + (uint64_t)ms * 1000u;
    uint32_t heartbeat = 0;
    while (now_us < end) {
        now_us += RAC_TICK_US;
        sim_advance(now_us);
        rac_core_tick(now_us);
        if (++heartbeat == 200) {
            heartbeat = 0;
            send_msg(RAC_MSG_HEARTBEAT, RAC_ARM_NONE, NULL, 0);
        }
    }
}

static void run_ms_silent(uint32_t ms)
{
    uint64_t end = now_us + (uint64_t)ms * 1000u;
    while (now_us < end) {
        now_us += RAC_TICK_US;
        sim_advance(now_us);
        rac_core_tick(now_us);
    }
}

static void setup(const char *name, uint8_t m0, uint8_t m1, uint8_t m2, uint8_t m3)
{
    printf("%s\n", name);
    now_us = 0;
    seq = 0;
    generation = 0;
    memset(event_counts, 0, sizeof(event_counts));
    memset(state_count, 0, sizeof(state_count));
    memset(state_gaps, 0, sizeof(state_gaps));
    ack_count = 0;
    rac_deframer_init(&rx_deframer);
    sim_init();
    sim_set_host_sink(sink);
    sim_set_model(0, m0);
    sim_set_model(1, m1);
    sim_set_model(2, m2);
    sim_set_model(3, m3);
    rac_core_init();
}

static void bind(uint8_t port, uint8_t arm, uint8_t model)
{
    rac_bind_port_t b;
    memset(&b, 0, sizeof(b));
    b.port = port;
    b.arm_id = arm;
    b.model = model;
    b.can_dialect = RAC_CAN_DIALECT_RS_PRIVATE;
    b.baudrate = 1000000u;
    send_msg(RAC_MSG_BIND_PORT, RAC_ARM_NONE, &b, sizeof(b));
}

static void pairing(uint8_t pair, uint8_t leader, uint8_t follower, float scale)
{
    rac_set_pairing_t p;
    memset(&p, 0, sizeof(p));
    p.pair = pair;
    p.leader_arm = leader;
    p.follower_arm = follower;
    p.enabled = 1;
    p.gripper_link = 1;
    for (uint8_t j = 0; j < J; j++) {
        p.source_joint[j] = j;
        p.scale[j] = scale;
    }
    send_msg(RAC_MSG_SET_PAIRING, RAC_ARM_NONE, &p, sizeof(p));
}

static void set_mode(uint8_t mode, uint8_t mask, uint8_t source)
{
    rac_set_mode_t m;
    memset(&m, 0, sizeof(m));
    m.mode = mode;
    m.arm_mask = mask;
    m.source = source;
    send_msg(RAC_MSG_SET_MODE, RAC_ARM_NONE, &m, sizeof(m));
}

static void teleop(uint8_t action, uint8_t follow, uint8_t mask)
{
    rac_teleop_control_t t;
    memset(&t, 0, sizeof(t));
    t.action = action;
    t.follow_mode = follow;
    t.pair_mask = mask;
    t.align_speed = 0.8f;
    send_msg(RAC_MSG_TELEOP_CONTROL, RAC_ARM_NONE, &t, sizeof(t));
}

static void hand_pose(uint8_t port, const float *pose)
{
    rac_sil_inject_t m;
    memset(&m, 0, sizeof(m));
    m.op = RAC_SIL_SET_HAND_POSE;
    m.arg = port;
    memcpy(m.values, pose, sizeof(m.values));
    sim_inject(&m);
}

static void inject(uint8_t op, uint8_t arg, uint8_t arg2)
{
    rac_sil_inject_t m;
    memset(&m, 0, sizeof(m));
    m.op = op;
    m.arg = arg;
    m.arg2 = arg2;
    sim_inject(&m);
}

/* Send a chunk that walks every joint linearly from `from` to `to`. */
static uint8_t send_ramp(uint8_t arm, uint32_t exec_seq, float from, float to, uint16_t points, uint32_t dt_us)
{
    static uint8_t body[sizeof(rac_target_chunk_t) + RAC_MAX_CHUNK_POINTS * J * sizeof(float)];
    rac_target_chunk_t c;
    memset(&c, 0, sizeof(c));
    c.exec_seq = exec_seq;
    c.dt_us = dt_us;
    c.point_count = points;
    c.joint_count = J;
    c.source = RAC_SOURCE_POLICY;
    memcpy(body, &c, sizeof(c));
    float *q = (float *)(void *)(body + sizeof(c));
    for (uint16_t i = 0; i < points; i++) {
        float a = points > 1 ? (float)i / (float)(points - 1) : 1.0f;
        for (int j = 0; j < J; j++) {
            float v = from + a * (to - from);
            q[i * J + j] = j == J - 1 ? 0.5f : v;
        }
    }
    send_msg(RAC_MSG_TARGET_CHUNK, arm, body, sizeof(c) + points * J * sizeof(float));
    return last_ack.status;
}

/* ------------------------------------------------------------ tests */

static void test_wire(void)
{
    printf("test_wire\n");
    uint8_t src[300], enc[320], dec[320];
    for (int i = 0; i < 300; i++) src[i] = (uint8_t)(i % 7 == 0 ? 0 : i);
    size_t e = rac_cobs_encode(src, sizeof(src), enc);
    for (size_t i = 0; i < e; i++) CHECK(enc[i] != 0);
    size_t d = rac_cobs_decode(enc, e, dec, sizeof(dec));
    CHECK(d == sizeof(src));
    CHECK(memcmp(src, dec, sizeof(src)) == 0);

    /* The known check value of CRC-16/CCITT-FALSE. */
    CHECK(rac_crc16((const uint8_t *)"123456789", 9) == 0x29B1);

    /* All non-zero input is the worst case for COBS block handling. */
    memset(src, 0x55, sizeof(src));
    e = rac_cobs_encode(src, 254, enc);
    d = rac_cobs_decode(enc, e, dec, sizeof(dec));
    CHECK(d == 254);
    CHECK(memcmp(src, dec, 254) == 0);

    /* A corrupted frame is dropped and counted, and the next one still parses. */
    rac_deframer_t df;
    rac_deframer_init(&df);
    uint8_t payload[32], frame[RAC_WIRE_MAX_FRAME], out[64];
    memset(payload, 0xA5, sizeof(payload));
    size_t n = rac_frame_build(payload, sizeof(payload), frame, sizeof(frame));
    frame[3] ^= 0x40;
    size_t got = 0;
    for (size_t i = 0; i < n; i++) got = rac_deframer_push(&df, frame[i], out, sizeof(out));
    CHECK(got == 0);
    CHECK(df.errors == 1);
    n = rac_frame_build(payload, sizeof(payload), frame, sizeof(frame));
    for (size_t i = 0; i < n; i++) got = rac_deframer_push(&df, frame[i], out, sizeof(out));
    CHECK(got == sizeof(payload));
}

static void test_robstride_frames(void)
{
    printf("test_robstride_frames\n");
    rac_can_frame_t f;
    /* Operation control: comm type 1, torque in the identifier, big endian payload. */
    rs_encode_motion(&f, RAC_CAN_DIALECT_RS_PRIVATE, RS_TYPE_RS00, 5, 0.0f, 0.0f, 250.0f, 2.5f, 0.0f);
    CHECK(f.extended == 1);
    CHECK(((f.id >> 24) & 0x1F) == RS_COMM_MOTION);
    CHECK((f.id & 0xFF) == 5);
    CHECK_NEAR((f.id >> 8) & 0xFFFF, 32768, 1);
    CHECK_NEAR(((uint32_t)f.data[0] << 8) | f.data[1], 32768, 1);
    CHECK_NEAR(((uint32_t)f.data[4] << 8) | f.data[5], 32768, 1);

    /* Round trip through a feedback frame built the way a motor builds it. */
    rac_can_frame_t fb;
    memset(&fb, 0, sizeof(fb));
    fb.extended = 1;
    fb.dlc = 8;
    uint32_t field = 3u | (2u << 14) | (1u << 8);
    fb.id = (RS_COMM_FEEDBACK << 24) | (field << 8) | RS_DEFAULT_HOST_ID;
    uint32_t p = rs_float_to_uint(1.25f, -12.57f, 12.57f, 16);
    fb.data[0] = (uint8_t)(p >> 8);
    fb.data[1] = (uint8_t)p;
    fb.data[6] = 0x01;
    fb.data[7] = 0x2C;
    rs_feedback_t out;
    extern rs_motor_type_t rac_test_type(uint8_t);
    CHECK(rs_decode_feedback(&fb, RAC_CAN_DIALECT_RS_PRIVATE, RS_DEFAULT_HOST_ID, rac_test_type, &out));
    CHECK(out.motor_id == 3);
    CHECK(out.fault_bits == 1);
    CHECK(out.mode_status == 2);
    CHECK_NEAR(out.position, 1.25f, 0.001f);
    CHECK_NEAR(out.temperature, 30.0f, 0.01f);

    /* A frame for another host is not ours. */
    CHECK(!rs_decode_feedback(&fb, RAC_CAN_DIALECT_RS_PRIVATE, 0x10, rac_test_type, &out));

    /* MIT dialect packs 12 bit fields across byte boundaries. */
    rs_encode_motion(&f, RAC_CAN_DIALECT_RS_MIT, RS_TYPE_RS00, 2, 0.0f, 0.0f, 0.0f, 5.0f, 0.0f);
    CHECK(f.extended == 0);
    CHECK(f.id == 2);
    uint32_t kd = ((uint32_t)f.data[5] << 4) | (f.data[6] >> 4);
    CHECK(kd == 4095);
}

rs_motor_type_t rac_test_type(uint8_t id)
{
    (void)id;
    return RS_TYPE_RS00;
}

static void test_fashionstar_frames(void)
{
    printf("test_fashionstar_frames\n");
    uint8_t b[FS_MAX_FRAME];
    /* Ping id 0: 12 4C 01 01 00 and the additive checksum. */
    size_t n = fs_encode_ping(b, 0);
    CHECK(n == 6);
    CHECK(b[0] == 0x12 && b[1] == 0x4C && b[2] == 0x01 && b[3] == 0x01 && b[4] == 0x00);
    CHECK(b[5] == (uint8_t)(0x12 + 0x4C + 0x01 + 0x01));

    /* Sync monitor carries: inner code, record size, count, ids. */
    static const uint8_t ids[3] = {0, 1, 2};
    n = fs_encode_sync_monitor(b, ids, 3);
    CHECK(b[2] == FS_CODE_SYNC);
    CHECK(b[3] == 6);
    CHECK(b[4] == FS_CODE_MONITOR && b[5] == 1 && b[6] == 3);
    CHECK(n == 11);

    /* Sync angle, the same bytes as struct.pack("<BlLHHH", id, tenths, 100, 50, 50, 0)
     * in Fashion Star's control script, sent as command 14 with 15 byte records. */
    float deg[1] = {-12.5f};
    static const uint8_t one[1] = {4};
    n = fs_encode_sync_angle(b, one, deg, 1, 100, 50, 50, 0);
    CHECK(n == 23);
    CHECK(b[4] == 14 && b[5] == 15 && b[6] == 1 && b[7] == 4);
    int32_t tenths = (int32_t)((uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16) | ((uint32_t)b[11] << 24));
    CHECK(tenths == -125);
    CHECK(b[12] == 100 && b[13] == 0 && b[14] == 0 && b[15] == 0);
    CHECK(b[16] == 50 && b[17] == 0 && b[18] == 50 && b[19] == 0);
    CHECK(b[20] == 0 && b[21] == 0);

    /* Reset the turn count on every servo: command 17, as the C SDK sends it. */
    n = fs_encode_reset_turns(b, FS_BROADCAST_ID);
    CHECK(n == 6 && b[2] == 17 && b[3] == 1 && b[4] == 0xFF);

    /* A monitor response parses through the byte parser, one byte at a time. */
    fs_parser_t parser;
    fs_parser_reset(&parser);
    parser.checksum_errors = 0;
    n = fs_build_monitor_response(b, 3, 45.0f);
    const uint8_t *frame = NULL;
    for (size_t i = 0; i < n; i++) frame = fs_parser_push(&parser, b[i]);
    CHECK(frame != NULL);
    fs_monitor_t m;
    CHECK(frame && fs_decode_monitor(frame, &m));
    CHECK(m.servo_id == 3);
    CHECK_NEAR(m.angle_deg, 45.0f, 0.01f);
    CHECK(m.angle_valid);

    /* Corruption is counted, and does not poison the following frame. */
    b[6] ^= 0xFF;
    frame = NULL;
    for (size_t i = 0; i < n; i++) frame = fs_parser_push(&parser, b[i]);
    CHECK(frame == NULL);
    CHECK(parser.checksum_errors == 1);
    n = fs_build_monitor_response(b, 2, -3.0f);
    for (size_t i = 0; i < n; i++) frame = fs_parser_push(&parser, b[i]);
    CHECK(frame != NULL);

    /* The sentinel the servo uses for an invalid reading. */
    n = fs_build_monitor_response(b, 1, 0.0f);
    int32_t bad = FS_MONITOR_INVALID_ANGLE;
    memcpy(b + 14, &bad, 4);
    b[n - 1] = fs_checksum(b, n - 1);
    frame = NULL;
    for (size_t i = 0; i < n; i++) frame = fs_parser_push(&parser, b[i]);
    CHECK(frame && fs_decode_monitor(frame, &m) && !m.angle_valid);
}

static void test_discover_is_read_only(void)
{
    setup("test_discover_is_read_only", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_STAR_102_HD,
          RAC_MODEL_STAR_102_FL);
    send_msg(RAC_MSG_HELLO, RAC_ARM_NONE, NULL, 0);
    run_ms(5);
    CHECK(last_info.protocol == RAC_PROTO_VERSION);
    CHECK(last_info.can_ports == 2 && last_info.uart_ports == 2);

    send_msg(RAC_MSG_DISCOVER, RAC_ARM_NONE, NULL, 0);
    run_ms(100);
    CHECK(last_port[0].detected_model == RAC_MODEL_REBOT_B601_RS);
    CHECK(last_port[0].online_mask == 0x7F);
    CHECK(last_port[0].can_dialect == RAC_CAN_DIALECT_RS_PRIVATE);
    CHECK(last_port[1].detected_model == RAC_MODEL_NONE);
    CHECK(last_port[2].detected_model == RAC_MODEL_STAR_102_HD); /* the button board answered */
    CHECK(last_port[3].detected_model == RAC_MODEL_STAR_102_UNKNOWN); /* the user confirms FL or LD */

    /* Nothing was energized: bound to an arm the motors stay exactly where they were. */
    rac_arm_debug_t a;
    rac_core_debug_arm(0, &a);
    CHECK(a.torque_on == 0);
    CHECK_NEAR(sim_motor_position(0, 1), 0.0f, 1e-6f);
}

static void test_bind_rejects_incapable_follower(void)
{
    setup("test_bind_rejects_incapable_follower", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_REBOT_B601_RS,
          RAC_MODEL_STAR_102_LD, RAC_MODEL_STAR_102_FL);
    run_ms(5);
    bind(2, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_STAR_102_LD);
    CHECK(last_ack.status == RAC_ACK_UNSUPPORTED);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_LD);
    CHECK(last_ack.status == RAC_ACK_OK);
    bind(3, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_STAR_102_FL);
    CHECK(last_ack.status == RAC_ACK_OK);
    /* A protocol mismatch between port kind and arm model. */
    bind(0, RAC_ARM_RIGHT_LEADER, RAC_MODEL_STAR_102_HD);
    CHECK(last_ack.status == RAC_ACK_UNSUPPORTED);
}

/* Cross-model pair: a Star HD leader drives an RS follower on the controller alone. */
static void test_local_teleop_cross_model(void)
{
    setup("test_local_teleop_cross_model", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_STAR_102_HD,
          RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    CHECK(last_ack.status == RAC_ACK_OK);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_HD);
    CHECK(last_ack.status == RAC_ACK_OK);
    pairing(0, RAC_ARM_LEFT_LEADER, RAC_ARM_LEFT_FOLLOWER, 1.0f);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(150);

    float pose[J] = {0.4f, 0.5f, 0.6f, -0.3f, 0.2f, 0.1f, 0.5f};
    hand_pose(2, pose);
    run_ms(2000);

    generation = 1;
    set_mode(RAC_MODE_LOCAL_TELEOP, (1u << RAC_ARM_LEFT_LEADER) | (1u << RAC_ARM_LEFT_FOLLOWER), RAC_SOURCE_TELEOP);
    CHECK(last_ack.status == RAC_ACK_OK);

    /* The follower has not moved yet: being in the mode is not following. */
    run_ms(100);
    CHECK_NEAR(sim_motor_position(0, 0), 0.0f, 0.02f);

    teleop(RAC_TELEOP_ALIGN, RAC_FOLLOW_ABSOLUTE, 1);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(4000);
    CHECK(event_counts[RAC_EVT_ALIGN_DONE] == 1);
    CHECK(last_status.pair_phase[0] == RAC_PAIR_ALIGNED);
    CHECK_NEAR(sim_motor_position(0, 0), 0.4f, 0.03f);
    CHECK_NEAR(sim_motor_position(0, 3), -0.3f, 0.03f);

    teleop(RAC_TELEOP_START, RAC_FOLLOW_ABSOLUTE, 1);
    CHECK(last_status.pair_phase[0] == RAC_PAIR_ALIGNED || last_ack.status == RAC_ACK_OK);

    /* Moving the leader moves the follower. Gravity is real in the plant. */
    pose[0] = -0.2f;
    hand_pose(2, pose);
    run_ms(3000);
    CHECK_NEAR(sim_motor_position(0, 0), -0.2f, 0.03f);
    CHECK(last_state[RAC_ARM_LEFT_FOLLOWER].flags & RAC_STATE_TORQUE_ON);

    /* Pause holds the follower while the leader keeps moving. */
    teleop(RAC_TELEOP_PAUSE, RAC_FOLLOW_ABSOLUTE, 1);
    pose[0] = 0.5f;
    hand_pose(2, pose);
    run_ms(1000);
    CHECK_NEAR(sim_motor_position(0, 0), -0.2f, 0.03f);

    /* Resume converges on the moved leader. */
    teleop(RAC_TELEOP_RESUME, RAC_FOLLOW_ABSOLUTE, 1);
    run_ms(3000);
    CHECK_NEAR(sim_motor_position(0, 0), 0.5f, 0.03f);

    /* The status reports a real synchronisation period on the service grid. */
    run_ms(100);
    CHECK_NEAR(last_status.sync_period_us[0], 4000, 500);
    CHECK(last_status.sync_latency_us[0] <= 20000);
}

/* Relative following keeps the offset without touching the calibration. */
static void test_relative_follow(void)
{
    setup("test_relative_follow", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_STAR_102_HD, RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_HD);
    pairing(0, RAC_ARM_LEFT_LEADER, RAC_ARM_LEFT_FOLLOWER, 1.0f);
    float pose[J] = {0.5f, 0, 0, 0, 0, 0, 0};
    hand_pose(2, pose);
    run_ms(2000);

    generation = 1;
    set_mode(RAC_MODE_LOCAL_TELEOP, 0x3, RAC_SOURCE_INTERVENTION);
    run_ms(50);

    /* No alignment: the follower is at 0 while the leader is at 0.5. */
    teleop(RAC_TELEOP_START, RAC_FOLLOW_RELATIVE, 1);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(500);
    CHECK_NEAR(sim_motor_position(0, 0), 0.0f, 0.03f); /* no jump to the leader's pose */
    CHECK_NEAR(last_status.rel_offset[0], -0.5f, 0.02f);

    pose[0] = 0.8f;
    hand_pose(2, pose);
    run_ms(2500);
    CHECK_NEAR(sim_motor_position(0, 0), 0.3f, 0.04f); /* moved by the same delta */

    /* The calibration is untouched: reported position still equals motor position. */
    CHECK_NEAR(last_state[RAC_ARM_LEFT_FOLLOWER].position[0], sim_motor_position(0, 0), 0.01f);
}

static void test_host_target_chunks(void)
{
    setup("test_host_target_chunks", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_NONE, RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    run_ms(150);
    generation = 1;
    set_mode(RAC_MODE_HOST_TARGET, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_POLICY);
    CHECK(last_ack.status == RAC_ACK_OK);

    /* 16 points at 50 ms walking to 0.3 rad. */
    uint8_t st = send_ramp(RAC_ARM_LEFT_FOLLOWER, 100, 0.0f, 0.3f, 16, 50000);
    CHECK(st == RAC_ACK_OK);
    CHECK(last_ack.queue_free == 128 - 16);
    run_ms(1500);
    CHECK_NEAR(sim_motor_position(0, 0), 0.3f, 0.03f);
    CHECK(last_state[RAC_ARM_LEFT_FOLLOWER].exec_seq == 115);
    CHECK(event_counts[RAC_EVT_QUEUE_UNDERRUN] == 1);

    /* action in the dataset is what was dispatched, and it carries its timestamp. */
    CHECK(last_state[RAC_ARM_LEFT_FOLLOWER].flags & RAC_STATE_TARGET_VALID);
    CHECK(last_state[RAC_ARM_LEFT_FOLLOWER].target_seq > 300);
    CHECK(state_gaps[RAC_ARM_LEFT_FOLLOWER] == 0);

    /* A chunk with an older generation never executes. */
    generation = 0;
    st = send_ramp(RAC_ARM_LEFT_FOLLOWER, 200, 0.3f, 1.0f, 8, 10000);
    CHECK(st == RAC_ACK_STALE_GENERATION);
    CHECK(event_counts[RAC_EVT_STALE_REJECTED] >= 1);
    generation = 1;
    run_ms(500);
    CHECK_NEAR(sim_motor_position(0, 0), 0.3f, 0.03f);

    /* A generation the device has not switched to is refused as well. */
    generation = 9;
    st = send_ramp(RAC_ARM_LEFT_FOLLOWER, 200, 0.3f, 1.0f, 8, 10000);
    CHECK(st == RAC_ACK_STALE_GENERATION);
    generation = 1;

    /* The queue refuses what it cannot hold rather than dropping part of it. */
    for (int i = 0; i < 8; i++) {
        st = send_ramp(RAC_ARM_LEFT_FOLLOWER, 300, 0.3f, 0.3f, 16, 100000);
        CHECK(st == RAC_ACK_OK);
    }
    st = send_ramp(RAC_ARM_LEFT_FOLLOWER, 400, 0.3f, 0.3f, 16, 100000);
    CHECK(st == RAC_ACK_QUEUE_FULL);

    /* Pause freezes playback; resume continues without a jump. */
    rac_pause_resume_t pr;
    memset(&pr, 0, sizeof(pr));
    pr.pause = 1;
    pr.arm_mask = 1u << RAC_ARM_LEFT_FOLLOWER;
    send_msg(RAC_MSG_PAUSE_RESUME, RAC_ARM_NONE, &pr, sizeof(pr));
    uint32_t exec_before = last_state[RAC_ARM_LEFT_FOLLOWER].exec_seq;
    run_ms(500);
    CHECK(last_state[RAC_ARM_LEFT_FOLLOWER].exec_seq == exec_before);
    pr.pause = 0;
    send_msg(RAC_MSG_PAUSE_RESUME, RAC_ARM_NONE, &pr, sizeof(pr));
    run_ms(500);
    CHECK(last_state[RAC_ARM_LEFT_FOLLOWER].exec_seq != exec_before);
}

/* Takeover: the old source's queued actions and late chunks must never run. */
static void test_takeover_clears_and_rejects(void)
{
    setup("test_takeover_clears_and_rejects", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_STAR_102_HD,
          RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_HD);
    pairing(0, RAC_ARM_LEFT_LEADER, RAC_ARM_LEFT_FOLLOWER, 1.0f);
    run_ms(150);

    generation = 1;
    set_mode(RAC_MODE_HOST_TARGET, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_POLICY);
    CHECK(send_ramp(RAC_ARM_LEFT_FOLLOWER, 1, 0.0f, 1.0f, 16, 200000) == RAC_ACK_OK);
    run_ms(400);
    float at_takeover = sim_motor_position(0, 0);
    CHECK(at_takeover < 0.3f);

    /* Takeover: new generation, the queue is dropped on the controller. */
    generation = 2;
    set_mode(RAC_MODE_LOCAL_TELEOP, 0x3, RAC_SOURCE_INTERVENTION);
    CHECK(last_ack.status == RAC_ACK_OK);
    CHECK(event_counts[RAC_EVT_QUEUE_CLEARED] == 1);

    /* A result from the old inference arrives late. */
    generation = 1;
    uint8_t st = send_ramp(RAC_ARM_LEFT_FOLLOWER, 50, 1.0f, 1.0f, 8, 10000);
    CHECK(st == RAC_ACK_STALE_GENERATION);
    generation = 2;

    run_ms(1500);
    /* The follower stayed where it was: nothing kept executing. */
    CHECK_NEAR(sim_motor_position(0, 0), at_takeover, 0.05f);

    /* Hand back: a third generation, and the old chunk still cannot run. */
    generation = 3;
    set_mode(RAC_MODE_HOST_TARGET, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_POLICY);
    CHECK(last_ack.status == RAC_ACK_OK);
    generation = 2;
    CHECK(send_ramp(RAC_ARM_LEFT_FOLLOWER, 60, 1.0f, 1.0f, 8, 10000) == RAC_ACK_STALE_GENERATION);
}

static void test_estop(void)
{
    setup("test_estop", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_STAR_102_LD, RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_LD);
    run_ms(150);

    generation = 1;
    set_mode(RAC_MODE_HOST_TARGET, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_POLICY);
    CHECK(send_ramp(RAC_ARM_LEFT_FOLLOWER, 1, 0.0f, 1.0f, 16, 100000) == RAC_ACK_OK);
    run_ms(300);

    inject(RAC_SIL_PRESS_ESTOP, 0, 0);
    run_ms(5);
    CHECK(event_counts[RAC_EVT_ESTOP_PRESSED] == 1);
    rac_arm_debug_t follower, leader;
    rac_core_debug_arm(RAC_ARM_LEFT_FOLLOWER, &follower);
    rac_core_debug_arm(RAC_ARM_LEFT_LEADER, &leader);
    CHECK(follower.mode == RAC_MODE_ESTOP);
    CHECK(follower.queue_depth == 0);

    /* The arm that was released was not energized by the stop. */
    CHECK(leader.torque_on == 0);

    /* The controller moved the generation itself, and it reports the new value. */
    run_ms(60);
    CHECK(last_mcu_generation == 2);

    /* While pressed: no modes, no targets, and reset is refused. */
    generation = 3;
    set_mode(RAC_MODE_HOLD, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_NONE);
    CHECK(last_ack.status == RAC_ACK_ESTOP_ACTIVE);
    send_msg(RAC_MSG_RESET_ESTOP, RAC_ARM_NONE, NULL, 0);
    CHECK(last_ack.status == RAC_ACK_ESTOP_ACTIVE);

    /* Releasing the button alone does not restart anything. */
    inject(RAC_SIL_RELEASE_ESTOP, 0, 0);
    run_ms(50);
    CHECK(last_status.estop == RAC_ESTOP_LATCHED);
    set_mode(RAC_MODE_HOLD, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_NONE);
    CHECK(last_ack.status == RAC_ACK_ESTOP_ACTIVE);

    send_msg(RAC_MSG_RESET_ESTOP, RAC_ARM_NONE, NULL, 0);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(100);
    CHECK(last_status.estop == RAC_ESTOP_CLEAR);
    rac_core_debug_arm(RAC_ARM_LEFT_FOLLOWER, &follower);
    CHECK(follower.mode == RAC_MODE_IDLE);
    CHECK(follower.torque_on == 0);
}

static void test_host_loss_stops(void)
{
    setup("test_host_loss_stops", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_NONE, RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    run_ms(150);
    generation = 1;
    set_mode(RAC_MODE_HOST_TARGET, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_POLICY);
    CHECK(send_ramp(RAC_ARM_LEFT_FOLLOWER, 1, 0.0f, 1.0f, 16, 300000) == RAC_ACK_OK);
    run_ms(200);

    /* The host goes silent while a long chunk is still queued. */
    event_counts[RAC_EVT_HOST_RESTORED] = 0; /* first contact already counted once */
    run_ms_silent(800);
    CHECK(event_counts[RAC_EVT_HOST_LOST] == 1);
    rac_arm_debug_t a;
    rac_core_debug_arm(RAC_ARM_LEFT_FOLLOWER, &a);
    CHECK(a.queue_depth == 0);
    CHECK(a.mode == RAC_MODE_HOLD);
    float held = sim_motor_position(0, 0);
    run_ms_silent(1000);
    CHECK_NEAR(sim_motor_position(0, 0), held, 0.02f);

    /* The host comes back and hears what happened. */
    send_msg(RAC_MSG_HEARTBEAT, RAC_ARM_NONE, NULL, 0);
    run_ms(10);
    CHECK(event_counts[RAC_EVT_HOST_RESTORED] == 1);
}

static void test_motor_loss_stops_pair(void)
{
    setup("test_motor_loss_stops_pair", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_STAR_102_HD, RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_HD);
    pairing(0, RAC_ARM_LEFT_LEADER, RAC_ARM_LEFT_FOLLOWER, 1.0f);
    run_ms(150);
    generation = 1;
    set_mode(RAC_MODE_LOCAL_TELEOP, 0x3, RAC_SOURCE_TELEOP);
    teleop(RAC_TELEOP_START, RAC_FOLLOW_ABSOLUTE, 1);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(200);
    CHECK(last_status.pair_phase[0] == RAC_PAIR_FOLLOWING);

    /* Lose one follower motor. */
    inject(RAC_SIL_DROP_MOTOR, 0, 1u << 3);
    run_ms(300);
    CHECK(event_counts[RAC_EVT_MOTOR_LOST] == 1);
    CHECK(last_status.pair_phase[0] == RAC_PAIR_OFF);
    rac_arm_debug_t a;
    rac_core_debug_arm(RAC_ARM_LEFT_FOLLOWER, &a);
    CHECK(a.valid_mask == (0x7F & ~(1u << 3)));
    /* A partial sample cannot be used to enter a mode that seeds from it. */
    set_mode(RAC_MODE_HOST_TARGET, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_POLICY);
    CHECK(last_ack.status == RAC_ACK_MODE_REJECTED);

    inject(RAC_SIL_RESTORE_MOTOR, 0, 1u << 3);
    run_ms(200);
    CHECK(event_counts[RAC_EVT_MOTOR_RESTORED] >= 1);
}

static void test_joint_limits_clamp(void)
{
    setup("test_joint_limits_clamp", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_NONE, RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    run_ms(150);

    rac_set_calibration_t c;
    memset(&c, 0, sizeof(c));
    c.version = 7;
    for (int j = 0; j < J; j++) {
        c.direction[j] = 1.0f;
        c.lower[j] = -0.2f;
        c.upper[j] = 0.2f;
    }
    c.lower[J - 1] = 0.0f;
    c.upper[J - 1] = 1.0f;
    c.gripper_open = 5.0f;
    send_msg(RAC_MSG_SET_CALIBRATION, RAC_ARM_LEFT_FOLLOWER, &c, sizeof(c));
    CHECK(last_ack.status == RAC_ACK_OK);

    generation = 1;
    set_mode(RAC_MODE_HOST_TARGET, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_POLICY);
    CHECK(send_ramp(RAC_ARM_LEFT_FOLLOWER, 1, 0.0f, 1.0f, 16, 20000) == RAC_ACK_OK);
    run_ms(1500);
    CHECK_NEAR(sim_motor_position(0, 0), 0.2f, 0.02f);
    CHECK(event_counts[RAC_EVT_LIMIT_CLAMPED] >= 1);

    /* Energized arms refuse a new calibration: it would move them. */
    send_msg(RAC_MSG_SET_CALIBRATION, RAC_ARM_LEFT_FOLLOWER, &c, sizeof(c));
    CHECK(last_ack.status == RAC_ACK_BUSY);
}

static void test_calibration_maps_raw(void)
{
    setup("test_calibration_maps_raw", RAC_MODEL_NONE, RAC_MODEL_NONE, RAC_MODEL_STAR_102_LD, RAC_MODEL_NONE);
    run_ms(5);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_LD);
    float pose[J] = {0.5f, 0, 0, 0, 0, 0, 0.6f};
    hand_pose(2, pose);
    run_ms(1500);

    rac_set_calibration_t c;
    memset(&c, 0, sizeof(c));
    for (int j = 0; j < J; j++) {
        c.direction[j] = 1.0f;
        c.lower[j] = -3.0f;
        c.upper[j] = 3.0f;
    }
    c.zero[0] = 0.2f;
    c.direction[0] = -1.0f;
    c.gripper_closed = 0.1f;
    c.gripper_open = 1.1f;
    send_msg(RAC_MSG_SET_CALIBRATION, RAC_ARM_LEFT_LEADER, &c, sizeof(c));
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(50);
    /* q = direction * (raw - zero) */
    CHECK_NEAR(last_state[RAC_ARM_LEFT_LEADER].position[0], -0.3f, 0.01f);
    /* gripper = (raw - closed) / (open - closed) */
    CHECK_NEAR(last_state[RAC_ARM_LEFT_LEADER].position[6], 0.5f, 0.01f);
}

/* Baseline capacity: 2 RS and 2 Star at once on one tick grid, no sample lost. */
static void test_four_arms_together(void)
{
    setup("test_four_arms_together", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_REBOT_B601_RS, RAC_MODEL_STAR_102_HD,
          RAC_MODEL_STAR_102_HD);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    bind(1, RAC_ARM_RIGHT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_HD);
    bind(3, RAC_ARM_RIGHT_LEADER, RAC_MODEL_STAR_102_HD);
    pairing(0, RAC_ARM_LEFT_LEADER, RAC_ARM_LEFT_FOLLOWER, 1.0f);
    pairing(1, RAC_ARM_RIGHT_LEADER, RAC_ARM_RIGHT_FOLLOWER, 1.0f);
    run_ms(150);

    generation = 1;
    set_mode(RAC_MODE_LOCAL_TELEOP, 0xF, RAC_SOURCE_TELEOP);
    CHECK(last_ack.status == RAC_ACK_OK);
    teleop(RAC_TELEOP_START, RAC_FOLLOW_ABSOLUTE, 0x3);
    CHECK(last_ack.status == RAC_ACK_OK);

    float pose_l[J] = {0.3f, 0, 0, 0, 0, 0, 0.5f};
    float pose_r[J] = {-0.3f, 0, 0, 0, 0, 0, 0.5f};
    hand_pose(2, pose_l);
    hand_pose(3, pose_r);

    memset(state_count, 0, sizeof(state_count));
    memset(state_gaps, 0, sizeof(state_gaps));
    uint32_t sent0 = sim_can_frames_sent(0);
    uint32_t sent1 = sim_can_frames_sent(1);
    run_ms(3000);

    /* RS at 250 Hz, Star at 125 Hz. */
    CHECK_NEAR(state_count[RAC_ARM_LEFT_FOLLOWER], 750, 10);
    CHECK_NEAR(state_count[RAC_ARM_RIGHT_FOLLOWER], 750, 10);
    CHECK_NEAR(state_count[RAC_ARM_LEFT_LEADER], 375, 10);
    CHECK_NEAR(state_count[RAC_ARM_RIGHT_LEADER], 375, 10);
    for (int i = 0; i < RAC_MAX_ARMS; i++) CHECK(state_gaps[i] == 0);
    /* Both CAN arms get exactly the same number of frames: they run in step. */
    CHECK(sim_can_frames_sent(0) - sent0 == sim_can_frames_sent(1) - sent1);

    CHECK_NEAR(sim_motor_position(0, 0), 0.3f, 0.03f);
    CHECK_NEAR(sim_motor_position(1, 0), -0.3f, 0.03f);
    CHECK(last_status.pair_phase[0] == RAC_PAIR_FOLLOWING);
    CHECK(last_status.pair_phase[1] == RAC_PAIR_FOLLOWING);
}

static void test_guide_needs_fresh_feedforward(void)
{
    setup("test_guide_needs_fresh_feedforward", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_NONE, RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_LEADER, RAC_MODEL_REBOT_B601_RS);
    run_ms(150);
    generation = 1;
    set_mode(RAC_MODE_GUIDE, 1u << RAC_ARM_LEFT_LEADER, RAC_SOURCE_GUIDE);
    CHECK(last_ack.status == RAC_ACK_OK);

    /* With gravity torque supplied, the arm stays put. */
    for (int i = 0; i < 100; i++) {
        rac_feedforward_t f;
        memset(&f, 0, sizeof(f));
        f.valid_ms = 200;
        f.torque[1] = 6.0f * cosf(sim_motor_position(0, 1));
        f.torque[2] = 3.0f * cosf(sim_motor_position(0, 2));
        send_msg(RAC_MSG_FEEDFORWARD, RAC_ARM_LEFT_LEADER, &f, sizeof(f));
        run_ms(20);
    }
    CHECK_NEAR(sim_motor_position(0, 1), 0.0f, 0.05f);

    /* When the feedforward stops, damping rises instead of the arm dropping fast. */
    run_ms(500);
    float q = sim_motor_position(0, 1);
    run_ms(100);
    float speed = fabsf(sim_motor_position(0, 1) - q) / 0.1f;
    CHECK(speed < 2.0f);

    /* Star arms have no low-impedance mode. */
    setup("  star has no guide", RAC_MODEL_NONE, RAC_MODEL_NONE, RAC_MODEL_STAR_102_FL, RAC_MODEL_NONE);
    run_ms(5);
    bind(2, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_STAR_102_FL);
    run_ms(100);
    generation = 1;
    set_mode(RAC_MODE_GUIDE, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_GUIDE);
    CHECK(last_ack.status == RAC_ACK_UNSUPPORTED);
}

static void test_star_follower_executes(void)
{
    setup("test_star_follower_executes", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_STAR_102_FL, RAC_MODEL_NONE);
    run_ms(5);
    /* The other direction: an RS leader in guide drives a Star follower. */
    bind(0, RAC_ARM_LEFT_LEADER, RAC_MODEL_REBOT_B601_RS);
    bind(2, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_STAR_102_FL);
    pairing(0, RAC_ARM_LEFT_LEADER, RAC_ARM_LEFT_FOLLOWER, 1.0f);
    run_ms(150);
    float pose[J] = {0.25f, 0, 0, 0, 0, 0, 0.5f};
    hand_pose(0, pose);
    run_ms(1500);

    generation = 1;
    set_mode(RAC_MODE_LOCAL_TELEOP, 0x3, RAC_SOURCE_TELEOP);
    CHECK(last_ack.status == RAC_ACK_OK);
    rac_arm_debug_t leader;
    rac_core_debug_arm(RAC_ARM_LEFT_LEADER, &leader);
    CHECK(leader.torque_on == 1); /* an RS leader is held in guide */

    teleop(RAC_TELEOP_START, RAC_FOLLOW_ABSOLUTE, 1);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(2000);
    CHECK_NEAR(sim_motor_position(2, 0), 0.25f, 0.04f);
}

static void test_mit_dialect(void)
{
    setup("test_mit_dialect", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_NONE, RAC_MODEL_NONE);
    rac_sil_inject_t none;
    memset(&none, 0, sizeof(none));
    run_ms(5);
    rac_bind_port_t b;
    memset(&b, 0, sizeof(b));
    b.port = 0;
    b.arm_id = RAC_ARM_LEFT_FOLLOWER;
    b.model = RAC_MODEL_REBOT_B601_RS;
    b.can_dialect = RAC_CAN_DIALECT_RS_PRIVATE;
    send_msg(RAC_MSG_BIND_PORT, RAC_ARM_NONE, &b, sizeof(b));
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(150);
    CHECK(last_state[RAC_ARM_LEFT_FOLLOWER].valid_mask == 0x7F);
}

/* Star servos count turns, and a reading on another turn is still the same pose. */
static void test_star_turn_count(void)
{
    setup("test_star_turn_count", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_STAR_102_LD, RAC_MODEL_STAR_102_FL);
    run_ms(5);

    /* A count left from an earlier session is cleared when a leader is bound. */
    sim_set_turns(2, 1, 3);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_LD);
    pairing(0, RAC_ARM_LEFT_LEADER, RAC_ARM_LEFT_FOLLOWER, 1.0f);
    run_ms(150);
    CHECK_NEAR(last_state[RAC_ARM_LEFT_LEADER].position[1], 0.0f, 0.01f);

    float pose[J] = {0.3f, 0, 0, 0, 0, 0, 0};
    hand_pose(2, pose);
    run_ms(1500);
    generation = 1;
    set_mode(RAC_MODE_LOCAL_TELEOP, (1u << RAC_ARM_LEFT_LEADER) | (1u << RAC_ARM_LEFT_FOLLOWER), RAC_SOURCE_TELEOP);
    teleop(RAC_TELEOP_ALIGN, RAC_FOLLOW_ABSOLUTE, 1);
    run_ms(3000);
    teleop(RAC_TELEOP_START, RAC_FOLLOW_ABSOLUTE, 1);
    run_ms(500);
    CHECK(last_status.pair_phase[0] == RAC_PAIR_FOLLOWING);

    /* A leader that reports another turn mid-session is read in the same place. */
    sim_set_turns(2, 0, -2);
    run_ms(500);
    CHECK_NEAR(last_state[RAC_ARM_LEFT_LEADER].position[0], 0.3f, 0.02f);
    CHECK_NEAR(sim_motor_position(0, 0), 0.3f, 0.03f);
    CHECK(last_status.pair_phase[0] == RAC_PAIR_FOLLOWING);
    CHECK(event_counts[RAC_EVT_LEADER_JUMP] == 0);

    /* A follower on another turn holds where it is, rather than spinning a full
     * turn back to the angle its reading was reduced to. */
    sim_set_turns(3, 2, 1);
    bind(3, RAC_ARM_RIGHT_FOLLOWER, RAC_MODEL_STAR_102_FL);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(150);
    CHECK_NEAR(last_state[RAC_ARM_RIGHT_FOLLOWER].position[2], 0.0f, 0.01f);
    set_mode(RAC_MODE_HOST_TARGET, 1u << RAC_ARM_RIGHT_FOLLOWER, RAC_SOURCE_TEACH);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(1000);
    CHECK(last_state[RAC_ARM_RIGHT_FOLLOWER].flags & RAC_STATE_TORQUE_ON);
    CHECK_NEAR(sim_motor_position(3, 2), 0.0f, 0.02f);
}

/* A reading that moves further than a hand can is not followed. */
static void test_leader_jump_stops_following(void)
{
    setup("test_leader_jump_stops_following", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_STAR_102_LD, RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    bind(2, RAC_ARM_LEFT_LEADER, RAC_MODEL_STAR_102_LD);
    pairing(0, RAC_ARM_LEFT_LEADER, RAC_ARM_LEFT_FOLLOWER, 1.0f);
    run_ms(150);
    generation = 1;
    set_mode(RAC_MODE_LOCAL_TELEOP, (1u << RAC_ARM_LEFT_LEADER) | (1u << RAC_ARM_LEFT_FOLLOWER), RAC_SOURCE_TELEOP);
    teleop(RAC_TELEOP_ALIGN, RAC_FOLLOW_ABSOLUTE, 1);
    run_ms(500);
    teleop(RAC_TELEOP_START, RAC_FOLLOW_ABSOLUTE, 1);
    run_ms(500);
    CHECK(last_status.pair_phase[0] == RAC_PAIR_FOLLOWING);
    float before = sim_motor_position(0, 0);

    sim_set_position(2, 0, 0.8f);
    run_ms(100);
    CHECK(event_counts[RAC_EVT_LEADER_JUMP] == 1);
    CHECK(last_status.pair_phase[0] == RAC_PAIR_OFF);

    run_ms(1000);
    CHECK_NEAR(sim_motor_position(0, 0), before, 0.03f);
    CHECK(last_state[RAC_ARM_LEFT_FOLLOWER].flags & RAC_STATE_TORQUE_ON);
}

static void test_guide_and_follow_gains(void)
{
    setup("test_guide_and_follow_gains", RAC_MODEL_REBOT_B601_RS, RAC_MODEL_REBOT_B601_RS, RAC_MODEL_NONE, RAC_MODEL_NONE);
    run_ms(5);
    bind(0, RAC_ARM_LEFT_LEADER, RAC_MODEL_REBOT_B601_RS);
    bind(1, RAC_ARM_LEFT_FOLLOWER, RAC_MODEL_REBOT_B601_RS);
    run_ms(150);

    rac_feedforward_t f;
    memset(&f, 0, sizeof(f));
    f.valid_ms = 200;
    float kp, kd;

    /* Guidance starts from position gains and fades, so there is no torque step. */
    generation = 1;
    send_msg(RAC_MSG_FEEDFORWARD, RAC_ARM_LEFT_LEADER, &f, sizeof(f));
    set_mode(RAC_MODE_GUIDE, 1u << RAC_ARM_LEFT_LEADER, RAC_SOURCE_GUIDE);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(20);
    sim_motor_gains(0, 1, &kp, &kd);
    CHECK(kp > 100.0f);
    for (int i = 0; i < 4; i++) {
        send_msg(RAC_MSG_FEEDFORWARD, RAC_ARM_LEFT_LEADER, &f, sizeof(f));
        run_ms(150);
    }
    sim_motor_gains(0, 1, &kp, &kd);
    CHECK_NEAR(kp, 1.5f, 0.1f);
    CHECK_NEAR(kd, 1.0f, 0.01f);
    sim_motor_gains(0, 6, &kp, &kd);
    CHECK_NEAR(kp, 0.0f, 0.01f);
    CHECK_NEAR(kd, 0.001f, 0.001f);

    /* Following a leader uses Seeed's softer gains. */
    pairing(0, RAC_ARM_LEFT_LEADER, RAC_ARM_LEFT_FOLLOWER, 1.0f);
    set_mode(RAC_MODE_LOCAL_TELEOP, 1u << RAC_ARM_LEFT_FOLLOWER, RAC_SOURCE_TELEOP);
    CHECK(last_ack.status == RAC_ACK_OK);
    run_ms(50);
    sim_motor_gains(1, 1, &kp, &kd);
    CHECK_NEAR(kp, 50.0f, 0.1f);
    CHECK_NEAR(kd, 5.0f, 0.01f);
    sim_motor_gains(1, 3, &kp, &kd);
    CHECK_NEAR(kp, 30.0f, 0.1f);
    CHECK_NEAR(kd, 3.0f, 0.01f);
}

static void test_time_sync(void)
{
    setup("test_time_sync", RAC_MODEL_NONE, RAC_MODEL_NONE, RAC_MODEL_NONE, RAC_MODEL_NONE);
    run_ms(100);
    send_msg(RAC_MSG_TIME_SYNC, RAC_ARM_NONE, NULL, 0);
    CHECK(last_sync.mcu_t2_us == now_us);
    CHECK(last_sync.mcu_t3_us >= last_sync.mcu_t2_us);
    CHECK(last_sync.host_t1_us == now_us);
}

int main(void)
{
    test_wire();
    test_robstride_frames();
    test_fashionstar_frames();
    test_discover_is_read_only();
    test_bind_rejects_incapable_follower();
    test_local_teleop_cross_model();
    test_relative_follow();
    test_host_target_chunks();
    test_takeover_clears_and_rejects();
    test_estop();
    test_host_loss_stops();
    test_motor_loss_stops_pair();
    test_joint_limits_clamp();
    test_calibration_maps_raw();
    test_four_arms_together();
    test_guide_needs_fresh_feedforward();
    test_star_follower_executes();
    test_star_turn_count();
    test_leader_jump_stops_following();
    test_guide_and_follow_gains();
    test_mit_dialect();
    test_time_sync();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}

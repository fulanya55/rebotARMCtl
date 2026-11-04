#include "rac_core.h"

#include <math.h>
#include <string.h>

#include "drv_fashionstar.h"
#include "drv_robstride.h"
#include "rac_wire.h"

#define PI_F 3.14159265358979f
#define J RAC_JOINTS
#define GRIPPER (RAC_JOINTS - 1)
#define ALL_JOINTS_MASK 0x7Fu

#define QUEUE_POINTS 128u
#define SAMPLE_RING 32u

#define MOTOR_TIMEOUT_US 100000u    /* a motor silent this long is offline */
#define LEADER_STALE_US 200000u     /* a leader silent this long stops its follower */
#define DISCOVER_WINDOW_US 60000u
#define STAR_RESPONSE_WINDOW_US 6000u
#define STATUS_PERIOD_US 50000u
#define PORT_STATUS_PERIOD_US 500000u
#define LIMIT_EVENT_PERIOD_US 500000u
#define DEFAULT_HOST_TIMEOUT_MS 500u

/* Alignment is judged on the commanded target. The measured pose only has to be
 * near it, because position control under gravity droops by load / kp, which on a
 * shoulder joint is several hundredths of a radian and does not go away. */
#define ALIGN_TARGET_TOLERANCE 0.01f   /* rad */
#define ALIGN_MEASURED_TOLERANCE 0.12f /* rad */

#define STAR_IDS 7u
#define STAR_BUTTON_BOARD_ID 7u

/* Gripper travel in raw motor radians, used until a calibration arrives. */
#define RS_GRIPPER_OPEN_RAD 5.0f
#define STAR_GRIPPER_OPEN_RAD 1.5708f

/* Damping applied by an e-stop or a damped stop. RS arms have no brakes, so
 * dropping torque outright would let the arm fall. */
#define RS_DAMP_KD_LARGE 4.0f
#define RS_DAMP_KD_SMALL 1.0f

/* Hand guidance: motors back-drive under a light velocity damping. */
#define RS_GUIDE_KD_LARGE 0.6f
#define RS_GUIDE_KD_SMALL 0.15f

typedef struct {
    uint64_t t_us;
    uint32_t seq;
    float q[J];
} queue_point_t;

typedef struct {
    queue_point_t points[QUEUE_POINTS];
    uint16_t head;
    uint16_t count;
    uint64_t shift_us;      /* accumulated pause time, added to every point */
    uint64_t pause_start;   /* non-zero while paused */
    uint64_t prev_t_us;
    float prev_q[J];
    uint8_t has_prev;
    uint32_t exec_seq;
    uint8_t underrun_reported;
} point_queue_t;

typedef enum {
    MOTION_NONE = 0, /* no torque */
    MOTION_POSITION, /* track target */
    MOTION_GUIDE,    /* low impedance plus feedforward */
    MOTION_DAMP,     /* velocity damping only */
} motion_kind_t;

typedef struct {
    uint8_t port; /* 0xFF when unbound */
    uint8_t model;
    uint8_t mode;
    uint8_t source;
    uint8_t torque_on;
    uint8_t motion;
    uint8_t pair; /* pair index this arm belongs to, 0xFF for none */
    uint8_t is_leader;

    rac_set_calibration_t cal;
    rac_set_limits_t limits;

    float position[J];
    float velocity[J];
    float torque[J];
    uint8_t temperature[J];
    uint8_t valid_mask;
    uint8_t fault_mask;
    uint32_t sample_seq;
    uint64_t sample_ts;
    uint8_t sample_fresh; /* set when a new sample completed, for pair mapping */

    float target[J];
    float target_vel[J];
    uint8_t target_valid;
    uint8_t target_dispatched;
    uint8_t clamped;
    uint32_t target_seq;
    uint64_t target_ts;
    uint64_t last_limit_event;

    float feedforward[J];
    uint64_t ff_expire;

    point_queue_t queue;
    uint8_t paused;
} arm_t;

typedef struct {
    uint8_t kind;
    uint8_t bus; /* CAN or UART index */
    uint8_t arm; /* RAC_ARM_NONE when unbound */
    uint8_t model;
    uint8_t detected_model;
    uint8_t dialect;
    uint8_t online_mask;
    uint8_t expected_mask;
    uint8_t cycle_mask; /* motors that answered during the current cycle */
    uint32_t baudrate;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t service_period_us;
    uint32_t read_period_us;
    uint64_t next_service;
    uint64_t next_read;
    uint64_t cycle_start;
    uint8_t cycle_open;
    uint32_t tx_bytes_window;
    uint64_t load_window_start;
    uint16_t load_permille;

    float raw_position[J];
    float raw_velocity[J];
    float raw_torque[J];
    uint8_t raw_temperature[J];
    uint8_t raw_fault[J];
    uint64_t raw_ts[J];

    fs_parser_t parser;
    uint64_t discover_until;
    uint8_t discover_mask;
    uint8_t discover_dialect;
    uint8_t discovering;
} port_t;

typedef struct {
    rac_set_pairing_t cfg;
    uint8_t configured;
    uint8_t phase;
    uint8_t follow_mode;
    float align_speed;
    float rel_offset[J];
    uint64_t last_dispatch;
    uint32_t sync_period_us;
    uint32_t sync_latency_us;
    uint32_t leader_seq_seen;
    uint8_t leader_stale;
} pair_t;

typedef struct {
    rac_header_t tx_header;
    uint32_t tx_seq;
    uint16_t device_id;
    uint32_t generation;

    arm_t arms[RAC_MAX_ARMS];
    port_t ports[RAC_MAX_PORTS];
    pair_t pairs[RAC_MAX_PAIRS];

    rac_deframer_t deframer;
    uint8_t rx_payload[RAC_WIRE_MAX_PAYLOAD + 2u];

    rac_arm_state_t sample_ring[SAMPLE_RING];
    uint8_t sample_arm[SAMPLE_RING];
    uint16_t sample_head;
    uint16_t sample_count;
    uint32_t samples_dropped;

    uint32_t upstream_us;
    uint64_t next_upstream;
    uint64_t next_status;
    uint64_t next_port_status;

    uint32_t host_timeout_ms;
    uint64_t last_host_rx;
    uint8_t host_alive;

    uint8_t estop_state;
    uint8_t estop_raw;

    uint64_t now;
    uint64_t next_tick_expected;
    uint32_t tick_count;
    uint32_t overrun_count;
    uint32_t max_tick_us;
    uint8_t booted;
    uint32_t led_counter;
} core_t;

static core_t g;

/* ------------------------------------------------------------------ helpers */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool model_can_execute(uint8_t model)
{
    return model == RAC_MODEL_REBOT_B601_RS || model == RAC_MODEL_STAR_102_FL;
}

static bool model_can_hold(uint8_t model)
{
    return model_can_execute(model) || model == RAC_MODEL_STAR_102_HD;
}

static bool model_can_guide(uint8_t model)
{
    return model == RAC_MODEL_REBOT_B601_RS;
}

static bool model_is_star(uint8_t model)
{
    return model == RAC_MODEL_STAR_102_FL || model == RAC_MODEL_STAR_102_HD ||
           model == RAC_MODEL_STAR_102_LD || model == RAC_MODEL_STAR_102_UNKNOWN;
}

static rs_motor_type_t rs_type_of_id(uint8_t motor_id)
{
    return (motor_id >= 1 && motor_id <= 3) ? RS_TYPE_RS06 : RS_TYPE_RS00;
}

/* --------------------------------------------------------------- transmit */

static void send_frame(uint8_t type, uint8_t arm_id, const void *body, size_t body_len)
{
    static uint8_t payload[RAC_WIRE_MAX_PAYLOAD];
    static uint8_t frame[RAC_WIRE_MAX_FRAME];
    rac_header_t h;

    if (sizeof(h) + body_len > sizeof(payload)) {
        return;
    }
    h.version = RAC_PROTO_VERSION;
    h.msg_type = type;
    h.flags = 0;
    h.device_id = g.device_id;
    h.arm_id = arm_id;
    h.reserved = 0;
    h.seq = g.tx_seq++;
    h.generation = g.generation;
    h.timestamp_us = rac_port_now_us();
    memcpy(payload, &h, sizeof(h));
    if (body_len) {
        memcpy(payload + sizeof(h), body, body_len);
    }
    size_t n = rac_frame_build(payload, sizeof(h) + body_len, frame, sizeof(frame));
    if (n) {
        (void)rac_port_host_send(frame, n);
    }
}

static void send_event(uint16_t code, uint8_t arm, uint8_t severity, uint32_t arg0, uint32_t arg1)
{
    rac_event_t e;
    e.event_ts_us = g.now;
    e.arg0 = arg0;
    e.arg1 = arg1;
    e.code = code;
    e.arm = arm;
    e.severity = severity;
    send_frame(RAC_MSG_EVENT, arm, &e, sizeof(e));
}

static void send_ack(const rac_header_t *req, uint8_t status, uint32_t exec_seq, uint16_t queue_free)
{
    rac_ack_t a;
    a.acked_seq = req->seq;
    a.exec_seq = exec_seq;
    a.acked_type = req->msg_type;
    a.status = status;
    a.queue_free = queue_free;
    send_frame(RAC_MSG_ACK, req->arm_id, &a, sizeof(a));
}

static void send_device_info(void)
{
    rac_device_info_t d;
    memset(&d, 0, sizeof(d));
    d.fw_major = RAC_FW_MAJOR;
    d.fw_minor = RAC_FW_MINOR;
    d.fw_patch = RAC_FW_PATCH;
    d.protocol = RAC_PROTO_VERSION;
    rac_port_unique_id(d.uid);
    d.hw_revision = RAC_HW_REVISION;
    d.can_ports = 2;
    d.uart_ports = 2;
    d.max_chunk_points = RAC_MAX_CHUNK_POINTS;
    d.capabilities = RAC_CAP_LOCAL_TELEOP | RAC_CAP_GUIDE | RAC_CAP_GRAVITY_FF | rac_port_capabilities();
    d.tick_hz = 1000000u / RAC_TICK_US;
    d.queue_points = QUEUE_POINTS;
    send_frame(RAC_MSG_DEVICE_INFO, RAC_ARM_NONE, &d, sizeof(d));
}

static void send_port_status(uint8_t index)
{
    const port_t *p = &g.ports[index];
    rac_port_status_t s;
    memset(&s, 0, sizeof(s));
    s.port = index;
    s.kind = p->kind;
    s.arm_id = p->arm;
    s.model = p->model;
    s.detected_model = p->detected_model;
    s.can_dialect = p->dialect;
    s.online_mask = p->online_mask;
    s.expected_mask = p->expected_mask;
    s.baudrate = p->baudrate;
    s.tx_errors = p->tx_errors;
    s.rx_errors = p->rx_errors + p->parser.checksum_errors;
    s.bus_load_permille = p->load_permille;
    send_frame(RAC_MSG_PORT_STATUS, p->arm, &s, sizeof(s));
}

static void send_controller_status(void)
{
    rac_controller_status_t s;
    memset(&s, 0, sizeof(s));
    s.uptime_us = g.now;
    s.tick_count = g.tick_count;
    s.overrun_count = g.overrun_count;
    s.max_tick_us = g.max_tick_us;
    s.host_rx_errors = g.deframer.errors;
    for (int i = 0; i < RAC_MAX_ARMS; i++) {
        s.arm_mode[i] = g.arms[i].mode;
    }
    /* The overall source is the follower's: that is what is executing. */
    for (int i = 0; i < RAC_MAX_ARMS; i++) {
        if (g.arms[i].source != RAC_SOURCE_NONE) {
            s.source = g.arms[i].source;
        }
    }
    s.estop = g.estop_state;
    s.host_alive = g.host_alive;
    for (int i = 0; i < RAC_MAX_PAIRS; i++) {
        const pair_t *p = &g.pairs[i];
        s.pair_phase[i] = p->phase;
        s.follow_mode[i] = p->follow_mode;
        s.sync_period_us[i] = p->sync_period_us;
        s.sync_latency_us[i] = p->sync_latency_us;
        memcpy(&s.rel_offset[i * J], p->rel_offset, sizeof(p->rel_offset));
    }
    send_frame(RAC_MSG_CONTROLLER_STATUS, RAC_ARM_NONE, &s, sizeof(s));
}

static void queue_sample(uint8_t arm_id, const rac_arm_state_t *s)
{
    uint16_t index;
    if (g.sample_count == SAMPLE_RING) {
        /* Drop the oldest. The gap is visible to the host in sample_seq. */
        g.sample_head = (uint16_t)((g.sample_head + 1u) % SAMPLE_RING);
        g.sample_count--;
        g.samples_dropped++;
    }
    index = (uint16_t)((g.sample_head + g.sample_count) % SAMPLE_RING);
    g.sample_ring[index] = *s;
    g.sample_arm[index] = arm_id;
    g.sample_count++;
}

static void flush_samples(void)
{
    while (g.sample_count) {
        send_frame(RAC_MSG_ARM_STATE, g.sample_arm[g.sample_head], &g.sample_ring[g.sample_head],
                   sizeof(rac_arm_state_t));
        g.sample_head = (uint16_t)((g.sample_head + 1u) % SAMPLE_RING);
        g.sample_count--;
    }
}

/* --------------------------------------------------------------- arm model */

static void set_default_calibration(arm_t *a)
{
    memset(&a->cal, 0, sizeof(a->cal));
    for (int i = 0; i < J; i++) {
        a->cal.direction[i] = 1.0f;
        a->cal.lower[i] = -6.28f;
        a->cal.upper[i] = 6.28f;
    }
    a->cal.lower[GRIPPER] = 0.0f;
    a->cal.upper[GRIPPER] = 1.0f;
    a->cal.gripper_closed = 0.0f;
    a->cal.gripper_open = model_is_star(a->model) ? STAR_GRIPPER_OPEN_RAD : RS_GRIPPER_OPEN_RAD;
}

static void set_default_limits(arm_t *a)
{
    /* MIT gains published for the B601-RS in its control stack documentation. */
    static const float rs_kp[J] = {80, 150, 150, 50, 50, 50, 50};
    static const float rs_kd[J] = {5, 10, 10, 5, 4, 4, 4};
    for (int i = 0; i < J; i++) {
        a->limits.velocity[i] = 1.5f;
        a->limits.acceleration[i] = 10.0f;
        a->limits.kp[i] = rs_kp[i];
        a->limits.kd[i] = rs_kd[i];
    }
    a->limits.velocity[GRIPPER] = 1.5f;
    a->limits.acceleration[GRIPPER] = 15.0f;
}

static float raw_to_joint(const arm_t *a, int j, float raw)
{
    if (j == GRIPPER) {
        float span = a->cal.gripper_open - a->cal.gripper_closed;
        if (span > -1e-6f && span < 1e-6f) return 0.0f;
        return (raw - a->cal.gripper_closed) / span;
    }
    return a->cal.direction[j] * (raw - a->cal.zero[j]);
}

static float joint_to_raw(const arm_t *a, int j, float q)
{
    if (j == GRIPPER) {
        return a->cal.gripper_closed + q * (a->cal.gripper_open - a->cal.gripper_closed);
    }
    float dir = a->cal.direction[j];
    if (dir > -1e-6f && dir < 1e-6f) dir = 1.0f;
    return a->cal.zero[j] + q / dir;
}

static float vel_to_joint(const arm_t *a, int j, float raw)
{
    if (j == GRIPPER) {
        float span = a->cal.gripper_open - a->cal.gripper_closed;
        if (span > -1e-6f && span < 1e-6f) return 0.0f;
        return raw / span;
    }
    return a->cal.direction[j] * raw;
}

static void queue_reset(arm_t *a, bool report)
{
    point_queue_t *q = &a->queue;
    if (report && q->count) {
        send_event(RAC_EVT_QUEUE_CLEARED, (uint8_t)(a - g.arms), 0, q->count, 0);
    }
    q->head = 0;
    q->count = 0;
    q->shift_us = 0;
    q->pause_start = 0;
    q->has_prev = 0;
    q->underrun_reported = 0;
    a->paused = 0;
}

static uint8_t arm_full_sample(const arm_t *a)
{
    return (uint8_t)((a->valid_mask & ALL_JOINTS_MASK) == ALL_JOINTS_MASK);
}

/* Seed the target from the measured pose. Energizing a joint against a stale
 * target would jump it there at full gain, so this runs on every transition
 * into a torque-carrying motion. */
static void seed_target(arm_t *a)
{
    memcpy(a->target, a->position, sizeof(a->target));
    memset(a->target_vel, 0, sizeof(a->target_vel));
    a->target_valid = 1;
}

static void port_send_stop_star(port_t *p, uint8_t method)
{
    uint8_t buf[16];
    for (uint8_t id = 0; id < STAR_IDS; id++) {
        size_t n = fs_encode_stop(buf, id, method, 0);
        if (!rac_port_uart_send(p->bus, buf, n)) {
            p->tx_errors++;
        }
        p->tx_bytes_window += (uint32_t)n;
    }
}

static void port_send_rs_all(port_t *p, bool enable)
{
    for (uint8_t i = 0; i < J; i++) {
        rac_can_frame_t f;
        uint8_t id = (uint8_t)(i + 1u);
        if (enable) {
            /* The private protocol keeps a run mode per motor. Selecting motion
             * control first means a motor last used elsewhere cannot start in
             * position or current mode. */
            if (p->dialect == RAC_CAN_DIALECT_RS_PRIVATE) {
                rs_encode_set_motion_mode(&f, id, RS_DEFAULT_HOST_ID);
                if (!rac_port_can_send(p->bus, &f)) p->tx_errors++;
            }
            rs_encode_enable(&f, p->dialect, id, RS_DEFAULT_HOST_ID);
        } else {
            rs_encode_disable(&f, p->dialect, id, RS_DEFAULT_HOST_ID, false);
        }
        if (!rac_port_can_send(p->bus, &f)) {
            p->tx_errors++;
        }
    }
}

static void arm_apply_torque(arm_t *a, bool enable)
{
    if (a->port == 0xFFu) return;
    port_t *p = &g.ports[a->port];
    if (p->kind == RAC_PORT_KIND_CAN) {
        port_send_rs_all(p, enable);
    } else if (!enable) {
        port_send_stop_star(p, FS_STOP_RELEASE);
    }
    /* A Star servo locks on its first position command, so enabling needs no frame. */
    a->torque_on = enable ? 1u : 0u;
}

static void arm_set_motion(arm_t *a, uint8_t motion)
{
    a->motion = motion;
    if (motion == MOTION_NONE) {
        if (a->torque_on) {
            arm_apply_torque(a, false);
        }
        a->target_valid = 0;
    } else if (!a->torque_on) {
        arm_apply_torque(a, true);
    }
}

static void arm_set_mode(arm_t *a, uint8_t mode, uint8_t source)
{
    uint8_t arm_id = (uint8_t)(a - g.arms);
    if (a->mode != mode) {
        send_event(RAC_EVT_MODE_CHANGED, arm_id, 0, mode, a->mode);
    }
    a->mode = mode;
    a->source = source;
}

/* Bring an arm to a stopped state. Used by STOP, comm loss and e-stop. */
static void arm_stop(arm_t *a, uint8_t kind)
{
    queue_reset(a, true);
    if (a->port == 0xFFu) {
        arm_set_mode(a, RAC_MODE_IDLE, RAC_SOURCE_NONE);
        return;
    }
    port_t *p = &g.ports[a->port];

    if (kind == RAC_STOP_RELEASE) {
        arm_set_motion(a, MOTION_NONE);
        arm_set_mode(a, RAC_MODE_IDLE, RAC_SOURCE_NONE);
        return;
    }
    if (kind == RAC_STOP_DAMP) {
        if (!a->torque_on) {
            /* Nothing to damp, and no reason to energize it. */
        } else if (p->kind == RAC_PORT_KIND_CAN) {
            arm_set_motion(a, MOTION_DAMP);
        } else {
            port_send_stop_star(p, FS_STOP_DAMP);
            a->torque_on = 0;
            a->motion = MOTION_NONE;
            a->target_valid = 0;
        }
        arm_set_mode(a, RAC_MODE_IDLE, RAC_SOURCE_NONE);
        return;
    }
    /* Hold, where the arm can. A passive leader stays passive. */
    if (model_can_hold(a->model) && arm_full_sample(a)) {
        seed_target(a);
        arm_set_motion(a, MOTION_POSITION);
        arm_set_mode(a, RAC_MODE_HOLD, RAC_SOURCE_NONE);
    } else {
        arm_set_motion(a, MOTION_NONE);
        arm_set_mode(a, RAC_MODE_IDLE, RAC_SOURCE_NONE);
    }
}

static void pair_set_phase(pair_t *p, uint8_t phase)
{
    if (p->phase != phase) {
        p->phase = phase;
        send_event(RAC_EVT_TELEOP_PHASE, RAC_ARM_NONE, 0, (uint32_t)(p - g.pairs), phase);
    }
}

static void pairs_stop_for_arm(uint8_t arm_id)
{
    for (int i = 0; i < RAC_MAX_PAIRS; i++) {
        pair_t *p = &g.pairs[i];
        if (p->configured && (p->cfg.leader_arm == arm_id || p->cfg.follower_arm == arm_id)) {
            pair_set_phase(p, RAC_PAIR_OFF);
        }
    }
}

static void stop_all(uint8_t kind)
{
    for (int i = 0; i < RAC_MAX_PAIRS; i++) {
        pair_set_phase(&g.pairs[i], RAC_PAIR_OFF);
    }
    for (int i = 0; i < RAC_MAX_ARMS; i++) {
        if (g.arms[i].port != 0xFFu) {
            arm_stop(&g.arms[i], kind);
        }
    }
}

/* ------------------------------------------------------------- bind and port */

static void port_unbind(uint8_t index)
{
    port_t *p = &g.ports[index];
    if (p->arm != RAC_ARM_NONE && p->arm < RAC_MAX_ARMS) {
        arm_t *a = &g.arms[p->arm];
        arm_set_motion(a, MOTION_NONE);
        arm_set_mode(a, RAC_MODE_IDLE, RAC_SOURCE_NONE);
        a->port = 0xFFu;
        a->model = RAC_MODEL_NONE;
        a->valid_mask = 0;
    }
    p->arm = RAC_ARM_NONE;
    p->model = RAC_MODEL_NONE;
    p->expected_mask = 0;
    p->online_mask = 0;
}

static uint8_t port_bind(uint8_t index, uint8_t arm_id, uint8_t model, uint8_t dialect, uint32_t baud)
{
    port_t *p = &g.ports[index];

    if (arm_id != RAC_ARM_NONE && arm_id >= RAC_MAX_ARMS) return RAC_ACK_BAD_ARGUMENT;
    if (p->kind == RAC_PORT_KIND_CAN && model_is_star(model)) return RAC_ACK_UNSUPPORTED;
    if (p->kind == RAC_PORT_KIND_UART && model == RAC_MODEL_REBOT_B601_RS) return RAC_ACK_UNSUPPORTED;
    /* A follower has to be able to execute. Leaders can be anything that reads. */
    if (arm_id != RAC_ARM_NONE && (arm_id == RAC_ARM_LEFT_FOLLOWER || arm_id == RAC_ARM_RIGHT_FOLLOWER) &&
        !model_can_execute(model)) {
        return RAC_ACK_UNSUPPORTED;
    }

    /* Rebinding an arm energized elsewhere is never silent: refuse it. */
    if (arm_id != RAC_ARM_NONE && g.arms[arm_id].torque_on && g.arms[arm_id].port != index) {
        return RAC_ACK_BUSY;
    }
    if (p->arm != RAC_ARM_NONE && g.arms[p->arm].torque_on) {
        return RAC_ACK_BUSY;
    }

    port_unbind(index);
    if (arm_id == RAC_ARM_NONE) {
        return RAC_ACK_OK;
    }
    if (g.arms[arm_id].port != 0xFFu) {
        port_unbind(g.arms[arm_id].port);
    }

    arm_t *a = &g.arms[arm_id];
    memset(a, 0, sizeof(*a));
    a->port = index;
    a->model = model;
    a->pair = 0xFFu;
    for (uint8_t i = 0; i < RAC_MAX_PAIRS; i++) {
        const pair_t *pr = &g.pairs[i];
        if (pr->configured && pr->cfg.enabled &&
            (pr->cfg.leader_arm == arm_id || pr->cfg.follower_arm == arm_id)) {
            a->pair = i;
        }
    }
    a->mode = RAC_MODE_IDLE;
    a->is_leader = (arm_id == RAC_ARM_LEFT_LEADER || arm_id == RAC_ARM_RIGHT_LEADER) ? 1u : 0u;
    set_default_calibration(a);
    set_default_limits(a);

    p->arm = arm_id;
    p->model = model;
    p->dialect = dialect;
    p->expected_mask = ALL_JOINTS_MASK;
    p->online_mask = 0;
    p->cycle_mask = 0;
    p->cycle_open = 0;
    if (p->kind == RAC_PORT_KIND_UART && baud) {
        p->baudrate = baud;
        rac_port_uart_set_baud(p->bus, baud);
    }
    memset(p->raw_ts, 0, sizeof(p->raw_ts));
    p->next_service = g.now;
    p->next_read = g.now;
    return RAC_ACK_OK;
}

static void discover_start(uint8_t index)
{
    port_t *p = &g.ports[index];
    /* Discovery is read-only, but a motor that is executing must never be
     * disturbed, so a port carrying torque simply reports what it knows. */
    if (p->arm != RAC_ARM_NONE && g.arms[p->arm].torque_on) {
        return;
    }
    p->discovering = 1;
    p->discover_mask = 0;
    p->discover_until = g.now + DISCOVER_WINDOW_US;
    p->discover_dialect = p->dialect;
    p->detected_model = RAC_MODEL_NONE;

    if (p->kind == RAC_PORT_KIND_CAN) {
        for (uint8_t i = 0; i < J; i++) {
            rac_can_frame_t f;
            /* Both dialects. A motor ignores frames of the protocol it is not in. */
            rs_encode_query(&f, RAC_CAN_DIALECT_RS_PRIVATE, (uint8_t)(i + 1u), RS_DEFAULT_HOST_ID);
            (void)rac_port_can_send(p->bus, &f);
            rs_encode_query(&f, RAC_CAN_DIALECT_RS_MIT, (uint8_t)(i + 1u), RS_DEFAULT_HOST_ID);
            (void)rac_port_can_send(p->bus, &f);
        }
    } else {
        fs_parser_reset(&p->parser);
        for (uint8_t id = 0; id <= STAR_BUTTON_BOARD_ID; id++) {
            uint8_t buf[8];
            size_t n = fs_encode_ping(buf, id);
            (void)rac_port_uart_send(p->bus, buf, n);
        }
    }
}

static void discover_finish(port_t *p)
{
    p->discovering = 0;
    if (p->kind == RAC_PORT_KIND_CAN) {
        p->detected_model = (p->discover_mask & ALL_JOINTS_MASK) ? RAC_MODEL_REBOT_B601_RS : RAC_MODEL_NONE;
        if (p->detected_model != RAC_MODEL_NONE) {
            p->dialect = p->discover_dialect;
        }
    } else {
        if ((p->discover_mask & ALL_JOINTS_MASK) == 0) {
            p->detected_model = RAC_MODEL_NONE;
        } else if (p->discover_mask & (1u << STAR_BUTTON_BOARD_ID)) {
            /* Only the HD carries the button board, at servo id 7. */
            p->detected_model = RAC_MODEL_STAR_102_HD;
        } else {
            /* LD and FL differ in servo model only, so the user confirms which. */
            p->detected_model = RAC_MODEL_STAR_102_UNKNOWN;
        }
    }
    if (p->arm == RAC_ARM_NONE) {
        p->online_mask = p->discover_mask & ALL_JOINTS_MASK;
    }
    send_port_status((uint8_t)(p - g.ports));
}

/* ---------------------------------------------------------------- feedback */

static void port_note_feedback(port_t *p, int joint, float pos, float vel, float tau, float temp,
                               uint8_t fault, uint64_t ts)
{
    p->raw_position[joint] = pos;
    p->raw_velocity[joint] = vel;
    p->raw_torque[joint] = tau;
    p->raw_temperature[joint] = (uint8_t)clampf(temp, 0.0f, 255.0f);
    p->raw_fault[joint] = fault;
    p->raw_ts[joint] = ts;
    p->cycle_mask |= (uint8_t)(1u << joint);
}

void rac_core_on_can_frame(uint8_t can_index, const rac_can_frame_t *frame, uint64_t rx_ts_us)
{
    port_t *p = NULL;
    for (int i = 0; i < RAC_MAX_PORTS; i++) {
        if (g.ports[i].kind == RAC_PORT_KIND_CAN && g.ports[i].bus == can_index) {
            p = &g.ports[i];
        }
    }
    if (!p) return;

    rs_feedback_t fb;
    if (p->discovering) {
        /* A 29-bit answer means private protocol, an 11-bit one means MIT. */
        uint8_t dialect = frame->extended ? RAC_CAN_DIALECT_RS_PRIVATE : RAC_CAN_DIALECT_RS_MIT;
        if (rs_decode_feedback(frame, dialect, RS_DEFAULT_HOST_ID, rs_type_of_id, &fb) &&
            fb.motor_id >= 1 && fb.motor_id <= J) {
            p->discover_mask |= (uint8_t)(1u << (fb.motor_id - 1u));
            p->discover_dialect = dialect;
        }
        return;
    }
    if (p->arm == RAC_ARM_NONE) return;
    if (!rs_decode_feedback(frame, p->dialect, RS_DEFAULT_HOST_ID, rs_type_of_id, &fb)) {
        return;
    }
    if (fb.motor_id < 1 || fb.motor_id > J) {
        p->rx_errors++;
        return;
    }
    port_note_feedback(p, fb.motor_id - 1, fb.position, fb.velocity, fb.torque, fb.temperature,
                       fb.fault_bits, rx_ts_us);
}

void rac_core_on_uart_bytes(uint8_t uart_index, const uint8_t *data, size_t len, uint64_t rx_ts_us)
{
    port_t *p = NULL;
    for (int i = 0; i < RAC_MAX_PORTS; i++) {
        if (g.ports[i].kind == RAC_PORT_KIND_UART && g.ports[i].bus == uart_index) {
            p = &g.ports[i];
        }
    }
    if (!p) return;

    for (size_t i = 0; i < len; i++) {
        const uint8_t *frame = fs_parser_push(&p->parser, data[i]);
        if (!frame) continue;

        if (p->discovering) {
            int id = fs_decode_ping(frame);
            if (id >= 0 && id <= (int)STAR_BUTTON_BOARD_ID) {
                p->discover_mask |= (uint8_t)(1u << id);
            }
            continue;
        }
        if (p->arm == RAC_ARM_NONE) continue;
        fs_monitor_t m;
        if (fs_decode_monitor(frame, &m) && m.servo_id < J && m.angle_valid) {
            float pos = m.angle_deg * PI_F / 180.0f;
            /* Star servos report no velocity, so difference it across samples. */
            int j = m.servo_id;
            float vel = 0.0f;
            if (p->raw_ts[j] != 0 && rx_ts_us > p->raw_ts[j]) {
                float dt = (float)(rx_ts_us - p->raw_ts[j]) * 1e-6f;
                vel = (pos - p->raw_position[j]) / dt;
            }
            /* Current (mA) stands in for torque; the host scales it per servo. */
            port_note_feedback(p, j, pos, vel, (float)m.current_ma * 1e-3f, fs_temperature_c(m.temp_raw),
                               (uint8_t)(m.status & 0x3Fu), rx_ts_us);
        }
    }
}

/* Close a read cycle: convert raw feedback, detect losses, and emit one sample. */
static void port_close_cycle(port_t *p, uint64_t now)
{
    if (p->arm == RAC_ARM_NONE) return;
    uint8_t arm_id = p->arm;
    arm_t *a = &g.arms[arm_id];

    uint8_t online = 0;
    uint8_t fault = 0;
    for (int j = 0; j < J; j++) {
        if (p->raw_ts[j] != 0 && now - p->raw_ts[j] < MOTOR_TIMEOUT_US) {
            online |= (uint8_t)(1u << j);
        }
        if (p->raw_fault[j]) {
            fault |= (uint8_t)(1u << j);
        }
    }
    uint8_t lost = (uint8_t)(p->online_mask & ~online);
    uint8_t found = (uint8_t)(online & ~p->online_mask);
    if (lost) {
        send_event(RAC_EVT_MOTOR_LOST, arm_id, 2, lost, 0);
    }
    if (found) {
        send_event(RAC_EVT_MOTOR_RESTORED, arm_id, 0, found, 0);
    }
    if (fault && !a->fault_mask) {
        send_event(RAC_EVT_MOTOR_FAULT, arm_id, 2, fault, 0);
    }
    p->online_mask = online;
    a->fault_mask = fault;

    uint8_t valid = p->cycle_mask;
    for (int j = 0; j < J; j++) {
        if (valid & (1u << j)) {
            a->position[j] = raw_to_joint(a, j, p->raw_position[j]);
            a->velocity[j] = vel_to_joint(a, j, p->raw_velocity[j]);
            a->torque[j] = p->raw_torque[j];
            a->temperature[j] = p->raw_temperature[j];
        }
    }
    a->valid_mask = online;
    a->sample_seq++;
    a->sample_ts = p->cycle_start ? p->cycle_start : now;
    a->sample_fresh = 1;

    rac_arm_state_t s;
    memset(&s, 0, sizeof(s));
    s.sample_seq = a->sample_seq;
    s.sample_ts_us = a->sample_ts;
    s.target_seq = a->target_seq;
    s.target_ts_us = a->target_ts;
    s.exec_seq = a->queue.exec_seq;
    s.valid_mask = valid;
    s.flags = 0;
    if (a->target_dispatched) s.flags |= RAC_STATE_TARGET_VALID;
    if (a->clamped) s.flags |= RAC_STATE_LIMIT_CLAMPED;
    if (a->torque_on) s.flags |= RAC_STATE_TORQUE_ON;
    if (a->queue.count == 0) s.flags |= RAC_STATE_QUEUE_EMPTY;
    if (a->pair != 0xFFu && g.pairs[a->pair].leader_stale) s.flags |= RAC_STATE_LEADER_STALE;
    s.mode = a->mode;
    s.source = a->source;
    s.queue_depth = a->queue.count;
    s.fault_mask = fault;
    s.phase = (a->pair != 0xFFu) ? g.pairs[a->pair].phase : (uint8_t)RAC_PAIR_OFF;
    memcpy(s.position, a->position, sizeof(s.position));
    memcpy(s.velocity, a->velocity, sizeof(s.velocity));
    memcpy(s.torque, a->torque, sizeof(s.torque));
    memcpy(s.target, a->target, sizeof(s.target));
    memcpy(s.temperature, a->temperature, sizeof(s.temperature));
    queue_sample(arm_id, &s);

    a->target_dispatched = 0;
    p->cycle_mask = 0;
    p->cycle_open = 0;

    /* A motor lost while energized stops the whole arm, and its pair. */
    if (lost && a->torque_on && a->mode != RAC_MODE_ESTOP) {
        pairs_stop_for_arm(arm_id);
        if (a->mode != RAC_MODE_HOLD) {
            arm_stop(a, RAC_STOP_HOLD);
        }
    }
    /* A faulted leader cannot be followed either. */
    if (lost && a->is_leader) {
        pairs_stop_for_arm(arm_id);
    }
}

/* ---------------------------------------------------------- target generators */

static void step_limited(float *pos, float *vel, float goal, float vmax, float amax, float dt)
{
    float error = goal - *pos;
    float stopping = sqrtf(2.0f * amax * fabsf(error));
    float desired = copysignf(stopping < vmax ? stopping : vmax, error);
    float linear = error / dt;
    if (fabsf(linear) < fabsf(desired)) {
        desired = linear;
    }
    float dv = clampf(desired - *vel, -amax * dt, amax * dt);
    *vel += dv;
    *pos += *vel * dt;
}

static void queue_update_goal(arm_t *a, uint64_t now, float *goal)
{
    point_queue_t *q = &a->queue;
    memcpy(goal, a->target, sizeof(float) * J);
    if (a->paused) {
        return;
    }
    while (q->count) {
        const queue_point_t *head = &q->points[q->head];
        uint64_t t = head->t_us + q->shift_us;
        if (t > now) {
            break;
        }
        memcpy(q->prev_q, head->q, sizeof(q->prev_q));
        q->prev_t_us = t;
        q->has_prev = 1;
        q->exec_seq = head->seq;
        q->head = (uint16_t)((q->head + 1u) % QUEUE_POINTS);
        q->count--;
    }
    if (q->count) {
        const queue_point_t *head = &q->points[q->head];
        uint64_t t = head->t_us + q->shift_us;
        if (q->has_prev && t > q->prev_t_us) {
            float alpha = (float)(now - q->prev_t_us) / (float)(t - q->prev_t_us);
            alpha = clampf(alpha, 0.0f, 1.0f);
            for (int i = 0; i < J; i++) {
                goal[i] = q->prev_q[i] + alpha * (head->q[i] - q->prev_q[i]);
            }
        } else if (q->has_prev) {
            memcpy(goal, q->prev_q, sizeof(q->prev_q));
        }
        /* With no previous point yet the goal stays at the current target, so
         * playback starts from where the arm is rather than from a jump. */
        q->underrun_reported = 0;
    } else {
        if (q->has_prev) {
            memcpy(goal, q->prev_q, sizeof(q->prev_q));
        }
        if (q->has_prev && !q->underrun_reported) {
            q->underrun_reported = 1;
            send_event(RAC_EVT_QUEUE_UNDERRUN, (uint8_t)(a - g.arms), 1, q->exec_seq, 0);
        }
    }
}

static void pair_map(const pair_t *p, const arm_t *leader, float *mapped)
{
    for (int j = 0; j < J; j++) {
        uint8_t src = p->cfg.source_joint[j];
        if (src >= J) src = (uint8_t)j;
        mapped[j] = p->cfg.scale[j] * leader->position[src] + p->cfg.offset[j];
    }
}

static void update_pairs(uint64_t now)
{
    for (int i = 0; i < RAC_MAX_PAIRS; i++) {
        pair_t *p = &g.pairs[i];
        if (!p->configured || !p->cfg.enabled || p->phase == RAC_PAIR_OFF) continue;
        const arm_t *leader = &g.arms[p->cfg.leader_arm];

        bool stale = (leader->port == 0xFFu) || !arm_full_sample(leader) ||
                     (now - leader->sample_ts > LEADER_STALE_US);
        if (stale && !p->leader_stale) {
            send_event(RAC_EVT_LEADER_STALE, p->cfg.leader_arm, 2, (uint32_t)i, 0);
        }
        p->leader_stale = stale ? 1u : 0u;
    }
}

/* Compute the goal for one arm in its service slot and step the rate limiter. */
static void arm_compute_target(arm_t *a, uint64_t now, float dt)
{
    float goal[J];
    float vscale = 1.0f;
    uint8_t arm_id = (uint8_t)(a - g.arms);

    if (!a->target_valid) return;
    memcpy(goal, a->target, sizeof(goal));

    if (a->mode == RAC_MODE_HOST_TARGET) {
        queue_update_goal(a, now, goal);
    } else if (a->mode == RAC_MODE_LOCAL_TELEOP && a->pair != 0xFFu) {
        pair_t *p = &g.pairs[a->pair];
        const arm_t *leader = &g.arms[p->cfg.leader_arm];
        if (p->phase == RAC_PAIR_PAUSED || p->leader_stale || p->phase == RAC_PAIR_OFF) {
            /* Hold what was last commanded. */
        } else {
            float mapped[J];
            pair_map(p, leader, mapped);
            for (int j = 0; j < J; j++) {
                goal[j] = mapped[j];
                if (p->follow_mode == RAC_FOLLOW_RELATIVE && j != GRIPPER) {
                    goal[j] += p->rel_offset[j];
                }
            }
            if (!p->cfg.gripper_link) {
                goal[GRIPPER] = a->target[GRIPPER];
            }
            if (p->phase == RAC_PAIR_ALIGNING || p->phase == RAC_PAIR_ALIGNED) {
                float max_v = 0.0f;
                for (int j = 0; j < J; j++) {
                    if (a->limits.velocity[j] > max_v) max_v = a->limits.velocity[j];
                }
                if (max_v > 0.0f && p->align_speed > 0.0f && p->align_speed < max_v) {
                    vscale = p->align_speed / max_v;
                }
            }
            if (p->phase == RAC_PAIR_ALIGNING) {
                bool done = true;
                for (int j = 0; j < J; j++) {
                    if (fabsf(goal[j] - a->target[j]) > ALIGN_TARGET_TOLERANCE ||
                        fabsf(goal[j] - a->position[j]) > ALIGN_MEASURED_TOLERANCE) {
                        done = false;
                    }
                }
                if (done) {
                    pair_set_phase(p, RAC_PAIR_ALIGNED);
                    send_event(RAC_EVT_ALIGN_DONE, arm_id, 0, (uint32_t)(p - g.pairs), 0);
                }
            }
            p->sync_latency_us = (uint32_t)(now - leader->sample_ts);
        }
        if (p->last_dispatch) {
            p->sync_period_us = (uint32_t)(now - p->last_dispatch);
        }
        p->last_dispatch = now;
    }

    uint8_t clamped = 0;
    for (int j = 0; j < J; j++) {
        float g_clamped = clampf(goal[j], a->cal.lower[j], a->cal.upper[j]);
        if (g_clamped != goal[j]) clamped |= (uint8_t)(1u << j);
        step_limited(&a->target[j], &a->target_vel[j], g_clamped, a->limits.velocity[j] * vscale,
                     a->limits.acceleration[j], dt);
        /* The limiter may overshoot by a step; the joint limit never gives way. */
        a->target[j] = clampf(a->target[j], a->cal.lower[j], a->cal.upper[j]);
    }
    a->clamped = clamped ? 1u : 0u;
    if (clamped && now - a->last_limit_event > LIMIT_EVENT_PERIOD_US) {
        a->last_limit_event = now;
        send_event(RAC_EVT_LIMIT_CLAMPED, arm_id, 1, clamped, 0);
    }
}

/* ---------------------------------------------------------------- dispatchers */

static void service_rs(port_t *p, uint64_t now)
{
    arm_t *a = &g.arms[p->arm];
    float dt = (float)p->service_period_us * 1e-6f;
    bool ff_fresh = now < a->ff_expire;

    if (a->motion == MOTION_NONE) {
        /* Reading only: a query is answered without energizing anything. */
        if (now >= p->next_read) {
            p->next_read = now + p->read_period_us;
            if (p->cycle_open) port_close_cycle(p, now);
            p->cycle_start = now;
            p->cycle_open = 1;
            for (uint8_t i = 0; i < J; i++) {
                rac_can_frame_t f;
                rs_encode_query(&f, p->dialect, (uint8_t)(i + 1u), RS_DEFAULT_HOST_ID);
                if (!rac_port_can_send(p->bus, &f)) p->tx_errors++;
            }
        }
        return;
    }

    if (p->cycle_open) port_close_cycle(p, now);
    p->cycle_start = now;
    p->cycle_open = 1;

    if (a->motion == MOTION_POSITION) {
        arm_compute_target(a, now, dt);
    }

    for (uint8_t i = 0; i < J; i++) {
        rac_can_frame_t f;
        uint8_t id = (uint8_t)(i + 1u);
        rs_motor_type_t type = rs_type_of_id(id);
        float tau = ff_fresh ? a->feedforward[i] : 0.0f;
        /* Motors take raw units; feedforward stays in motor torque sign. */
        tau *= a->cal.direction[i] < 0.0f ? -1.0f : 1.0f;

        switch (a->motion) {
        case MOTION_POSITION: {
            float raw = joint_to_raw(a, i, a->target[i]);
            float raw_vel = a->target_vel[i];
            if (i == GRIPPER) {
                raw_vel *= (a->cal.gripper_open - a->cal.gripper_closed);
            } else if (a->cal.direction[i] < 0.0f) {
                raw_vel = -raw_vel;
            }
            rs_encode_motion(&f, p->dialect, type, id, raw, raw_vel, a->limits.kp[i], a->limits.kd[i], tau);
            break;
        }
        case MOTION_GUIDE: {
            float kd = (i < 3) ? RS_GUIDE_KD_LARGE : RS_GUIDE_KD_SMALL;
            /* If the host stops refreshing gravity torque the arm must not be
             * left unsupported, so damping rises to the stop value. */
            if (!ff_fresh) kd = (i < 3) ? RS_DAMP_KD_LARGE : RS_DAMP_KD_SMALL;
            rs_encode_motion(&f, p->dialect, type, id, 0.0f, 0.0f, 0.0f, kd, tau);
            break;
        }
        case MOTION_DAMP:
        default: {
            float kd = (i < 3) ? RS_DAMP_KD_LARGE : RS_DAMP_KD_SMALL;
            rs_encode_motion(&f, p->dialect, type, id, 0.0f, 0.0f, 0.0f, kd, 0.0f);
            break;
        }
        }
        if (!rac_port_can_send(p->bus, &f)) p->tx_errors++;
    }

    if (a->motion == MOTION_POSITION) {
        a->target_seq++;
        a->target_ts = now;
        a->target_dispatched = 1;
    }
}

static void service_star(port_t *p, uint64_t now)
{
    static const uint8_t ids[STAR_IDS] = {0, 1, 2, 3, 4, 5, 6};
    uint8_t buf[FS_MAX_FRAME];
    arm_t *a = &g.arms[p->arm];
    float dt = (float)p->service_period_us * 1e-6f;

    /* A half-duplex bus runs one exchange at a time, so a cycle only closes
     * once the previous one has had its full response window. */
    bool read_due = now >= p->next_read;
    if (read_due) {
        p->next_read = now + p->read_period_us;
        if (p->cycle_open) {
            port_close_cycle(p, now);
        }
        p->cycle_start = now;
        p->cycle_open = 1;
        fs_parser_reset(&p->parser);
    }

    if (a->motion == MOTION_POSITION && model_can_hold(a->model)) {
        float degrees[STAR_IDS];
        arm_compute_target(a, now, dt);
        for (int j = 0; j < J; j++) {
            degrees[j] = joint_to_raw(a, j, a->target[j]) * 180.0f / PI_F;
        }
        uint32_t interval_ms = p->service_period_us / 1000u;
        if (interval_ms == 0) interval_ms = 1;
        size_t n = fs_encode_sync_angle(buf, ids, degrees, STAR_IDS, interval_ms, 0);
        if (!rac_port_uart_send(p->bus, buf, n)) p->tx_errors++;
        p->tx_bytes_window += (uint32_t)n;
        a->target_seq++;
        a->target_ts = now;
        a->target_dispatched = 1;
    }

    if (read_due) {
        size_t n = fs_encode_sync_monitor(buf, ids, STAR_IDS);
        if (!rac_port_uart_send(p->bus, buf, n)) p->tx_errors++;
        p->tx_bytes_window += (uint32_t)n;
    }
}

static void service_ports(uint64_t now)
{
    for (int i = 0; i < RAC_MAX_PORTS; i++) {
        port_t *p = &g.ports[i];

        if (p->discovering) {
            if (now >= p->discover_until) discover_finish(p);
            continue;
        }
        if (p->arm == RAC_ARM_NONE) continue;
        if (now < p->next_service) continue;

        /* Stay on the original grid: late service does not shift later slots,
         * which is what keeps two arms in step over a long session. */
        p->next_service += p->service_period_us;
        if (p->next_service <= now) p->next_service = now + p->service_period_us;

        if (p->kind == RAC_PORT_KIND_CAN) {
            service_rs(p, now);
        } else {
            service_star(p, now);
        }

        if (now - p->load_window_start >= 1000000u) {
            /* CAN frames carry roughly 130 bits, UART bytes 10 bits. */
            uint32_t rate = p->kind == RAC_PORT_KIND_CAN ? 1000000u : (p->baudrate ? p->baudrate : 1000000u);
            uint32_t bits = p->kind == RAC_PORT_KIND_CAN
                                ? (uint32_t)(2u * J * (1000000u / p->service_period_us) * 130u)
                                : p->tx_bytes_window * 10u + (uint32_t)(147u * 10u * (1000000u / p->service_period_us));
            p->load_permille = (uint16_t)((uint64_t)bits * 1000u / rate);
            p->tx_bytes_window = 0;
            p->load_window_start = now;
        }
    }
}

/* ------------------------------------------------------------------ commands */

static bool arm_mode_allowed(const arm_t *a, uint8_t mode)
{
    switch (mode) {
    case RAC_MODE_IDLE: return true;
    case RAC_MODE_HOLD: return model_can_hold(a->model);
    case RAC_MODE_HOST_TARGET: return model_can_execute(a->model);
    case RAC_MODE_LOCAL_TELEOP:
        /* A leader may stay passive; a follower must execute. */
        return a->is_leader ? true : model_can_execute(a->model);
    case RAC_MODE_GUIDE: return model_can_guide(a->model);
    default: return false;
    }
}

static uint8_t handle_set_mode(const rac_header_t *h, const rac_set_mode_t *m)
{
    if (g.estop_state != RAC_ESTOP_CLEAR) return RAC_ACK_ESTOP_ACTIVE;
    if (m->mode == RAC_MODE_ESTOP) return RAC_ACK_BAD_ARGUMENT;

    /* Validate the whole mask first so a rejection changes nothing. */
    for (uint8_t i = 0; i < RAC_MAX_ARMS; i++) {
        if (!(m->arm_mask & (1u << i))) continue;
        arm_t *a = &g.arms[i];
        if (a->port == 0xFFu) return RAC_ACK_BAD_ARGUMENT;
        if (!arm_mode_allowed(a, m->mode)) return RAC_ACK_UNSUPPORTED;
        bool needs_pose = m->mode == RAC_MODE_HOLD || m->mode == RAC_MODE_HOST_TARGET ||
                          (m->mode == RAC_MODE_LOCAL_TELEOP && !a->is_leader);
        if (needs_pose && !arm_full_sample(a)) return RAC_ACK_MODE_REJECTED;
    }

    g.generation = h->generation;

    for (uint8_t i = 0; i < RAC_MAX_ARMS; i++) {
        if (!(m->arm_mask & (1u << i))) continue;
        arm_t *a = &g.arms[i];
        bool was_position = a->motion == MOTION_POSITION;
        queue_reset(a, true);

        switch (m->mode) {
        case RAC_MODE_IDLE:
            pairs_stop_for_arm(i);
            arm_set_motion(a, MOTION_NONE);
            break;
        case RAC_MODE_HOLD:
        case RAC_MODE_HOST_TARGET:
            /* Keep the running target across a handover so a change of source
             * cannot produce a step at the joint. */
            if (!was_position) seed_target(a);
            arm_set_motion(a, MOTION_POSITION);
            break;
        case RAC_MODE_LOCAL_TELEOP:
            if (a->is_leader) {
                /* An RS leader with no brakes needs guide; a Star leader reads. */
                if (model_can_guide(a->model)) {
                    arm_set_motion(a, MOTION_GUIDE);
                } else {
                    arm_set_motion(a, MOTION_NONE);
                }
            } else {
                if (!was_position) seed_target(a);
                arm_set_motion(a, MOTION_POSITION);
            }
            break;
        case RAC_MODE_GUIDE:
            arm_set_motion(a, MOTION_GUIDE);
            break;
        default:
            break;
        }
        arm_set_mode(a, m->mode, m->source);
    }
    return RAC_ACK_OK;
}

static uint8_t handle_target_chunk(const rac_header_t *h, const uint8_t *body, size_t len, uint16_t *free_out)
{
    if (len < sizeof(rac_target_chunk_t)) return RAC_ACK_BAD_ARGUMENT;
    rac_target_chunk_t c;
    memcpy(&c, body, sizeof(c));
    if (h->arm_id >= RAC_MAX_ARMS) return RAC_ACK_BAD_ARGUMENT;
    if (c.joint_count != J || c.point_count == 0 || c.point_count > RAC_MAX_CHUNK_POINTS) {
        return RAC_ACK_BAD_ARGUMENT;
    }
    if (len != sizeof(c) + (size_t)c.point_count * J * sizeof(float)) return RAC_ACK_BAD_ARGUMENT;

    arm_t *a = &g.arms[h->arm_id];
    *free_out = (uint16_t)(QUEUE_POINTS - a->queue.count);
    if (g.estop_state != RAC_ESTOP_CLEAR) return RAC_ACK_ESTOP_ACTIVE;
    if (a->mode != RAC_MODE_HOST_TARGET) return RAC_ACK_MODE_REJECTED;
    if ((uint32_t)a->queue.count + c.point_count > QUEUE_POINTS) return RAC_ACK_QUEUE_FULL;

    point_queue_t *q = &a->queue;
    const uint8_t *data = body + sizeof(c);
    uint64_t t = c.t_start_us;
    if (t == 0) {
        /* Continue right after what is queued, or begin now. */
        if (q->count) {
            uint16_t last = (uint16_t)((q->head + q->count - 1u) % QUEUE_POINTS);
            t = q->points[last].t_us + c.dt_us;
        } else {
            t = g.now + c.dt_us;
        }
    }
    /* Interpolation runs from the running target to the first queued point. */
    if (q->count == 0 && !q->has_prev) {
        memcpy(q->prev_q, a->target, sizeof(q->prev_q));
        q->prev_t_us = g.now;
        q->has_prev = 1;
    }
    for (uint16_t i = 0; i < c.point_count; i++) {
        uint16_t index = (uint16_t)((q->head + q->count) % QUEUE_POINTS);
        queue_point_t *pt = &q->points[index];
        pt->t_us = t + (uint64_t)i * c.dt_us;
        pt->seq = c.exec_seq + i;
        memcpy(pt->q, data + (size_t)i * J * sizeof(float), sizeof(pt->q));
        q->count++;
    }
    *free_out = (uint16_t)(QUEUE_POINTS - q->count);
    return RAC_ACK_OK;
}

static uint8_t handle_teleop(const rac_teleop_control_t *c)
{
    if (g.estop_state != RAC_ESTOP_CLEAR) return RAC_ACK_ESTOP_ACTIVE;

    for (uint8_t i = 0; i < RAC_MAX_PAIRS; i++) {
        if (!(c->pair_mask & (1u << i))) continue;
        pair_t *p = &g.pairs[i];
        if (!p->configured || !p->cfg.enabled) return RAC_ACK_BAD_ARGUMENT;
        arm_t *leader = &g.arms[p->cfg.leader_arm];
        arm_t *follower = &g.arms[p->cfg.follower_arm];
        if (leader->port == 0xFFu || follower->port == 0xFFu) return RAC_ACK_BAD_ARGUMENT;
        if (c->action != RAC_TELEOP_END) {
            if (follower->mode != RAC_MODE_LOCAL_TELEOP) return RAC_ACK_MODE_REJECTED;
            if (!arm_full_sample(leader) || !arm_full_sample(follower)) return RAC_ACK_MODE_REJECTED;
        }
    }

    for (uint8_t i = 0; i < RAC_MAX_PAIRS; i++) {
        if (!(c->pair_mask & (1u << i))) continue;
        pair_t *p = &g.pairs[i];
        arm_t *leader = &g.arms[p->cfg.leader_arm];
        arm_t *follower = &g.arms[p->cfg.follower_arm];

        switch (c->action) {
        case RAC_TELEOP_ALIGN:
            p->align_speed = c->align_speed;
            p->follow_mode = RAC_FOLLOW_ABSOLUTE;
            memset(p->rel_offset, 0, sizeof(p->rel_offset));
            pair_set_phase(p, RAC_PAIR_ALIGNING);
            break;
        case RAC_TELEOP_START:
            p->follow_mode = c->follow_mode;
            if (c->follow_mode == RAC_FOLLOW_RELATIVE) {
                float mapped[J];
                pair_map(p, leader, mapped);
                for (int j = 0; j < J; j++) {
                    p->rel_offset[j] = follower->position[j] - mapped[j];
                }
                /* The gripper tracks absolutely: a relative gripper would be
                 * unable to close once offset. */
                p->rel_offset[GRIPPER] = 0.0f;
            } else {
                memset(p->rel_offset, 0, sizeof(p->rel_offset));
            }
            pair_set_phase(p, RAC_PAIR_FOLLOWING);
            break;
        case RAC_TELEOP_PAUSE:
            pair_set_phase(p, RAC_PAIR_PAUSED);
            break;
        case RAC_TELEOP_RESUME:
            /* In relative mode the offset is captured again, so the hand may
             * have moved during the pause without the follower jumping. */
            if (p->follow_mode == RAC_FOLLOW_RELATIVE) {
                float mapped[J];
                pair_map(p, leader, mapped);
                for (int j = 0; j < J; j++) {
                    p->rel_offset[j] = follower->target[j] - mapped[j];
                }
                p->rel_offset[GRIPPER] = 0.0f;
            }
            pair_set_phase(p, RAC_PAIR_FOLLOWING);
            break;
        case RAC_TELEOP_END:
            pair_set_phase(p, RAC_PAIR_OFF);
            break;
        default:
            return RAC_ACK_BAD_ARGUMENT;
        }
    }
    return RAC_ACK_OK;
}

static void handle_message(const uint8_t *payload, size_t len, uint64_t rx_ts)
{
    rac_header_t h;
    if (len < sizeof(h)) {
        g.deframer.errors++;
        return;
    }
    memcpy(&h, payload, sizeof(h));
    const uint8_t *body = payload + sizeof(h);
    size_t body_len = len - sizeof(h);

    if (h.version != RAC_PROTO_VERSION) {
        send_ack(&h, RAC_ACK_UNSUPPORTED, 0, 0);
        return;
    }

    g.last_host_rx = rx_ts;
    if (!g.host_alive) {
        g.host_alive = 1;
        send_event(RAC_EVT_HOST_RESTORED, RAC_ARM_NONE, 0, 0, 0);
    }

    /* Time sync and heartbeats predate any generation question. */
    if (h.msg_type == RAC_MSG_TIME_SYNC) {
        rac_time_sync_reply_t r;
        r.host_t1_us = h.timestamp_us;
        r.mcu_t2_us = rx_ts;
        r.mcu_t3_us = rac_port_now_us();
        send_frame(RAC_MSG_TIME_SYNC_REPLY, RAC_ARM_NONE, &r, sizeof(r));
        return;
    }
    if (h.msg_type == RAC_MSG_HEARTBEAT) {
        return;
    }

    /* Everything that can start motion is subject to the generation check. A
     * stop is exempt because it must always work, and read-only requests are
     * exempt because a freshly started host has not learned the generation yet. */
    bool exempt = h.msg_type == RAC_MSG_STOP || h.msg_type == RAC_MSG_HELLO ||
                  h.msg_type == RAC_MSG_QUERY_STATUS || h.msg_type == RAC_MSG_DISCOVER;
    if (!exempt && h.generation < g.generation) {
        send_event(RAC_EVT_STALE_REJECTED, h.arm_id, 1, h.generation, g.generation);
        send_ack(&h, RAC_ACK_STALE_GENERATION, 0, 0);
        return;
    }

    uint8_t status = RAC_ACK_OK;
    uint32_t exec = 0;
    uint16_t queue_free = 0;

#define REQUIRE_BODY(T)                       \
    T m;                                      \
    if (body_len != sizeof(m)) {              \
        send_ack(&h, RAC_ACK_BAD_ARGUMENT, 0, 0); \
        return;                               \
    }                                         \
    memcpy(&m, body, sizeof(m))

    switch (h.msg_type) {
    case RAC_MSG_HELLO:
        send_device_info();
        for (uint8_t i = 0; i < RAC_MAX_PORTS; i++) send_port_status(i);
        send_controller_status();
        break;
    case RAC_MSG_QUERY_STATUS:
        send_controller_status();
        for (uint8_t i = 0; i < RAC_MAX_PORTS; i++) send_port_status(i);
        break;
    case RAC_MSG_DISCOVER:
        for (uint8_t i = 0; i < RAC_MAX_PORTS; i++) discover_start(i);
        break;
    case RAC_MSG_BIND_PORT: {
        REQUIRE_BODY(rac_bind_port_t);
        if (m.port >= RAC_MAX_PORTS) {
            status = RAC_ACK_BAD_ARGUMENT;
        } else {
            status = port_bind(m.port, m.arm_id, m.model, m.can_dialect, m.baudrate);
            send_port_status(m.port);
        }
        break;
    }
    case RAC_MSG_SET_CALIBRATION: {
        REQUIRE_BODY(rac_set_calibration_t);
        if (h.arm_id >= RAC_MAX_ARMS || g.arms[h.arm_id].port == 0xFFu) {
            status = RAC_ACK_BAD_ARGUMENT;
        } else if (g.arms[h.arm_id].torque_on) {
            /* The target is expressed against the calibration, so changing one
             * under an energized arm would move it. */
            status = RAC_ACK_BUSY;
        } else {
            g.arms[h.arm_id].cal = m;
        }
        break;
    }
    case RAC_MSG_SET_LIMITS: {
        REQUIRE_BODY(rac_set_limits_t);
        if (h.arm_id >= RAC_MAX_ARMS) {
            status = RAC_ACK_BAD_ARGUMENT;
        } else {
            g.arms[h.arm_id].limits = m;
        }
        break;
    }
    case RAC_MSG_SET_PAIRING: {
        REQUIRE_BODY(rac_set_pairing_t);
        if (m.pair >= RAC_MAX_PAIRS || m.leader_arm >= RAC_MAX_ARMS || m.follower_arm >= RAC_MAX_ARMS ||
            m.leader_arm == m.follower_arm) {
            status = RAC_ACK_BAD_ARGUMENT;
        } else {
            pair_t *p = &g.pairs[m.pair];
            if (p->phase != RAC_PAIR_OFF) {
                status = RAC_ACK_BUSY;
            } else {
                if (p->configured) {
                    g.arms[p->cfg.leader_arm].pair = 0xFFu;
                    g.arms[p->cfg.follower_arm].pair = 0xFFu;
                }
                p->cfg = m;
                p->configured = 1;
                if (m.enabled) {
                    g.arms[m.leader_arm].pair = m.pair;
                    g.arms[m.follower_arm].pair = m.pair;
                }
            }
        }
        break;
    }
    case RAC_MSG_TELEOP_CONTROL: {
        REQUIRE_BODY(rac_teleop_control_t);
        status = handle_teleop(&m);
        break;
    }
    case RAC_MSG_SET_RATES: {
        REQUIRE_BODY(rac_set_rates_t);
        for (int i = 0; i < RAC_MAX_PORTS; i++) {
            if (m.motor_service_us[i] >= RAC_TICK_US) g.ports[i].service_period_us = m.motor_service_us[i];
            if (m.state_read_us[i] >= RAC_TICK_US) g.ports[i].read_period_us = m.state_read_us[i];
        }
        if (m.upstream_us >= RAC_TICK_US) g.upstream_us = m.upstream_us;
        if (m.host_timeout_ms) g.host_timeout_ms = m.host_timeout_ms;
        break;
    }
    case RAC_MSG_SET_MODE: {
        REQUIRE_BODY(rac_set_mode_t);
        status = handle_set_mode(&h, &m);
        break;
    }
    case RAC_MSG_TARGET_CHUNK:
        /* Strict equality: a chunk from a source that lost control, or from a
         * generation this device has not switched to yet, never executes. */
        if (h.generation != g.generation) {
            send_event(RAC_EVT_STALE_REJECTED, h.arm_id, 1, h.generation, g.generation);
            status = RAC_ACK_STALE_GENERATION;
        } else {
            status = handle_target_chunk(&h, body, body_len, &queue_free);
            if (h.arm_id < RAC_MAX_ARMS) exec = g.arms[h.arm_id].queue.exec_seq;
        }
        break;
    case RAC_MSG_STOP: {
        REQUIRE_BODY(rac_stop_t);
        if (h.generation > g.generation) g.generation = h.generation;
        for (uint8_t i = 0; i < RAC_MAX_ARMS; i++) {
            if (m.arm_mask & (1u << i)) {
                pairs_stop_for_arm(i);
                if (g.arms[i].port != 0xFFu && g.estop_state == RAC_ESTOP_CLEAR) {
                    arm_stop(&g.arms[i], m.kind);
                }
            }
        }
        break;
    }
    case RAC_MSG_PAUSE_RESUME: {
        REQUIRE_BODY(rac_pause_resume_t);
        for (uint8_t i = 0; i < RAC_MAX_ARMS; i++) {
            if (!(m.arm_mask & (1u << i))) continue;
            arm_t *a = &g.arms[i];
            if (m.pause && !a->paused) {
                a->paused = 1;
                a->queue.pause_start = g.now;
            } else if (!m.pause && a->paused) {
                a->paused = 0;
                if (a->queue.pause_start) a->queue.shift_us += g.now - a->queue.pause_start;
                a->queue.pause_start = 0;
            }
        }
        break;
    }
    case RAC_MSG_FEEDFORWARD: {
        REQUIRE_BODY(rac_feedforward_t);
        if (h.arm_id >= RAC_MAX_ARMS) {
            status = RAC_ACK_BAD_ARGUMENT;
        } else {
            memcpy(g.arms[h.arm_id].feedforward, m.torque, sizeof(m.torque));
            g.arms[h.arm_id].ff_expire = g.now + (uint64_t)m.valid_ms * 1000u;
        }
        break;
    }
    case RAC_MSG_TORQUE: {
        REQUIRE_BODY(rac_torque_t);
        if (g.estop_state != RAC_ESTOP_CLEAR && m.enable) {
            status = RAC_ACK_ESTOP_ACTIVE;
        } else {
            for (uint8_t i = 0; i < RAC_MAX_ARMS; i++) {
                if (!(m.arm_mask & (1u << i))) continue;
                arm_t *a = &g.arms[i];
                if (a->port == 0xFFu) continue;
                if (m.enable) {
                    if (!model_can_hold(a->model)) {
                        status = RAC_ACK_UNSUPPORTED;
                    } else if (!arm_full_sample(a)) {
                        status = RAC_ACK_MODE_REJECTED;
                    } else {
                        seed_target(a);
                        arm_set_motion(a, MOTION_POSITION);
                        arm_set_mode(a, RAC_MODE_HOLD, RAC_SOURCE_NONE);
                    }
                } else {
                    pairs_stop_for_arm(i);
                    arm_stop(a, RAC_STOP_RELEASE);
                }
            }
        }
        break;
    }
    case RAC_MSG_RESET_ESTOP:
        if (rac_port_estop_active()) {
            status = RAC_ACK_ESTOP_ACTIVE;
        } else if (g.estop_state != RAC_ESTOP_CLEAR) {
            g.estop_state = RAC_ESTOP_CLEAR;
            g.generation = h.generation;
            for (uint8_t i = 0; i < RAC_MAX_ARMS; i++) {
                if (g.arms[i].port != 0xFFu) {
                    arm_set_motion(&g.arms[i], MOTION_NONE);
                    arm_set_mode(&g.arms[i], RAC_MODE_IDLE, RAC_SOURCE_NONE);
                }
            }
            rac_port_safety_enable(true);
            send_event(RAC_EVT_ESTOP_RESET, RAC_ARM_NONE, 0, 0, 0);
        }
        break;
    default:
        if (!rac_port_handle_extension(h.msg_type, h.arm_id, body, body_len)) {
            status = RAC_ACK_UNSUPPORTED;
        }
        break;
    }
#undef REQUIRE_BODY

    send_ack(&h, status, exec, queue_free);
}

void rac_core_on_host_bytes(const uint8_t *data, size_t len, uint64_t rx_ts_us)
{
    for (size_t i = 0; i < len; i++) {
        size_t n = rac_deframer_push(&g.deframer, data[i], g.rx_payload, sizeof(g.rx_payload));
        if (n) {
            handle_message(g.rx_payload, n, rx_ts_us);
        }
    }
}

/* -------------------------------------------------------------- periodic work */

static void update_estop(uint64_t now)
{
    bool active = rac_port_estop_active();
    (void)now;
    if (active && g.estop_state != RAC_ESTOP_PRESSED) {
        bool was_clear = g.estop_state == RAC_ESTOP_CLEAR;
        g.estop_state = RAC_ESTOP_PRESSED;
        if (was_clear) {
            /* Bump the generation: nothing issued before the press may execute after it. */
            g.generation++;
            send_event(RAC_EVT_ESTOP_PRESSED, RAC_ARM_NONE, 3, 0, 0);
            rac_port_safety_enable(false);
            for (int i = 0; i < RAC_MAX_PAIRS; i++) {
                pair_set_phase(&g.pairs[i], RAC_PAIR_OFF);
            }
            for (uint8_t i = 0; i < RAC_MAX_ARMS; i++) {
                arm_t *a = &g.arms[i];
                if (a->port == 0xFFu) continue;
                queue_reset(a, true);
                port_t *p = &g.ports[a->port];
                /* An arm that was released stays released: an emergency stop must
                 * never be the thing that energizes a motor. */
                if (a->torque_on) {
                    if (p->kind == RAC_PORT_KIND_CAN) {
                        arm_set_motion(a, MOTION_DAMP);
                    } else {
                        port_send_stop_star(p, FS_STOP_DAMP);
                        a->torque_on = 0;
                        a->motion = MOTION_NONE;
                        a->target_valid = 0;
                    }
                }
                arm_set_mode(a, RAC_MODE_ESTOP, RAC_SOURCE_NONE);
            }
        }
    } else if (!active && g.estop_state == RAC_ESTOP_PRESSED) {
        g.estop_state = RAC_ESTOP_LATCHED;
        send_event(RAC_EVT_ESTOP_RELEASED, RAC_ARM_NONE, 1, 0, 0);
    }
}

static void update_host_watchdog(uint64_t now)
{
    if (!g.host_alive) return;
    if (now - g.last_host_rx > (uint64_t)g.host_timeout_ms * 1000u) {
        g.host_alive = 0;
        send_event(RAC_EVT_HOST_LOST, RAC_ARM_NONE, 2, 0, 0);
        /* Without a host nobody can supervise motion, so everything holds. The
         * generation moves on so nothing queued before the loss runs after it. */
        g.generation++;
        if (g.estop_state == RAC_ESTOP_CLEAR) {
            stop_all(RAC_STOP_HOLD);
        }
    }
}

void rac_core_init(void)
{
    memset(&g, 0, sizeof(g));
    uint8_t uid[12];
    rac_port_unique_id(uid);
    g.device_id = rac_crc16(uid, sizeof(uid));
    g.upstream_us = 4000u;
    g.host_timeout_ms = DEFAULT_HOST_TIMEOUT_MS;
    g.generation = 0;
    rac_deframer_init(&g.deframer);

    for (uint8_t i = 0; i < RAC_MAX_ARMS; i++) {
        g.arms[i].port = 0xFFu;
        g.arms[i].pair = 0xFFu;
        g.arms[i].is_leader = (i == RAC_ARM_LEFT_LEADER || i == RAC_ARM_RIGHT_LEADER) ? 1u : 0u;
    }
    for (uint8_t i = 0; i < RAC_MAX_PORTS; i++) {
        port_t *p = &g.ports[i];
        p->kind = i < 2 ? RAC_PORT_KIND_CAN : RAC_PORT_KIND_UART;
        p->bus = i < 2 ? i : (uint8_t)(i - 2u);
        p->arm = RAC_ARM_NONE;
        p->dialect = RAC_CAN_DIALECT_RS_PRIVATE;
        p->baudrate = i < 2 ? 1000000u : 1000000u;
        /* 250 Hz on a CAN arm is under 50% bus load with 7 motors and replies;
         * a Star exchange takes about 2.5 ms of the 8 ms slot. */
        p->service_period_us = i < 2 ? 4000u : 8000u;
        p->read_period_us = i < 2 ? 10000u : 8000u;
        fs_parser_reset(&p->parser);
    }
    rac_port_safety_enable(!rac_port_estop_active());
}

void rac_core_tick(uint64_t now_us)
{
    g.now = now_us;

    if (!g.booted) {
        g.booted = 1;
        g.next_upstream = now_us;
        g.next_status = now_us;
        g.next_port_status = now_us;
        g.next_tick_expected = now_us;
        send_event(RAC_EVT_BOOT, RAC_ARM_NONE, 0, RAC_FW_MAJOR, RAC_FW_MINOR);
    }

    /* Lateness of this tick against its slot, kept as a health statistic. */
    if (now_us > g.next_tick_expected + RAC_TICK_US) {
        g.overrun_count++;
    }
    g.next_tick_expected = now_us + RAC_TICK_US;
    g.tick_count++;

    update_estop(now_us);
    update_host_watchdog(now_us);
    update_pairs(now_us);
    service_ports(now_us);

    if (now_us >= g.next_upstream) {
        g.next_upstream = now_us + g.upstream_us;
        flush_samples();
    }
    if (now_us >= g.next_status) {
        g.next_status = now_us + STATUS_PERIOD_US;
        send_controller_status();
    }
    if (now_us >= g.next_port_status) {
        g.next_port_status = now_us + PORT_STATUS_PERIOD_US;
        for (uint8_t i = 0; i < RAC_MAX_PORTS; i++) {
            send_port_status(i);
            const port_t *p = &g.ports[i];
            rac_port_led_status(i, p->online_mask != 0,
                                p->arm != RAC_ARM_NONE && g.arms[p->arm].torque_on);
        }
    }

    g.led_counter++;
    rac_port_led_run((g.led_counter / (1000000u / RAC_TICK_US / 2u)) & 1u);
    rac_port_led_fault(g.estop_state != RAC_ESTOP_CLEAR || !g.host_alive);
    rac_port_watchdog_kick();

    uint64_t elapsed = rac_port_now_us() - now_us;
    if (elapsed > g.max_tick_us) g.max_tick_us = (uint32_t)elapsed;
}

void rac_core_debug_arm(uint8_t arm, rac_arm_debug_t *out)
{
    const arm_t *a = &g.arms[arm];
    out->mode = a->mode;
    out->source = a->source;
    out->torque_on = a->torque_on;
    out->valid_mask = a->valid_mask;
    out->generation = g.generation;
    out->queue_depth = a->queue.count;
    memcpy(out->position, a->position, sizeof(out->position));
    memcpy(out->target, a->target, sizeof(out->target));
}

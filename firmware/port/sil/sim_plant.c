#include "sim_plant.h"

#include <math.h>
#include <string.h>

#include "drv_fashionstar.h"
#include "drv_robstride.h"
#include "rac_core.h"
#include "rac_wire.h"

#define J RAC_JOINTS
#define PI_F 3.14159265358979f
#define CAN_FRAME_US 130u
#define UART_BYTE_US 10u
#define SERVO_REPLY_DELAY_US 20u
#define MAX_EVENTS 512u
#define STEP_US 500u

typedef struct {
    float pos, vel;
    float ref, vref, kp, kd, tau;
    float goal;
    uint8_t enabled;
    uint8_t online;
    uint8_t star_mode; /* 0 released, 1 position, 2 damped */
} sim_motor_t;

typedef struct {
    uint8_t model;
    uint8_t dialect;
    uint8_t online;
    uint8_t hand_active;
    uint8_t hand_sine;
    float hand_amp;
    uint64_t hand_start;
    float hand_pose[J];
    sim_motor_t motors[J];
    uint64_t busy_until;
    uint32_t can_sent;
    uint8_t rx_state;
    uint8_t rx_buf[FS_MAX_FRAME];
    uint16_t rx_len;
    uint16_t rx_need;
} sim_port_t;

typedef struct {
    uint64_t at;
    uint8_t port;
    uint8_t is_uart;
    uint8_t len;
    rac_can_frame_t can;
    uint8_t bytes[32];
} sim_event_t;

static sim_port_t ports[RAC_MAX_PORTS];
static sim_event_t events[MAX_EVENTS];
static uint32_t event_count;
static uint64_t sim_now;
static uint64_t last_step;
static sim_host_sink_t host_sink;
static uint8_t estop;

static void push_event(const sim_event_t *e)
{
    if (event_count < MAX_EVENTS) {
        events[event_count++] = *e;
    }
}

static rs_motor_type_t type_of_joint(int j)
{
    return j < 3 ? RS_TYPE_RS06 : RS_TYPE_RS00;
}

static rs_motor_type_t type_of_id(uint8_t id)
{
    return (id >= 1 && id <= 3) ? RS_TYPE_RS06 : RS_TYPE_RS00;
}

/* ------------------------------------------------------------ RobStride side */

static void rs_reply(uint8_t port_index, int joint, uint64_t start_at)
{
    sim_port_t *p = &ports[port_index];
    sim_motor_t *m = &p->motors[joint];
    const rs_limits_t *l = rs_limits(type_of_joint(joint));
    sim_event_t e;
    uint8_t id = (uint8_t)(joint + 1);
    memset(&e, 0, sizeof(e));
    e.port = port_index;
    e.at = start_at + CAN_FRAME_US;

    if (p->dialect == RAC_CAN_DIALECT_RS_PRIVATE) {
        uint32_t field = (uint32_t)id | ((uint32_t)(m->enabled ? 2u : 0u) << 14);
        uint32_t pos = rs_float_to_uint(m->pos, -l->p_max, l->p_max, 16);
        uint32_t vel = rs_float_to_uint(m->vel, -l->v_max, l->v_max, 16);
        uint32_t tor = rs_float_to_uint(m->tau, -l->t_max, l->t_max, 16);
        e.can.id = ((uint32_t)RS_COMM_FEEDBACK << 24) | (field << 8) | RS_DEFAULT_HOST_ID;
        e.can.extended = 1;
        e.can.dlc = 8;
        e.can.data[0] = (uint8_t)(pos >> 8);
        e.can.data[1] = (uint8_t)pos;
        e.can.data[2] = (uint8_t)(vel >> 8);
        e.can.data[3] = (uint8_t)vel;
        e.can.data[4] = (uint8_t)(tor >> 8);
        e.can.data[5] = (uint8_t)tor;
        e.can.data[6] = 0x01; /* 30.0 degrees C */
        e.can.data[7] = 0x2C;
    } else {
        uint32_t pos = rs_float_to_uint(m->pos, -l->p_max, l->p_max, 16);
        uint32_t vel = rs_float_to_uint(m->vel, -l->v_max, l->v_max, 12);
        uint32_t tor = rs_float_to_uint(m->tau, -l->t_max, l->t_max, 12);
        e.can.id = 0x100u + id;
        e.can.extended = 0;
        e.can.dlc = 8;
        e.can.data[0] = id;
        e.can.data[1] = (uint8_t)(pos >> 8);
        e.can.data[2] = (uint8_t)pos;
        e.can.data[3] = (uint8_t)(vel >> 4);
        e.can.data[4] = (uint8_t)(((vel & 0xFu) << 4) | (tor >> 8));
        e.can.data[5] = (uint8_t)tor;
        e.can.data[6] = 0x01;
        e.can.data[7] = 0x2C;
    }
    push_event(&e);
}

static void rs_receive(uint8_t port_index, const rac_can_frame_t *f, uint64_t done_at)
{
    sim_port_t *p = &ports[port_index];
    if (p->model != RAC_MODEL_REBOT_B601_RS || !p->online) return;

    int joint = -1;
    uint8_t reply = 0;

    if (f->extended) {
        /* Motors in the MIT protocol ignore 29-bit frames, and the reverse. */
        if (p->dialect != RAC_CAN_DIALECT_RS_PRIVATE) return;
        uint32_t comm = (f->id >> 24) & 0x1Fu;
        uint8_t id = (uint8_t)(f->id & 0xFFu);
        if (id < 1 || id > J) return;
        joint = id - 1;
        sim_motor_t *m = &p->motors[joint];
        if (!m->online) return;
        const rs_limits_t *l = rs_limits(type_of_id(id));
        if (comm == RS_COMM_MOTION) {
            uint32_t raw_tau = (f->id >> 8) & 0xFFFFu;
            m->tau = rs_uint_to_float(raw_tau, -l->t_max, l->t_max, 16);
            m->ref = rs_uint_to_float(((uint32_t)f->data[0] << 8) | f->data[1], -l->p_max, l->p_max, 16);
            m->vref = rs_uint_to_float(((uint32_t)f->data[2] << 8) | f->data[3], -l->v_max, l->v_max, 16);
            m->kp = rs_uint_to_float(((uint32_t)f->data[4] << 8) | f->data[5], 0.0f, l->kp_max, 16);
            m->kd = rs_uint_to_float(((uint32_t)f->data[6] << 8) | f->data[7], 0.0f, l->kd_max, 16);
            reply = 1;
        } else if (comm == RS_COMM_ENABLE) {
            m->enabled = 1;
            m->ref = m->pos;
            m->kp = 0;
            m->kd = 0;
            m->tau = 0;
            reply = 1;
        } else if (comm == RS_COMM_STOP) {
            m->enabled = 0;
            m->tau = 0;
            reply = 1;
        }
    } else {
        if (p->dialect != RAC_CAN_DIALECT_RS_MIT) return;
        uint8_t id = (uint8_t)(f->id & 0x7Fu);
        if (id < 1 || id > J) return;
        joint = id - 1;
        sim_motor_t *m = &p->motors[joint];
        if (!m->online) return;
        const rs_limits_t *l = rs_limits(type_of_id(id));
        bool is_command = f->data[0] == 0xFF && f->data[1] == 0xFF && f->data[2] == 0xFF &&
                          f->data[3] == 0xFF && f->data[4] == 0xFF && f->data[5] == 0xFF &&
                          f->data[6] == 0xFF;
        if (is_command) {
            if (f->data[7] == RS_MIT_ENABLE) {
                m->enabled = 1;
                m->ref = m->pos;
                m->kp = 0;
                m->kd = 0;
                m->tau = 0;
            } else if (f->data[7] == RS_MIT_DISABLE) {
                m->enabled = 0;
                m->tau = 0;
            }
            reply = 1;
        } else {
            uint32_t pos = ((uint32_t)f->data[0] << 8) | f->data[1];
            uint32_t vel = ((uint32_t)f->data[2] << 4) | (f->data[3] >> 4);
            uint32_t kp = (((uint32_t)f->data[3] & 0xFu) << 8) | f->data[4];
            uint32_t kd = ((uint32_t)f->data[5] << 4) | (f->data[6] >> 4);
            uint32_t tor = (((uint32_t)f->data[6] & 0xFu) << 8) | f->data[7];
            m->ref = rs_uint_to_float(pos, -l->p_max, l->p_max, 16);
            m->vref = rs_uint_to_float(vel, -l->v_max, l->v_max, 12);
            m->kp = rs_uint_to_float(kp, 0.0f, l->kp_max, 12);
            m->kd = rs_uint_to_float(kd, 0.0f, l->kd_max, 12);
            m->tau = rs_uint_to_float(tor, -l->t_max, l->t_max, 12);
            reply = 1;
        }
    }

    if (reply && joint >= 0) {
        uint64_t start = p->busy_until > done_at ? p->busy_until : done_at;
        p->busy_until = start + CAN_FRAME_US;
        rs_reply(port_index, joint, start);
    }
}

bool rac_port_can_send(uint8_t can_index, const rac_can_frame_t *frame)
{
    sim_port_t *p = &ports[can_index];
    uint64_t start = p->busy_until > sim_now ? p->busy_until : sim_now;
    sim_event_t e;
    memset(&e, 0, sizeof(e));
    e.at = start + CAN_FRAME_US;
    e.port = can_index;
    e.can = *frame;
    e.len = 0xFF; /* marks a frame travelling towards the motors */
    p->busy_until = e.at;
    p->can_sent++;
    push_event(&e);
    return true;
}

/* ----------------------------------------------------------- Fashion Star side */

static void uart_reply(uint8_t port_index, const uint8_t *frame, size_t len, uint64_t at)
{
    sim_event_t e;
    memset(&e, 0, sizeof(e));
    e.at = at + len * UART_BYTE_US;
    e.port = port_index;
    e.is_uart = 1;
    e.len = (uint8_t)len;
    memcpy(e.bytes, frame, len);
    push_event(&e);
}

static void star_request(uint8_t port_index, const uint8_t *frame, uint64_t at)
{
    sim_port_t *p = &ports[port_index];
    uint8_t code = frame[2];
    uint8_t size = frame[3];
    const uint8_t *params = frame + 4;
    uint8_t out[40];

    if (!p->online || p->model == RAC_MODEL_NONE) return;

    if (code == FS_CODE_PING) {
        uint8_t id = params[0];
        bool present = (id < J && p->motors[id].online) ||
                       (id == 7 && p->model == RAC_MODEL_STAR_102_HD);
        if (present) {
            size_t n = fs_build_response(out, FS_CODE_PING, &id, 1);
            uint64_t start = p->busy_until > at ? p->busy_until : at;
            p->busy_until = start + n * UART_BYTE_US + SERVO_REPLY_DELAY_US;
            uart_reply(port_index, out, n, start + SERVO_REPLY_DELAY_US);
        }
    } else if (code == FS_CODE_SYNC && size >= 3) {
        uint8_t inner = params[0];
        uint8_t record = params[1];
        uint8_t count = params[2];
        const uint8_t *r = params + 3;
        if (inner == FS_CODE_MONITOR) {
            for (uint8_t i = 0; i < count; i++) {
                uint8_t id = r[i];
                if (id >= J || !p->motors[id].online) continue;
                size_t n = fs_build_monitor_response(out, id, p->motors[id].pos * 180.0f / PI_F);
                uint64_t start = p->busy_until > at ? p->busy_until : at;
                p->busy_until = start + n * UART_BYTE_US + SERVO_REPLY_DELAY_US;
                uart_reply(port_index, out, n, start + SERVO_REPLY_DELAY_US);
            }
        } else if (inner == FS_CODE_SET_ANGLE_MTURN && record == 11) {
            for (uint8_t i = 0; i < count; i++, r += record) {
                uint8_t id = r[0];
                if (id >= J || !p->motors[id].online) continue;
                int32_t tenths = (int32_t)((uint32_t)r[1] | ((uint32_t)r[2] << 8) | ((uint32_t)r[3] << 16) |
                                           ((uint32_t)r[4] << 24));
                p->motors[id].goal = (float)tenths * 0.1f * PI_F / 180.0f;
                p->motors[id].star_mode = 1;
                p->motors[id].enabled = 1;
            }
        }
    } else if (code == FS_CODE_STOP_ON_CONTROL && size >= 2) {
        uint8_t id = params[0];
        if (id < J) {
            p->motors[id].star_mode = params[1] == FS_STOP_HOLD ? 1u : (params[1] == FS_STOP_DAMP ? 2u : 0u);
            p->motors[id].enabled = p->motors[id].star_mode == 1;
        }
    }
}

bool rac_port_uart_send(uint8_t uart_index, const uint8_t *data, size_t len)
{
    uint8_t port_index = (uint8_t)(uart_index + 2u);
    sim_port_t *p = &ports[port_index];
    uint64_t start = p->busy_until > sim_now ? p->busy_until : sim_now;
    uint64_t t = start;

    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        t += UART_BYTE_US;
        if (p->rx_state == 0) {
            if (b == 0x12) {
                p->rx_buf[0] = b;
                p->rx_state = 1;
            }
        } else if (p->rx_state == 1) {
            if (b == 0x4C) {
                p->rx_buf[1] = b;
                p->rx_len = 2;
                p->rx_state = 2;
            } else {
                p->rx_state = 0;
            }
        } else {
            p->rx_buf[p->rx_len++] = b;
            if (p->rx_len == 4) {
                p->rx_need = (uint16_t)(5u + p->rx_buf[3]);
            }
            if (p->rx_len >= 4 && p->rx_len == p->rx_need) {
                p->rx_state = 0;
                if (fs_checksum(p->rx_buf, p->rx_need - 1u) == p->rx_buf[p->rx_need - 1u]) {
                    p->busy_until = t;
                    star_request(port_index, p->rx_buf, t);
                    if (p->busy_until > t) t = p->busy_until;
                }
            }
        }
    }
    if (t > p->busy_until) p->busy_until = t;
    return true;
}

void rac_port_uart_set_baud(uint8_t uart_index, uint32_t baud)
{
    (void)uart_index;
    (void)baud;
}

/* ------------------------------------------------------------------- physics */

static float gravity_torque(int joint, float q)
{
    /* Enough to show the difference gravity compensation makes. */
    if (joint == 1) return 6.0f * cosf(q);
    if (joint == 2) return 3.0f * cosf(q);
    return 0.0f;
}

static void step_motor(sim_port_t *p, int j, float dt, float hand)
{
    sim_motor_t *m = &p->motors[j];
    bool hand_moved = p->hand_active && !(m->enabled && m->kp > 0.0f);

    if (p->model == RAC_MODEL_REBOT_B601_RS) {
        if (hand_moved) {
            float v = (hand - m->pos) * 8.0f;
            m->vel = v;
            m->pos += v * dt;
            return;
        }
        if (!m->enabled) {
            m->vel = 0.0f;
            return;
        }
        float inertia = j < 3 ? 0.06f : 0.008f;
        const rs_limits_t *l = rs_limits(type_of_joint(j));
        float cmd = m->kp * (m->ref - m->pos) + m->kd * (m->vref - m->vel) + m->tau;
        if (cmd > l->t_max) cmd = l->t_max;
        if (cmd < -l->t_max) cmd = -l->t_max;
        float accel = (cmd - gravity_torque(j, m->pos) - 0.3f * m->vel) / inertia;
        m->vel += accel * dt;
        m->pos += m->vel * dt;
    } else {
        if (hand_moved && m->star_mode != 1) {
            float v = (hand - m->pos) * 8.0f;
            m->vel = v;
            m->pos += v * dt;
            return;
        }
        if (m->star_mode == 1) {
            float e = m->goal - m->pos;
            float v = e * 30.0f;
            if (v > 6.0f) v = 6.0f;
            if (v < -6.0f) v = -6.0f;
            m->vel = v;
            m->pos += v * dt;
        } else {
            m->vel = 0.0f;
        }
    }
}

static void step_physics(uint64_t now)
{
    float dt = (float)STEP_US * 1e-6f;
    for (int i = 0; i < RAC_MAX_PORTS; i++) {
        sim_port_t *p = &ports[i];
        if (p->model == RAC_MODEL_NONE) continue;
        for (int j = 0; j < J; j++) {
            float hand = p->hand_pose[j];
            if (p->hand_sine) {
                float t = (float)(now - p->hand_start) * 1e-6f;
                hand += p->hand_amp * sinf(2.0f * PI_F * 0.2f * t + (float)j * 0.5f);
            }
            step_motor(p, j, dt, hand);
        }
    }
}

void sim_advance(uint64_t now_us)
{
    /* Physics in fixed steps so the integrator does not depend on host jitter. */
    while (last_step + STEP_US <= now_us) {
        last_step += STEP_US;
        step_physics(last_step);
    }
    sim_now = now_us;

    /* Deliver in time order. The list stays short, so a scan is fine. */
    for (;;) {
        int best = -1;
        for (uint32_t i = 0; i < event_count; i++) {
            if (events[i].at <= now_us && (best < 0 || events[i].at < events[best].at)) {
                best = (int)i;
            }
        }
        if (best < 0) break;
        sim_event_t e = events[best];
        events[best] = events[--event_count];
        if (e.is_uart) {
            rac_core_on_uart_bytes((uint8_t)(e.port - 2u), e.bytes, e.len, e.at);
        } else if (e.len == 0xFF) {
            rs_receive(e.port, &e.can, e.at);
        } else {
            rac_core_on_can_frame(e.port, &e.can, e.at);
        }
    }
}

/* ---------------------------------------------------------------- setup, port */

void sim_set_model(uint8_t port, uint8_t model)
{
    sim_port_t *p = &ports[port];
    p->model = model;
    p->online = model != RAC_MODEL_NONE;
    for (int j = 0; j < J; j++) {
        p->motors[j].online = model != RAC_MODEL_NONE;
    }
}

void sim_init(void)
{
    memset(ports, 0, sizeof(ports));
    event_count = 0;
    sim_now = 0;
    last_step = 0;
    estop = 0;
    for (uint8_t i = 0; i < RAC_MAX_PORTS; i++) {
        ports[i].dialect = RAC_CAN_DIALECT_RS_PRIVATE;
    }
}

void sim_set_host_sink(sim_host_sink_t sink)
{
    host_sink = sink;
}

void sim_inject(const rac_sil_inject_t *m)
{
    uint8_t arg = m->arg < RAC_MAX_PORTS ? m->arg : 0;
    sim_port_t *p = &ports[arg];
    switch (m->op) {
    case RAC_SIL_SET_HAND_POSE:
        p->hand_active = 1;
        p->hand_sine = 0;
        memcpy(p->hand_pose, m->values, sizeof(p->hand_pose));
        break;
    case RAC_SIL_HAND_SINE:
        p->hand_active = 1;
        p->hand_sine = m->values[0] != 0.0f;
        p->hand_amp = m->values[0];
        p->hand_start = sim_now;
        break;
    case RAC_SIL_PRESS_ESTOP:
        estop = 1;
        break;
    case RAC_SIL_RELEASE_ESTOP:
        estop = 0;
        break;
    case RAC_SIL_DROP_MOTOR:
    case RAC_SIL_RESTORE_MOTOR:
        for (int j = 0; j < J; j++) {
            if (m->arg2 & (1u << j)) p->motors[j].online = m->op == RAC_SIL_RESTORE_MOTOR;
        }
        break;
    case RAC_SIL_DROP_PORT:
        p->online = 0;
        break;
    case RAC_SIL_RESTORE_PORT:
        p->online = 1;
        break;
    case RAC_SIL_SET_MODEL:
        sim_set_model(arg, m->arg2);
        break;
    default:
        break;
    }
}

float sim_motor_position(uint8_t port, uint8_t joint)
{
    return ports[port].motors[joint].pos;
}

uint32_t sim_can_frames_sent(uint8_t bus)
{
    return ports[bus].can_sent;
}

uint64_t rac_port_now_us(void)
{
    return sim_now;
}

bool rac_port_host_send(const uint8_t *data, size_t len)
{
    if (host_sink) host_sink(data, len);
    return true;
}

bool rac_port_estop_active(void)
{
    return estop != 0;
}

void rac_port_led_status(uint8_t port_index, bool online, bool active)
{
    (void)port_index;
    (void)online;
    (void)active;
}

void rac_port_led_run(bool on)
{
    (void)on;
}

void rac_port_led_fault(bool on)
{
    (void)on;
}

void rac_port_safety_enable(bool enable)
{
    (void)enable;
}

void rac_port_sync_pulse(bool level)
{
    (void)level;
}

void rac_port_watchdog_kick(void)
{
}

void rac_port_unique_id(uint8_t out[12])
{
    static const uint8_t uid[12] = {'R', 'A', 'C', '-', 'S', 'I', 'L', 0, 0, 0, 0, 1};
    memcpy(out, uid, 12);
}

uint32_t rac_port_capabilities(void)
{
    return RAC_CAP_SIL | RAC_CAP_HW_ESTOP;
}

bool rac_port_handle_extension(uint8_t msg_type, uint8_t arm_id, const uint8_t *body, size_t len)
{
    (void)arm_id;
    if (msg_type == RAC_MSG_SIL_INJECT && len == sizeof(rac_sil_inject_t)) {
        rac_sil_inject_t m;
        memcpy(&m, body, sizeof(m));
        sim_inject(&m);
        return true;
    }
    return false;
}

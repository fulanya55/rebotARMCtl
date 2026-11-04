#include "drv_robstride.h"

#include <string.h>

/* Numeric ranges per the RobStride manual: position, velocity, torque, kp, kd. */
static const rs_limits_t LIMITS[] = {
    [RS_TYPE_RS00] = {12.57f, 33.0f, 14.0f, 500.0f, 5.0f},
    [RS_TYPE_RS06] = {12.57f, 50.0f, 36.0f, 5000.0f, 100.0f},
};

const rs_limits_t *rs_limits(rs_motor_type_t type)
{
    return &LIMITS[type];
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

uint32_t rs_float_to_uint(float value, float min, float max, int bits)
{
    float span = max - min;
    float clamped = clampf(value, min, max);
    float scale = (float)((1u << bits) - 1u);
    return (uint32_t)((clamped - min) * scale / span + 0.5f);
}

float rs_uint_to_float(uint32_t value, float min, float max, int bits)
{
    float span = max - min;
    float scale = (float)((1u << bits) - 1u);
    return (float)value * span / scale + min;
}

static uint32_t private_id(uint32_t comm_type, uint32_t data16, uint8_t low8)
{
    return ((comm_type & 0x1Fu) << 24) | ((data16 & 0xFFFFu) << 8) | low8;
}

void rs_encode_motion(rac_can_frame_t *out, uint8_t dialect, rs_motor_type_t type, uint8_t motor_id,
                      float position, float velocity, float kp, float kd, float torque)
{
    const rs_limits_t *l = rs_limits(type);
    memset(out, 0, sizeof(*out));
    out->dlc = 8;

    if (dialect == 0u) { /* private: torque travels in the identifier */
        uint32_t t = rs_float_to_uint(torque, -l->t_max, l->t_max, 16);
        uint32_t p = rs_float_to_uint(position, -l->p_max, l->p_max, 16);
        uint32_t v = rs_float_to_uint(velocity, -l->v_max, l->v_max, 16);
        uint32_t k = rs_float_to_uint(kp, 0.0f, l->kp_max, 16);
        uint32_t d = rs_float_to_uint(kd, 0.0f, l->kd_max, 16);
        out->id = private_id(RS_COMM_MOTION, t, motor_id);
        out->extended = 1;
        out->data[0] = (uint8_t)(p >> 8);
        out->data[1] = (uint8_t)p;
        out->data[2] = (uint8_t)(v >> 8);
        out->data[3] = (uint8_t)v;
        out->data[4] = (uint8_t)(k >> 8);
        out->data[5] = (uint8_t)k;
        out->data[6] = (uint8_t)(d >> 8);
        out->data[7] = (uint8_t)d;
    } else { /* MIT: 16 bit position, 12 bit velocity, kp, kd and torque */
        uint32_t p = rs_float_to_uint(position, -l->p_max, l->p_max, 16);
        uint32_t v = rs_float_to_uint(velocity, -l->v_max, l->v_max, 12);
        uint32_t k = rs_float_to_uint(kp, 0.0f, l->kp_max, 12);
        uint32_t d = rs_float_to_uint(kd, 0.0f, l->kd_max, 12);
        uint32_t t = rs_float_to_uint(torque, -l->t_max, l->t_max, 12);
        out->id = motor_id;
        out->extended = 0;
        out->data[0] = (uint8_t)(p >> 8);
        out->data[1] = (uint8_t)p;
        out->data[2] = (uint8_t)(v >> 4);
        out->data[3] = (uint8_t)(((v & 0xFu) << 4) | (k >> 8));
        out->data[4] = (uint8_t)k;
        out->data[5] = (uint8_t)(d >> 4);
        out->data[6] = (uint8_t)(((d & 0xFu) << 4) | (t >> 8));
        out->data[7] = (uint8_t)t;
    }
}

static void mit_command(rac_can_frame_t *out, uint8_t motor_id, uint8_t code)
{
    memset(out, 0, sizeof(*out));
    out->id = motor_id;
    out->extended = 0;
    out->dlc = 8;
    memset(out->data, 0xFF, 7);
    out->data[7] = code;
}

void rs_encode_set_motion_mode(rac_can_frame_t *out, uint8_t motor_id, uint8_t host_id)
{
    memset(out, 0, sizeof(*out));
    out->id = private_id(RS_COMM_PARAM_WRITE, host_id, motor_id);
    out->extended = 1;
    out->dlc = 8;
    out->data[0] = (uint8_t)(RS_PARAM_RUN_MODE & 0xFFu);
    out->data[1] = (uint8_t)(RS_PARAM_RUN_MODE >> 8);
    /* value 0 selects motion control */
}

void rs_encode_enable(rac_can_frame_t *out, uint8_t dialect, uint8_t motor_id, uint8_t host_id)
{
    if (dialect == 0u) {
        memset(out, 0, sizeof(*out));
        out->id = private_id(RS_COMM_ENABLE, host_id, motor_id);
        out->extended = 1;
        out->dlc = 8;
    } else {
        mit_command(out, motor_id, RS_MIT_ENABLE);
    }
}

void rs_encode_disable(rac_can_frame_t *out, uint8_t dialect, uint8_t motor_id, uint8_t host_id,
                       bool clear_fault)
{
    if (dialect == 0u) {
        memset(out, 0, sizeof(*out));
        out->id = private_id(RS_COMM_STOP, host_id, motor_id);
        out->extended = 1;
        out->dlc = 8;
        out->data[0] = clear_fault ? 1u : 0u;
    } else {
        mit_command(out, motor_id, RS_MIT_DISABLE);
    }
}

void rs_encode_query(rac_can_frame_t *out, uint8_t dialect, uint8_t motor_id, uint8_t host_id)
{
    if (dialect == 0u) {
        /* A stop with clear_fault = 0 changes nothing on a released motor and
         * is answered with a feedback frame. */
        rs_encode_disable(out, dialect, motor_id, host_id, false);
    } else {
        mit_command(out, motor_id, RS_MIT_QUERY);
    }
}

bool rs_decode_feedback(const rac_can_frame_t *frame, uint8_t dialect, uint8_t host_id,
                        rs_motor_type_t (*type_of)(uint8_t motor_id), rs_feedback_t *out)
{
    if (frame->dlc < 8) {
        return false;
    }
    const uint8_t *d = frame->data;

    if (dialect == 0u) {
        if (!frame->extended) return false;
        uint32_t comm = (frame->id >> 24) & 0x1Fu;
        if (comm != RS_COMM_FEEDBACK) return false;
        if ((frame->id & 0xFFu) != host_id) return false;
        uint32_t field = (frame->id >> 8) & 0xFFFFu;
        uint8_t motor = (uint8_t)(field & 0xFFu);
        const rs_limits_t *l = rs_limits(type_of(motor));
        out->motor_id = motor;
        out->fault_bits = (uint8_t)((field >> 8) & 0x3Fu);
        out->mode_status = (uint8_t)((field >> 14) & 0x03u);
        out->position = rs_uint_to_float(((uint32_t)d[0] << 8) | d[1], -l->p_max, l->p_max, 16);
        out->velocity = rs_uint_to_float(((uint32_t)d[2] << 8) | d[3], -l->v_max, l->v_max, 16);
        out->torque = rs_uint_to_float(((uint32_t)d[4] << 8) | d[5], -l->t_max, l->t_max, 16);
        out->temperature = (float)(((uint32_t)d[6] << 8) | d[7]) / 10.0f;
        return true;
    }

    /* MIT feedback: byte 0 is the motor id, then 16/12/12 bit fields and temperature. */
    if (frame->extended) return false;
    uint8_t motor = d[0];
    if (motor == 0 || motor > 0x7Fu) return false;
    const rs_limits_t *l = rs_limits(type_of(motor));
    out->motor_id = motor;
    out->fault_bits = 0;
    out->mode_status = 0;
    out->position = rs_uint_to_float(((uint32_t)d[1] << 8) | d[2], -l->p_max, l->p_max, 16);
    out->velocity = rs_uint_to_float(((uint32_t)d[3] << 4) | (d[4] >> 4), -l->v_max, l->v_max, 12);
    out->torque = rs_uint_to_float((((uint32_t)d[4] & 0xFu) << 8) | d[5], -l->t_max, l->t_max, 12);
    out->temperature = (float)(((uint32_t)d[6] << 8) | d[7]) / 10.0f;
    return true;
}

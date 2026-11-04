/*
 * RobStride RS00 / RS06 CAN driver (classic CAN, 1 Mbit/s).
 *
 * Written from the protocol, not from vendor sources: the reference repository
 * RobStride/RobStrideSPC declares no license, so only the frame layout and the
 * numeric ranges it documents are used. See docs/LICENSE_AUDIT.md.
 *
 * Two dialects are supported because both are in the field:
 *   - private protocol: 29-bit identifiers, the factory default, what the
 *     reBot B601-RS stack drives (feedback_id 0xFD);
 *   - MIT protocol: 11-bit identifiers, what the MakerMods Maker arm drives.
 *
 * Pure encode and decode only. Scheduling lives in rac_core.c.
 */
#ifndef DRV_ROBSTRIDE_H
#define DRV_ROBSTRIDE_H

#include <stdbool.h>
#include <stdint.h>

#include "rac_port.h"

typedef enum {
    RS_TYPE_RS00 = 0,
    RS_TYPE_RS06 = 1,
} rs_motor_type_t;

typedef struct {
    float p_max, v_max, t_max, kp_max, kd_max;
} rs_limits_t;

const rs_limits_t *rs_limits(rs_motor_type_t type);

typedef struct {
    uint8_t motor_id;
    uint8_t fault_bits;
    uint8_t mode_status;
    float position; /* rad */
    float velocity; /* rad/s */
    float torque;   /* N*m */
    float temperature; /* degrees C */
} rs_feedback_t;

/* Motion control frame: position, velocity, kp, kd and feedforward torque. */
void rs_encode_motion(rac_can_frame_t *out, uint8_t dialect, rs_motor_type_t type, uint8_t motor_id,
                      float position, float velocity, float kp, float kd, float torque);

/* Private protocol only: write run_mode = motion control. Needs a released motor. */
void rs_encode_set_motion_mode(rac_can_frame_t *out, uint8_t motor_id, uint8_t host_id);

void rs_encode_enable(rac_can_frame_t *out, uint8_t dialect, uint8_t motor_id, uint8_t host_id);
void rs_encode_disable(rac_can_frame_t *out, uint8_t dialect, uint8_t motor_id, uint8_t host_id,
                       bool clear_fault);

/*
 * Read-only status request. Answered by a feedback frame and safe on a motor
 * with torque released, so it is what discovery and idle reading use.
 */
void rs_encode_query(rac_can_frame_t *out, uint8_t dialect, uint8_t motor_id, uint8_t host_id);

/* Returns true when the frame is feedback from a motor. */
bool rs_decode_feedback(const rac_can_frame_t *frame, uint8_t dialect, uint8_t host_id,
                        rs_motor_type_t (*type_of)(uint8_t motor_id), rs_feedback_t *out);

/* Shared with the simulator, which is the other end of the same protocol. */
float rs_uint_to_float(uint32_t value, float min, float max, int bits);
uint32_t rs_float_to_uint(float value, float min, float max, int bits);

#define RS_DEFAULT_HOST_ID 0xFDu

/* Private protocol communication types, carried in identifier bits 24..28. */
#define RS_COMM_GET_ID 0u
#define RS_COMM_MOTION 1u
#define RS_COMM_FEEDBACK 2u
#define RS_COMM_ENABLE 3u
#define RS_COMM_STOP 4u
#define RS_COMM_PARAM_WRITE 18u

#define RS_PARAM_RUN_MODE 0x7005u

/* MIT protocol command bytes, sent as seven 0xFF and this code. */
#define RS_MIT_ENABLE 0xFCu
#define RS_MIT_DISABLE 0xFDu
#define RS_MIT_QUERY 0xFBu

#endif /* DRV_ROBSTRIDE_H */

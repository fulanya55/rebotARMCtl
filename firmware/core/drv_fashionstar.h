/*
 * Fashion Star UART servo driver for the Star Arm 102 (half-duplex, 1 Mbit/s).
 *
 * Frames checked against the STM32 SDK in servodevelop/servo-uart-rs485-sdk and
 * the commands against Fashion Star's control script for the 102 arms.
 *
 * Frame:    header(2) code(1) size(1) params(size) checksum(1)
 * Request:  header 0x12 0x4C.  Response: header 0x05 0x1C.
 * Checksum: sum of all preceding bytes modulo 256.
 * Angles:   little endian, 0.1 degree units.
 */
#ifndef DRV_FASHIONSTAR_H
#define DRV_FASHIONSTAR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FS_CODE_PING 1u
#define FS_CODE_SET_ANGLE_MTURN_INTERVAL 14u
#define FS_CODE_RESET_TURNS 17u
#define FS_CODE_DAMPING 9u
#define FS_CODE_MONITOR 22u
#define FS_CODE_STOP_ON_CONTROL 24u
#define FS_CODE_SYNC 25u

/* Behaviour selected by the stop-on-control command. */
#define FS_STOP_RELEASE 0x10u
#define FS_STOP_HOLD 0x11u
#define FS_STOP_DAMP 0x12u

/* The servo reports this angle when it has no valid reading. */
#define FS_MONITOR_INVALID_ANGLE (-235929599)

#define FS_BROADCAST_ID 0xFFu

#define FS_MAX_FRAME 128u

typedef struct {
    uint8_t servo_id;
    uint16_t voltage_mv;
    uint16_t current_ma;
    uint16_t power_mw;
    uint16_t temp_raw;
    uint8_t status;
    float angle_deg;
    int16_t turns;
    bool angle_valid;
} fs_monitor_t;

size_t fs_encode_ping(uint8_t *out, uint8_t servo_id);
size_t fs_encode_sync_monitor(uint8_t *out, const uint8_t *ids, uint8_t count);

/* Synchronous multi-turn position command, so one frame moves the whole arm at once. */
size_t fs_encode_sync_angle(uint8_t *out, const uint8_t *ids, const float *degrees, uint8_t count,
                            uint32_t interval_ms, uint16_t accel_ms, uint16_t decel_ms, uint16_t power_mw);

size_t fs_encode_stop(uint8_t *out, uint8_t servo_id, uint8_t method, uint16_t power_mw);

/* Clear the turn count, so the servo reads its angle within one revolution again. */
size_t fs_encode_reset_turns(uint8_t *out, uint8_t servo_id);

/* Convert the servo's NTC reading to degrees Celsius. */
float fs_temperature_c(uint16_t raw);

/* Incremental response parser. */
typedef struct {
    uint8_t buf[FS_MAX_FRAME];
    uint8_t len;
    uint8_t need;
    uint32_t checksum_errors;
} fs_parser_t;

void fs_parser_reset(fs_parser_t *p);

/* Feed one byte. Returns a pointer to a complete frame when one is ready. */
const uint8_t *fs_parser_push(fs_parser_t *p, uint8_t byte);

/* Decode a monitor response. Returns false for any other frame. */
bool fs_decode_monitor(const uint8_t *frame, fs_monitor_t *out);

/* Returns the id of a ping response, or -1 for any other frame. */
int fs_decode_ping(const uint8_t *frame);

/* Used by the simulator to build responses and to read requests. */
uint8_t fs_checksum(const uint8_t *data, size_t len);
size_t fs_build_response(uint8_t *out, uint8_t code, const uint8_t *params, uint8_t size);
size_t fs_build_monitor_response(uint8_t *out, uint8_t servo_id, float angle_deg);

#endif /* DRV_FASHIONSTAR_H */

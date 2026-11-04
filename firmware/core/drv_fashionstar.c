#include "drv_fashionstar.h"

#include <math.h>
#include <string.h>

#define REQ_H0 0x12u
#define REQ_H1 0x4Cu
#define RSP_H0 0x05u
#define RSP_H1 0x1Cu

#define MONITOR_PARAM_SIZE 16u
#define SYNC_ANGLE_RECORD_SIZE 11u

uint8_t fs_checksum(const uint8_t *data, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)sum;
}

static size_t finish(uint8_t *out, uint8_t code, uint8_t size)
{
    out[0] = REQ_H0;
    out[1] = REQ_H1;
    out[2] = code;
    out[3] = size;
    out[4u + size] = fs_checksum(out, 4u + size);
    return 5u + size;
}

size_t fs_encode_ping(uint8_t *out, uint8_t servo_id)
{
    out[4] = servo_id;
    return finish(out, FS_CODE_PING, 1);
}

size_t fs_encode_sync_monitor(uint8_t *out, const uint8_t *ids, uint8_t count)
{
    uint8_t *p = out + 4;
    *p++ = FS_CODE_MONITOR;
    *p++ = 1; /* record size: one id */
    *p++ = count;
    memcpy(p, ids, count);
    return finish(out, FS_CODE_SYNC, (uint8_t)(3u + count));
}

size_t fs_encode_sync_angle(uint8_t *out, const uint8_t *ids, const float *degrees, uint8_t count,
                            uint32_t interval_ms, uint16_t power_mw)
{
    uint8_t *p = out + 4;
    *p++ = FS_CODE_SET_ANGLE_MTURN;
    *p++ = SYNC_ANGLE_RECORD_SIZE;
    *p++ = count;
    for (uint8_t i = 0; i < count; i++) {
        int32_t tenths = (int32_t)lroundf(degrees[i] * 10.0f);
        *p++ = ids[i];
        *p++ = (uint8_t)tenths;
        *p++ = (uint8_t)(tenths >> 8);
        *p++ = (uint8_t)(tenths >> 16);
        *p++ = (uint8_t)(tenths >> 24);
        *p++ = (uint8_t)interval_ms;
        *p++ = (uint8_t)(interval_ms >> 8);
        *p++ = (uint8_t)(interval_ms >> 16);
        *p++ = (uint8_t)(interval_ms >> 24);
        *p++ = (uint8_t)power_mw;
        *p++ = (uint8_t)(power_mw >> 8);
    }
    return finish(out, FS_CODE_SYNC, (uint8_t)(3u + count * SYNC_ANGLE_RECORD_SIZE));
}

size_t fs_encode_stop(uint8_t *out, uint8_t servo_id, uint8_t method, uint16_t power_mw)
{
    out[4] = servo_id;
    out[5] = method;
    out[6] = (uint8_t)power_mw;
    out[7] = (uint8_t)(power_mw >> 8);
    return finish(out, FS_CODE_STOP_ON_CONTROL, 4);
}

float fs_temperature_c(uint16_t raw)
{
    if (raw == 0 || raw >= 4096u) {
        return 0.0f;
    }
    /* NTC with beta 3435 and R25 equal to the divider resistor. */
    float ratio = (float)raw / (float)(4096u - raw);
    return 1.0f / (logf(ratio) / 3435.0f + 1.0f / 298.15f) - 273.15f;
}

void fs_parser_reset(fs_parser_t *p)
{
    p->len = 0;
    p->need = 0;
}

const uint8_t *fs_parser_push(fs_parser_t *p, uint8_t byte)
{
    /* Hunt for the response header. A half-duplex bus echoes nothing here since
     * the transceiver disables the receiver while driving. */
    if (p->len == 0) {
        if (byte == RSP_H0) {
            p->buf[p->len++] = byte;
        }
        return NULL;
    }
    if (p->len == 1) {
        if (byte == RSP_H1) {
            p->buf[p->len++] = byte;
        } else {
            p->len = (byte == RSP_H0) ? 1u : 0u;
        }
        return NULL;
    }

    p->buf[p->len++] = byte;
    if (p->len == 4) {
        uint32_t total = 5u + (uint32_t)p->buf[3];
        if (total > FS_MAX_FRAME) {
            fs_parser_reset(p);
            return NULL;
        }
        p->need = (uint8_t)total;
        return NULL;
    }
    if (p->need && p->len == p->need) {
        uint8_t total = p->need;
        fs_parser_reset(p);
        if (fs_checksum(p->buf, total - 1u) != p->buf[total - 1u]) {
            p->checksum_errors++;
            return NULL;
        }
        return p->buf;
    }
    return NULL;
}

static uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

bool fs_decode_monitor(const uint8_t *frame, fs_monitor_t *out)
{
    if (frame[2] != FS_CODE_MONITOR || frame[3] != MONITOR_PARAM_SIZE) {
        return false;
    }
    const uint8_t *p = frame + 4;
    int32_t angle = (int32_t)((uint32_t)p[10] | ((uint32_t)p[11] << 8) | ((uint32_t)p[12] << 16) |
                              ((uint32_t)p[13] << 24));
    int16_t turns = (int16_t)u16le(p + 14);
    out->servo_id = p[0];
    out->voltage_mv = u16le(p + 1);
    out->current_ma = u16le(p + 3);
    out->power_mw = u16le(p + 5);
    out->temp_raw = u16le(p + 7);
    out->status = p[9];
    out->angle_deg = (float)angle / 10.0f;
    out->turns = turns;
    out->angle_valid = !(angle == FS_MONITOR_INVALID_ANGLE && turns == 0);
    return true;
}

int fs_decode_ping(const uint8_t *frame)
{
    if (frame[2] != FS_CODE_PING || frame[3] != 1u) {
        return -1;
    }
    return frame[4];
}

size_t fs_build_response(uint8_t *out, uint8_t code, const uint8_t *params, uint8_t size)
{
    out[0] = RSP_H0;
    out[1] = RSP_H1;
    out[2] = code;
    out[3] = size;
    if (size) {
        memcpy(out + 4, params, size);
    }
    out[4u + size] = fs_checksum(out, 4u + size);
    return 5u + size;
}

size_t fs_build_monitor_response(uint8_t *out, uint8_t servo_id, float angle_deg)
{
    uint8_t params[MONITOR_PARAM_SIZE];
    int32_t tenths = (int32_t)lroundf(angle_deg * 10.0f);
    memset(params, 0, sizeof(params));
    params[0] = servo_id;
    params[1] = 0xE0; /* 12000 mV */
    params[2] = 0x2E;
    params[7] = 0x00; /* NTC mid scale, about 25 degrees C */
    params[8] = 0x08;
    params[10] = (uint8_t)tenths;
    params[11] = (uint8_t)(tenths >> 8);
    params[12] = (uint8_t)(tenths >> 16);
    params[13] = (uint8_t)(tenths >> 24);
    return fs_build_response(out, FS_CODE_MONITOR, params, MONITOR_PARAM_SIZE);
}

#include "rac_wire.h"

#include <string.h>

uint16_t rac_crc16(const uint8_t *data, size_t len)
{
    /* CRC-16/CCITT-FALSE, bitwise on purpose: frames are short and the table
     * would cost 512 bytes of flash for no measurable gain at this rate. */
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t rac_cobs_encode(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t read = 0;
    size_t write = 1;
    size_t code_at = 0;
    uint8_t code = 1;

    while (read < len) {
        if (src[read] == 0) {
            dst[code_at] = code;
            code = 1;
            code_at = write++;
            read++;
        } else {
            dst[write++] = src[read++];
            code++;
            if (code == 0xFFu) {
                dst[code_at] = code;
                code = 1;
                code_at = write++;
            }
        }
    }
    dst[code_at] = code;
    return write;
}

size_t rac_cobs_decode(const uint8_t *src, size_t len, uint8_t *dst, size_t dst_cap)
{
    size_t read = 0;
    size_t write = 0;

    while (read < len) {
        uint8_t code = src[read++];
        if (code == 0) {
            return 0;
        }
        for (uint8_t i = 1; i < code; i++) {
            if (read >= len || write >= dst_cap || src[read] == 0) {
                return 0;
            }
            dst[write++] = src[read++];
        }
        if (code != 0xFFu && read < len) {
            if (write >= dst_cap) {
                return 0;
            }
            dst[write++] = 0;
        }
    }
    return write;
}

void rac_deframer_init(rac_deframer_t *d)
{
    memset(d, 0, sizeof(*d));
}

size_t rac_deframer_push(rac_deframer_t *d, uint8_t byte, uint8_t *out, size_t out_cap)
{
    if (byte != 0) {
        if (d->len < sizeof(d->buf)) {
            d->buf[d->len++] = byte;
        } else {
            d->overflow = 1;
        }
        return 0;
    }

    size_t encoded = d->len;
    uint8_t overflow = d->overflow;
    d->len = 0;
    d->overflow = 0;

    if (encoded == 0) {
        return 0; /* consecutive delimiters are harmless, and are how a link resyncs */
    }
    if (overflow) {
        d->errors++;
        return 0;
    }
    size_t decoded = rac_cobs_decode(d->buf, encoded, out, out_cap);
    if (decoded < 2) {
        d->errors++;
        return 0;
    }
    uint16_t wire_crc = (uint16_t)(out[decoded - 2] | ((uint16_t)out[decoded - 1] << 8));
    if (rac_crc16(out, decoded - 2) != wire_crc) {
        d->errors++;
        return 0;
    }
    return decoded - 2;
}

size_t rac_frame_build(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap)
{
    static uint8_t scratch[RAC_WIRE_MAX_PAYLOAD + 2u];
    if (len > RAC_WIRE_MAX_PAYLOAD || out_cap < RAC_WIRE_MAX_FRAME) {
        return 0;
    }
    memcpy(scratch, payload, len);
    uint16_t crc = rac_crc16(payload, len);
    scratch[len] = (uint8_t)(crc & 0xFFu);
    scratch[len + 1u] = (uint8_t)(crc >> 8);
    size_t encoded = rac_cobs_encode(scratch, len + 2u, out);
    out[encoded] = 0;
    return encoded + 1u;
}

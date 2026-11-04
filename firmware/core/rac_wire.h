/* COBS framing and CRC for RAC-Link. */
#ifndef RAC_WIRE_H
#define RAC_WIRE_H

#include <stddef.h>
#include <stdint.h>

#define RAC_WIRE_MAX_PAYLOAD 640u
/* COBS adds at most one byte per 254, plus the code byte and the delimiter. */
#define RAC_WIRE_MAX_FRAME (RAC_WIRE_MAX_PAYLOAD + 2u + RAC_WIRE_MAX_PAYLOAD / 254u + 2u)

uint16_t rac_crc16(const uint8_t *data, size_t len);

/* Encode src into dst. Returns the encoded length, without the delimiter. */
size_t rac_cobs_encode(const uint8_t *src, size_t len, uint8_t *dst);

/* Decode src into dst. Returns the decoded length, or 0 for malformed input. */
size_t rac_cobs_decode(const uint8_t *src, size_t len, uint8_t *dst, size_t dst_cap);

/* Stream reassembly: bytes in, complete validated payloads out. */
typedef struct {
    uint8_t buf[RAC_WIRE_MAX_FRAME];
    size_t len;
    uint8_t overflow;
    uint32_t errors;
} rac_deframer_t;

void rac_deframer_init(rac_deframer_t *d);

/*
 * Feed one byte. When a complete frame with a valid CRC has arrived, writes the
 * payload without the CRC to out and returns its length. Returns 0 otherwise.
 */
size_t rac_deframer_push(rac_deframer_t *d, uint8_t byte, uint8_t *out, size_t out_cap);

/* Build a frame around header+body bytes. Returns the on-wire length. */
size_t rac_frame_build(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap);

#endif /* RAC_WIRE_H */

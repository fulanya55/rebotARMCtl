/*
 * Controller core: one scheduler, four ports, four logical arms, two pairs.
 *
 * Threading: everything here runs in one context. The port pushes received CAN
 * frames and UART bytes through ring buffers from its interrupts and calls the
 * functions below from the main loop, so no state in this module needs a lock
 * and a tick can never observe half-applied host configuration.
 */
#ifndef RAC_CORE_H
#define RAC_CORE_H

#include <stddef.h>
#include <stdint.h>

#include "rac_link_gen.h"
#include "rac_port.h"

#define RAC_FW_MAJOR 0
#define RAC_FW_MINOR 1
#define RAC_FW_PATCH 0

#define RAC_HW_REVISION 1

/* The scheduler quantum. Motor service periods are multiples of it. */
#define RAC_TICK_US 500u

void rac_core_init(void);

/* Call every RAC_TICK_US. */
void rac_core_tick(uint64_t now_us);

void rac_core_on_host_bytes(const uint8_t *data, size_t len, uint64_t rx_ts_us);
void rac_core_on_can_frame(uint8_t can_index, const rac_can_frame_t *frame, uint64_t rx_ts_us);
void rac_core_on_uart_bytes(uint8_t uart_index, const uint8_t *data, size_t len, uint64_t rx_ts_us);

/* Exposed so the simulator and tests can inspect behaviour directly. */
typedef struct {
    uint8_t mode;
    uint8_t source;
    uint8_t torque_on;
    uint8_t valid_mask;
    uint32_t generation;
    uint32_t queue_depth;
    float position[RAC_JOINTS];
    float target[RAC_JOINTS];
} rac_arm_debug_t;

void rac_core_debug_arm(uint8_t arm, rac_arm_debug_t *out);

#endif /* RAC_CORE_H */

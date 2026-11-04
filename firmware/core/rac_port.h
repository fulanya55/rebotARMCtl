/*
 * Board abstraction for the controller core.
 *
 * Everything in firmware/core is plain C with no hardware access, so the same
 * code runs on the STM32G474 (port/stm32g4) and on a host computer against a
 * simulated plant (port/sil). A port provides the functions below and calls
 * the rac_core_on_* entry points from rac_core.h.
 */
#ifndef RAC_PORT_H
#define RAC_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t id;      /* 11 or 29 bit identifier */
    uint8_t extended; /* non-zero for a 29 bit identifier */
    uint8_t dlc;
    uint8_t data[8];
} rac_can_frame_t;

/* Free-running monotonic microsecond clock. Must never wrap in practice. */
uint64_t rac_port_now_us(void);

/* Queue one CAN frame for transmission. Returns false when the queue is full. */
bool rac_port_can_send(uint8_t can_index, const rac_can_frame_t *frame);

/* Queue bytes on a half-duplex servo bus. The port owns direction control. */
bool rac_port_uart_send(uint8_t uart_index, const uint8_t *data, size_t len);
void rac_port_uart_set_baud(uint8_t uart_index, uint32_t baud);

/* Queue bytes to the host. Returns false when the upstream buffer is full. */
bool rac_port_host_send(const uint8_t *data, size_t len);

/* True while the emergency stop chain is open (button pressed or wire broken). */
bool rac_port_estop_active(void);

/* Status indicators. Each call is cheap; ports may ignore any of them. */
void rac_port_led_status(uint8_t port_index, bool online, bool active);
void rac_port_led_run(bool on);
void rac_port_led_fault(bool on);

/* Permission signal for the safety relay. The hardware chain can still open it. */
void rac_port_safety_enable(bool enable);

/* Pulse the camera trigger output. */
void rac_port_sync_pulse(bool level);

/* Refresh the independent watchdog. */
void rac_port_watchdog_kick(void);

/* 96 bit unique device identifier. */
void rac_port_unique_id(uint8_t out[12]);

/* Capabilities only this port knows about, for example RAC_CAP_HW_ESTOP. */
uint32_t rac_port_capabilities(void);

/* Port-specific host messages. Returns true when the message was consumed. */
bool rac_port_handle_extension(uint8_t msg_type, uint8_t arm_id, const uint8_t *body, size_t len);

#endif /* RAC_PORT_H */

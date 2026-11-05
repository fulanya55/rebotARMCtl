/*
 * Simulated arms for software-in-the-loop runs.
 *
 * The plant answers on the same wire protocols as the real motors, at the byte
 * and frame level with bus timing, so the driver code and the scheduler in
 * firmware/core run unchanged and are what is being exercised.
 */
#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rac_link_gen.h"
#include "rac_port.h"

void sim_init(void);

/* Advance simulated physics and deliver every bus event due by this time. */
void sim_advance(uint64_t now_us);

/* Attach an arm model to a port, or RAC_MODEL_NONE to leave it unconnected. */
void sim_set_model(uint8_t port, uint8_t model);

/* Upstream sink. Set by the program that hosts the plant. */
typedef void (*sim_host_sink_t)(const uint8_t *data, size_t len);
void sim_set_host_sink(sim_host_sink_t sink);

/* Inspection and fault injection, also reachable through SIL_INJECT. */
void sim_inject(const rac_sil_inject_t *m);
float sim_motor_position(uint8_t port, uint8_t joint);
void sim_set_turns(uint8_t port, uint8_t joint, int16_t turns);
void sim_set_position(uint8_t port, uint8_t joint, float position);
void sim_motor_gains(uint8_t port, uint8_t joint, float *kp, float *kd);
uint32_t sim_can_frames_sent(uint8_t bus);

#endif /* SIM_PLANT_H */

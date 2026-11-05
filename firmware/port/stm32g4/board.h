/*
 * Pin map of the sync controller, revision A. It matches hardware/sync-controller
 * and was checked against ST's STM32_open_pin_data. A change here is a board change.
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32g4xx.h"

#define HSE_HZ 8000000u
#define SYSCLK_HZ 170000000u
#define PCLK1_HZ SYSCLK_HZ

/* CAN1 port: FDCAN2 on PB12 (RX), PB13 (TX), AF9.
 * FDCAN1 is unusable here: its pins are USB and BOOT0. */
#define CAN1_PERIPH FDCAN2
#define CAN1_RAM_OFFSET 0x350u
#define CAN1_IRQ FDCAN2_IT0_IRQn

/* CAN2 port: FDCAN3 on PA8 (RX), PA15 (TX), AF11. */
#define CAN2_PERIPH FDCAN3
#define CAN2_RAM_OFFSET 0x6A0u
#define CAN2_IRQ FDCAN3_IT0_IRQn

/* UART1 port: USART2 on PA2 (TX), PA3 (RX), AF7.
 * UART2 port: USART3 on PB10 (TX), PB11 (RX), AF7. */

/* Safety chain. */
#define ESTOP_CH1_PORT GPIOC /* PC0: high while contact 1 is closed */
#define ESTOP_CH1_PIN 0u
#define ESTOP_CH2_PORT GPIOC /* PC1: low while contact 2 is closed */
#define ESTOP_CH2_PIN 1u
#define SAFETY_EN_PORT GPIOC /* PC2: drives the relay MOSFET, pulled down off-chip */
#define SAFETY_EN_PIN 2u
#define STOP_SENSE_PORT GPIOC /* PC3: gate node; low while enabled means STOP is held */
#define STOP_SENSE_PIN 3u

/* Camera sync, both pins 5 V tolerant. */
#define SYNC_OUT_PORT GPIOC
#define SYNC_OUT_PIN 10u
#define SYNC_IN_PORT GPIOC
#define SYNC_IN_PIN 11u

/* LEDs, active high. */
#define LED_PORT0_GPIO GPIOC
#define LED_PORT0_PIN 6u
#define LED_PORT1_GPIO GPIOC
#define LED_PORT1_PIN 7u
#define LED_PORT2_GPIO GPIOC
#define LED_PORT2_PIN 8u
#define LED_PORT3_GPIO GPIOC
#define LED_PORT3_PIN 9u
#define LED_RUN_GPIO GPIOB
#define LED_RUN_PIN 14u
#define LED_FAULT_GPIO GPIOB
#define LED_FAULT_PIN 15u

#endif /* BOARD_H */

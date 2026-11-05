#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU OPT_MCU_STM32G4
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_DEBUG 0
#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENDPOINT0_SIZE 64
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

/* Upstream carries up to four arm streams: a few frames of slack, not a backlog. */
#define CFG_TUD_CDC_RX_BUFSIZE 2048
#define CFG_TUD_CDC_TX_BUFSIZE 8192
#define CFG_TUD_CDC_EP_BUFSIZE 64

#endif /* TUSB_CONFIG_H */

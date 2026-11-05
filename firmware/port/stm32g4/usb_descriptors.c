/* USB identity: one CDC-ACM serial port. rac_link.transport finds the board by VID and PID. */
#include <stddef.h>
#include <string.h>

#include "rac_port.h"
#include "tusb.h"

/* pid.codes test range: a development value until the board has its own allocation. */
#define USB_VID 0x1209
#define USB_PID 0x0001

static const tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&device_descriptor;
}

enum { ITF_CDC_CONTROL, ITF_CDC_DATA, ITF_COUNT };

#define CONFIG_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#define EP_CDC_NOTIFY 0x81
#define EP_CDC_OUT 0x02
#define EP_CDC_IN 0x82

static const uint8_t configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_LEN, 0x00, 400),
    TUD_CDC_DESCRIPTOR(ITF_CDC_CONTROL, 4, EP_CDC_NOTIFY, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return configuration_descriptor;
}

static const char *const strings[] = {
    NULL,
    "rebotARMCtl",
    "RAC Sync Controller",
    NULL, /* serial: from the chip's unique id */
    "RAC-Link",
};

static uint16_t string_buffer[32];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint8_t count = 0;

    if (index == 0) {
        string_buffer[1] = 0x0409;
        count = 1;
    } else if (index == 3) {
        static const char hex[] = "0123456789ABCDEF";
        uint8_t uid[12];
        rac_port_unique_id(uid);
        for (uint8_t i = 0; i < 12; i++) {
            string_buffer[1 + count++] = (uint16_t)hex[uid[i] >> 4];
            string_buffer[1 + count++] = (uint16_t)hex[uid[i] & 0xFu];
        }
    } else {
        if (index >= sizeof(strings) / sizeof(strings[0]) || !strings[index]) {
            return NULL;
        }
        const char *text = strings[index];
        while (text[count] && count < 31) {
            string_buffer[1 + count] = (uint16_t)text[count];
            count++;
        }
    }
    string_buffer[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2u * count + 2u));
    return string_buffer;
}

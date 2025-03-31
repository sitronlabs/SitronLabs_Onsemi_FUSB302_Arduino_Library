#ifndef USB_PD_H
#define USB_PD_H

/* C/C++ libraries */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

/**
 * List of possible protocol revisions.
 * @see Section 6.2.1.1.5 Specification Revision.
 */
enum usb_pd_protocol_revision {
    USB_PD_PROTOCOL_REVISION_1_0 = 0b00,
    USB_PD_PROTOCOL_REVISION_2_0 = 0b01,
    USB_PD_PROTOCOL_REVISION_3_0 = 0b10,
};

/**
 * List of possible messages.
 * @see Table 6-5 Control Message Types.
 * @see Table 6-6 Data Message Types.
 */
enum usb_pd_message_type {
    USB_PD_MESSAGE_TYPE_CONTROL_GOODCRC = 0b00001,
    USB_PD_MESSAGE_TYPE_CONTROL_GOTOMIN = 0b00010,
    USB_PD_MESSAGE_TYPE_CONTROL_ACCEPT = 0b00011,
    USB_PD_MESSAGE_TYPE_CONTROL_REJECT = 0b00100,
    USB_PD_MESSAGE_TYPE_CONTROL_PS_RDY = 0b00110,
    USB_PD_MESSAGE_TYPE_CONTROL_GET_SOURCE_CAP = 0b00111,
    USB_PD_MESSAGE_TYPE_CONTROL_GET_SINK_CAP = 0b01000,
    USB_PD_MESSAGE_TYPE_CONTROL_SOFT_RESET = 0b01101,
    USB_PD_MESSAGE_TYPE_CONTROL_GET_STATUS = 0b10010,
    USB_PD_MESSAGE_TYPE_DATA_SOURCE_CAPABILITIES = 0b00001,
    USB_PD_MESSAGE_TYPE_DATA_REQUEST = 0b00010,
    USB_PD_MESSAGE_TYPE_DATA_SINK_CAPABILITIES = 0b00100,
    USB_PD_MESSAGE_TYPE_DATA_BATTERY_STATUS = 0b00101,
    USB_PD_MESSAGE_TYPE_DATA_ALERT = 0b001100,
    USB_PD_MESSAGE_TYPE_DATA_VENDOR_DEFINNED = 0b01111,
};

/**
 * List of possible power data object types.
 * @see Table 6-7 Power Data Object
 */
enum usb_pd_pdo_type {
    USB_PD_PDO_TYPE_FIXED = 0b00,
    USB_PD_PDO_TYPE_BATTERY = 0b01,
    USB_PD_PDO_TYPE_VARIABLE = 0b10,
    USB_PD_PDO_TYPE_AUGMENTED = 0b11,
};

/**
 *
 */
struct usb_pd_pdo {
    enum usb_pd_pdo_type type;
    union {
        struct {
            float voltage;
            float current;
        } fixed;
    } data;
};

/**
 *
 */
struct usb_pd_message {
    uint16_t address;
    uint16_t header;
    uint32_t objects[7];
    uint8_t object_count;
};

#endif

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
 * List of possible destinations.
 * @see Table 6-4 Destination.
 */
enum usb_pd_sop_type {
    USB_PD_SOP_TYPE_DEFAULT,       //!< SOP. Used for regular PD messages like capabilities exchange, power requests, etc.
    USB_PD_SOP_TYPE_PRIME,         //!< SOP'. Used for reading cable identity (e.g. electronically marked cable info, VCONN management)
    USB_PD_SOP_TYPE_PRIME_DOUBLE,  //!< SOP''. Used for same kind of things as SOP', but for the opposite end of the cable.
    USB_PD_SOP_TYPE_UNKNOWN,       //!< Unknown SOP type
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
 * USB Power Delivery message structure.
 *
 * This structure represents a complete USB PD message including:
 * - SOP type (destination: device, cable plug prime, or cable plug double prime)
 * - Message header containing message type, data role, power role, etc.
 * - Optional data objects (up to 7 objects)
 *
 * @see Section 6.2 of the USB Power Delivery Specification
 */
struct usb_pd_message {
    enum usb_pd_sop_type sop_type = USB_PD_SOP_TYPE_DEFAULT;  //!< Message destination type
    uint16_t header = 0;                                      //!< 16-bit message header
    uint32_t objects[7] = {0};                                //!< Data objects (PDOs or VDOs)
    uint8_t object_count = 0;                                 //!< Number of data objects (0-7)
};

#endif

/**
 * USB Type-C Definitions
 *
 * This file contains enumerations for USB Type-C operation including
 * cable orientations and power roles.
 */

#ifndef USB_TYPEC_H
#define USB_TYPEC_H

/* C/C++ libraries */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Cable Orientation
 *
 * USB Type-C is reversible. These values indicate cable orientation.
 */
enum usb_typec_cc_orientation {
    USB_TYPEC_CC_ORIENTATION_NONE,     //!< No cable connected
    USB_TYPEC_CC_ORIENTATION_NORMAL,   //!< Cable is plugged in normally (CC1 active)
    USB_TYPEC_CC_ORIENTATION_REVERSE,  //!< Cable is plugged in upside-down (CC2 active)
};

/**
 * CC Pin Status Values
 *
 * The CC (Configuration Channel) pins can have different voltage levels that indicate
 * what is connected and available power.
 */
enum usb_typec_cc_status {
    USB_TYPEC_CC_STATUS_OPEN,    //!< Nothing connected (open circuit)
    USB_TYPEC_CC_STATUS_RA,      //!< Audio accessory detected (Ra = low resistance)
    USB_TYPEC_CC_STATUS_RD,      //!< Device/sink connected (Rd = pull-down resistor)
    USB_TYPEC_CC_STATUS_RP_DEF,  //!< Host/source connected, default USB power (up to 900mA @ 5V)
    USB_TYPEC_CC_STATUS_RP_1_5,  //!< Host/source connected, medium power (up to 1.5A @ 5V)
    USB_TYPEC_CC_STATUS_RP_3_0,  //!< Host/source connected, high power (up to 3.0A @ 5V)
};

/**
 * Data Role
 *
 * In USB Type-C, a device can be either the host (controlling data transfer) or
 * the device (receiving commands).
 */
enum usb_typec_data_role {
    USB_TYPEC_DATA_ROLE_DEVICE,  //!< Acting as a USB device (peripheral)
    USB_TYPEC_DATA_ROLE_HOST,    //!< Acting as a USB host (controller)
};

/**
 * Power Role
 *
 * In USB Type-C with Power Delivery, a device can be either providing power (source)
 * or consuming power (sink). This can change during operation.
 */
enum usb_typec_power_role {
    USB_TYPEC_DATA_POWER_SINK,    //!< Consuming power (charging/powered device)
    USB_TYPEC_DATA_POWER_SOURCE,  //!< Providing power (charger/power supply)
};

#endif

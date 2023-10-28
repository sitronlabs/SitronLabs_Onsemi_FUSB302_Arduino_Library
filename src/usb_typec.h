#ifndef USB_TYPEC_H
#define USB_TYPEC_H

/* C/C++ libraries */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

/** List of possible cable orientations */
enum usb_typec_cc_orientation {
    USB_TYPEC_CC_ORIENTATION_NONE,
    USB_TYPEC_CC_ORIENTATION_NORMAL,   //!< Default orientation, where cc line is on cc1 pin.
    USB_TYPEC_CC_ORIENTATION_REVERSE,  //!< Flipped orientation, where cc line is on cc2 pin.
};

/** List of possible status for the CC pins */
enum usb_typec_cc_status {
    USB_TYPEC_CC_STATUS_OPEN,    //!<
    USB_TYPEC_CC_STATUS_RA,      //!<
    USB_TYPEC_CC_STATUS_RD,      //!<
    USB_TYPEC_CC_STATUS_RP_DEF,  //!<
    USB_TYPEC_CC_STATUS_RP_1_5,  //!<
    USB_TYPEC_CC_STATUS_RP_3_0,  //!<
};

/** List of possible data roles */
enum usb_typec_data_role {
    USB_TYPEC_DATA_ROLE_DEVICE,
    USB_TYPEC_DATA_ROLE_HOST,
};

/** List of possible power roles */
enum usb_typec_power_role {
    USB_TYPEC_DATA_POWER_SINK,
    USB_TYPEC_DATA_POWER_SOURCE,
};

#endif

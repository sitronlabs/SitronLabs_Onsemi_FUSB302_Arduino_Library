
/* Arduino libraries */
#include <Wire.h>
#include <fusb302.h>

/* Peripheral */
static fusb302 m_fusb302;

/**
 * @brief Setup function, executed only once.
 */
void setup() {
    int res;

    /* Setup serial */
    Serial.begin(115200);
    Serial.println("Hello world!");

    /* Setup i2c */
    Wire.begin();

    /* Setup fusb302 ic */
    res = m_fusb302.setup(Wire, 0x22);
    if (res < 0) {
        Serial.println("Failed to register fusb302 ic!");
        while (1);
    }
    if (m_fusb302.detect() != true) {
        Serial.println("Failed to detect fusb302 ic!");
        while (1);
    }
    res = m_fusb302.power_set(true);
    if (res < 0) {
        Serial.println("Failed to power on fusb302 ic!");
        while (1);
    }
}

/**
 * @brief Periodic function, executed in and endless loop.
 */
void loop() {
    int res;

    /* Measure voltages on the cc pins */
    enum usb_typec_cc_status cc1, cc2;
    res = m_fusb302.cc_measure(cc1, cc2);
    if (res < 0) {
        Serial.println("Failed to read cc voltages!");
        return;
    }

    /* Detect orientation */
    usb_typec_cc_orientation orientation;
    if (cc1 > USB_TYPEC_CC_STATUS_OPEN && cc2 == USB_TYPEC_CC_STATUS_OPEN) {
        Serial.println("Type-C orientation is default.");
        orientation = USB_TYPEC_CC_ORIENTATION_NORMAL;
    } else if (cc1 == USB_TYPEC_CC_STATUS_OPEN && cc2 > USB_TYPEC_CC_STATUS_OPEN) {
        Serial.println("Type-C orientation is flipped.");
        orientation = USB_TYPEC_CC_ORIENTATION_REVERSE;
    } else {
        Serial.println("Invalid cc pin logic");
        return;
    }
}

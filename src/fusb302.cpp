/* Self header */
#include "fusb302.h"

/* Config */
#define CONFIG_FUSB302_RX_QUEUE_SIZE 5  //!<
#define CONFIG_FUSB302_I2C_ATTEMPTS 3   //!<
#ifndef CONFIG_FUSB302_LOG_FUNCTION
#define CONFIG_FUSB302_LOG_FUNCTION(...) (void)0  //!< Replace by { Serial.printf(__VA_ARGS__); Serial.println(); } to output on serial port
#endif

/**
 *
 * @param[in] i2c_library
 * @param[in] i2c_address
 * @return
 */
int fusb302::setup(TwoWire &i2c_library, const uint8_t i2c_address) {

    /* Save parameters */
    m_i2c_library = &i2c_library;
    m_i2c_address = i2c_address;

    /* Return success */
    return 0;
}

/**
 * Tries to detect the device.
 * @return true if the device has been detected, or false otherwise.
 */
bool fusb302::detect(void) {
    int res;

    /* Read device id register */
    uint8_t reg_device_id;
    res = register_read(FUSB302_REGISTER_DEVICE_ID, &reg_device_id);
    if (res < 0) {
        return false;
    }

    /* Return whether or not the value matches */
    uint8_t device_id = reg_device_id >> 4U;
    if ((device_id == 0b1001) ||  //
        (device_id == 0b1010) ||  //
        (device_id == 0b1011)) {
        return true;
    } else {
        return false;
    }
}

/**
 * Resets the entire chip.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int fusb302::reset(void) {

    /* Reset ic */
    uint8_t reg_reset = 0x01;
    int res = register_write(FUSB302_REGISTER_RESET, &reg_reset);
    if (res < 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * @brief
 * @param[in] on
 * @return 0 in case of success, or a negative error code otherwise.
 */
int fusb302::power_set(const bool on) {

    /* Turn power on or off */
    uint8_t reg_power = (on == true) ? 0x0F : 0x00;
    int res = register_write(FUSB302_REGISTER_POWER, &reg_power);
    if (res < 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * Reads the contents of the given register.
 * @param[in] address The address of the register.
 * @param[out] content A pointer to a variable that will be updated with the contents of the register.
 * @param[in] count
 * @return 0 in case of success, or a negative error code otherwise.
 */
int fusb302::register_read(const enum fusb302_register address, uint8_t *const content, const size_t count) {

    /* Ensure library has been configured */
    if (m_i2c_library == NULL) {
        return -EINVAL;
    }

    /* Send register address */
    m_i2c_library->beginTransmission(m_i2c_address);
    m_i2c_library->write(address);
    if (m_i2c_library->endTransmission(false) != 0) {
        return -EIO;
    }

    /* Read data */
    m_i2c_library->requestFrom(m_i2c_address, (uint8_t)count, (uint8_t) true);
    if (m_i2c_library->available() != ((int)count)) {
        return -EIO;
    }
    for (size_t i = 0; i < count; i++) {
        content[i] = m_i2c_library->read();
    }

    /* Return success */
    return 0;
}

/**
 * Updates the content of the given register.
 * @param[in] reg_address The address of the register.
 * @param[in] reg_content The new content of the register.
 * @param[in] count
 * @return 0 in case of success, or a negative error code otherwise.
 */
int fusb302::register_write(const enum fusb302_register address, const uint8_t *const content, const size_t count) {

    /* Ensure library has been configured */
    if (m_i2c_library == NULL) {
        return -EINVAL;
    }

    /* Write register to device */
    m_i2c_library->beginTransmission(m_i2c_address);
    m_i2c_library->write((uint8_t)address);
    m_i2c_library->write(content, count);
    if (m_i2c_library->endTransmission(true) != 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Presents Rd resistors on both CC lines.
 * @note This is the power-on default for the FUSB302B but not for the FUSB302T.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int fusb302::cc_pull_down(void) {
    int res;

    /* Clear PU_EN1 and PU_EN2 to disable pullup resistors
     * Set PDWN1 and PDWN2 to enable pulldown resistors */
    uint8_t reg_switches0;
    res = register_read(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }
    reg_switches0 &= ~0b11000000;
    reg_switches0 |= 0b00000011;
    res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Presents Rp resistors on both CC lines.
 * @param[in] status The value of the Rp resistors to select with current is advertised.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int fusb302::cc_pull_up(const enum usb_typec_cc_status status) {
    int res;

    /* Ensure status is one of the expected Rp values */
    if ((status != USB_TYPEC_CC_STATUS_RP_DEF) &&  //
        (status != USB_TYPEC_CC_STATUS_RP_1_5) &&  //
        (status != USB_TYPEC_CC_STATUS_RP_3_0)) {
        return -EINVAL;
    }

    /* Clear PDWN1 and PDWN2 to disable pulldown resistors */
    uint8_t reg_switches0;
    res = register_read(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }
    reg_switches0 &= ~0b00000011;
    res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }

    /* Set HOST_CUR */
    uint8_t reg_control0 = 0;
    res = register_read(FUSB302_REGISTER_CONTROL0, &reg_control0);
    if (res < 0) {
        return -EIO;
    }
    reg_control0 &= ~(0b11 << 2);
    if (status == USB_TYPEC_CC_STATUS_RP_DEF) {
        reg_control0 |= (0b01 << 2);
    } else if (status == USB_TYPEC_CC_STATUS_RP_1_5) {
        reg_control0 |= (0b10 << 2);
    } else if (status == USB_TYPEC_CC_STATUS_RP_3_0) {
        reg_control0 |= (0b11 << 2);
    }
    res = register_write(FUSB302_REGISTER_CONTROL0, &reg_control0);
    if (res < 0) {
        return -EIO;
    }

    /* Set PU_EN1 and PU_EN2 to enable pullup resistors */
    reg_switches0 |= 0b11000000;
    res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * @brief
 * @param
 * @param
 * @return
 * @see https://github.com/graycatlabs/usb-c-arduino/blob/master/usb-c-demo/FUSB302.c#L97
 */
int fusb302::cc_measure(enum usb_typec_cc_status &cc1, enum usb_typec_cc_status &cc2) {
    int res;

    /* Clear MEAS_VBUS bit */
    uint8_t reg_measure;
    res = register_read(FUSB302_REGISTER_MEASURE, &reg_measure);
    if (res < 0) {
        return -EIO;
    }
    if ((reg_measure & (1 << 6)) != 0) {
        reg_measure &= ~(1 << 6);
        res = register_write(FUSB302_REGISTER_MEASURE, &reg_measure);
        if (res < 0) {
            return -EIO;
        }
    }

    /* Read register SWITCHES0 */
    uint8_t reg_switches0_original;
    res = register_read(FUSB302_REGISTER_SWITCHES0, &reg_switches0_original);
    if (res < 0) {
        return -EIO;
    }

    /* If we have Rd pull-down resistors enabled */
    if ((reg_switches0_original & 0b11000011) == 0b00000011) {

        /* Request to measure cc1 */
        uint8_t reg_switches0 = reg_switches0_original;
        reg_switches0 &= ~0b00001100;
        reg_switches0 |= 0b00000100;
        res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to start measurement on cc1 pin!");
            return -EIO;
        }

        /* Wait for at least 250 microseconds */
        delayMicroseconds(250);

        /* Read measured cc1 voltage */
        uint8_t reg_status0 = 0;
        res = register_read(FUSB302_REGISTER_STATUS0, &reg_status0);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to read measurement on cc1 pin!");
            return -EIO;
        }

        /* Store voltage */
        cc1 = USB_TYPEC_CC_STATUS_OPEN;
        if ((0b00000011 & reg_status0) == 0x1) {
            cc1 = USB_TYPEC_CC_STATUS_RP_DEF;
        } else if ((0b00000011 & reg_status0) == 0x2) {
            cc1 = USB_TYPEC_CC_STATUS_RP_1_5;
        } else if ((0b00000011 & reg_status0) == 0x3) {
            cc1 = USB_TYPEC_CC_STATUS_RP_3_0;
        }

        /* Request to measure cc2 */
        reg_switches0 = reg_switches0_original;
        reg_switches0 &= ~0b00001100;
        reg_switches0 |= 0b00001000;
        res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to start measurement cc2 pin!");
            return -EIO;
        }

        /* Wait for at least 250 microseconds */
        delayMicroseconds(250);

        /* Read cc2 voltage */
        reg_status0 = 0;
        res = register_read(FUSB302_REGISTER_STATUS0, &reg_status0);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to read measurement on cc2 pin!");
            return -EIO;
        }

        /* Store voltage */
        cc2 = USB_TYPEC_CC_STATUS_OPEN;
        if ((0b00000011 & reg_status0) == 0x1) {
            cc2 = USB_TYPEC_CC_STATUS_RP_DEF;
        } else if ((0b00000011 & reg_status0) == 0x2) {
            cc2 = USB_TYPEC_CC_STATUS_RP_1_5;
        } else if ((0b00000011 & reg_status0) == 0x3) {
            cc2 = USB_TYPEC_CC_STATUS_RP_3_0;
        }

        /* Disable measurements */
        reg_switches0 = reg_switches0_original;
        reg_switches0 &= ~0b00001100;
        res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to disable measurements on cc pins!");
            return -EIO;
        }
    }

    /* If we have Rp pull-up resistors enabled
     * @todo Maybe when we source vconn Rp on that CC pin has to be disabled, which means the test would fail? I'm not sure. */
    else if ((reg_switches0_original & 0b11000011) == 0b11000000) {

        /* Determine the MDAC level for COMP based on HOST_CUR */
        uint8_t reg_control0 = 0;
        res = register_read(FUSB302_REGISTER_CONTROL0, &reg_control0);
        if (res < 0) {
            return -EIO;
        }
        uint8_t host_cur = (reg_control0 >> 2U) & 0b11;
        uint8_t reg_measure;
        if ((host_cur == 0b01) || (host_cur == 0b10)) {
            reg_measure = 0b00100110;
        } else if (host_cur == 0b11) {
            reg_measure = 0b00111110;
        } else {
            return -EINVAL;
        }
        res = register_write(FUSB302_REGISTER_MEASURE, &reg_measure);
        if (res < 0) {
            return -EIO;
        }

        /* Request to measure cc1 */
        uint8_t reg_switches0 = reg_switches0_original;
        reg_switches0 &= ~0b00001100;
        reg_switches0 |= 0b00000100;
        res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to start measurement on cc1 pin!");
            return -EIO;
        }

        /* Wait for at least 250 microseconds */
        delayMicroseconds(250);

        /* Look at COMP bit */
        uint8_t reg_status0;
        res = register_read(FUSB302_REGISTER_STATUS0, &reg_status0);
        if (res < 0) {
            return -EIO;
        }
        if ((reg_status0 & 0b00100000) == 0) {
            cc1 = USB_TYPEC_CC_STATUS_RD;
        } else {
            cc1 = USB_TYPEC_CC_STATUS_OPEN;
        }

        // TODO Ra

        /* Request to measure cc2 */
        reg_switches0 = reg_switches0_original;
        reg_switches0 &= ~0b00001100;
        reg_switches0 |= 0b00001000;
        res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to start measurement cc2 pin!");
            return -EIO;
        }

        /* Wait for at least 250 microseconds */
        delayMicroseconds(250);

        /* Look at COMP bit */
        res = register_read(FUSB302_REGISTER_STATUS0, &reg_status0);
        if (res < 0) {
            return -EIO;
        }
        if ((reg_status0 & 0b00100000) == 0) {
            cc2 = USB_TYPEC_CC_STATUS_RD;
        } else {
            cc2 = USB_TYPEC_CC_STATUS_OPEN;
        }

        // TODO Ra

        /* Disable measurements */
        reg_switches0 = reg_switches0_original;
        reg_switches0 &= ~0b00001100;
        res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to disable measurements on cc pins!");
            return -EIO;
        }
    }

    /* Otherwise */
    else {
        return -EINVAL;
    }

    /* Return success */
    return 0;
}

/**
 *
 */
int fusb302::cc_orientation_set(const enum usb_typec_cc_orientation orientation) {
    int res;

    /* Ensure arguments are valid */
    if ((orientation != USB_TYPEC_CC_ORIENTATION_NORMAL) &&  //
        (orientation != USB_TYPEC_CC_ORIENTATION_REVERSE)) {
        return -EINVAL;
    }

    /* Set polarity for RX
     * @note Interfers with DFP/SRC code */
    uint8_t reg_switches0;
    res = register_read(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }
    if (orientation == USB_TYPEC_CC_ORIENTATION_NORMAL) {
        reg_switches0 &= 0b11110011;
        reg_switches0 |= 0b00000100;
    } else {
        reg_switches0 &= 0b11110011;
        reg_switches0 |= 0b00001000;
    }
    res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }

    /* Set polarity for TX */
    uint8_t reg_switches1;
    res = register_read(FUSB302_REGISTER_SWITCHES1, &reg_switches1);
    if (res < 0) {
        return -EIO;
    }
    if (orientation == USB_TYPEC_CC_ORIENTATION_NORMAL) {
        reg_switches1 &= 0b11111100;
        reg_switches1 |= 0b00000001;
    } else {
        reg_switches1 &= 0b11111100;
        reg_switches1 |= 0b00000010;
    }
    res = register_write(FUSB302_REGISTER_SWITCHES1, &reg_switches1);
    if (res < 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 *
 * @param[out] voltage_v
 * @return 0 in case of success, or a negative error code otherwise.
 */
int fusb302::vbus_measure(float &voltage_v) {
    int res;

    /* Disable MEAS_CC */
    uint8_t reg_switches0;
    res = register_read(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }
    reg_switches0 &= 0xF3;
    res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }

    /* Iterate over all 6-bit values of the comparator
     * @note This is quite inefficient, and should probably rewritten to make use of a binary search algorithm */
    voltage_v = 0;
    for (uint8_t i = 0; i < 64; i++) {

        /* Set MEAS_VBUS bit and MDAC value */
        uint8_t reg_measure = (1 << 6) | i;
        res = register_write(FUSB302_REGISTER_MEASURE, &reg_measure);
        if (res < 0) {
            return -EIO;
        }

        /* Look at COMP bit */
        uint8_t reg_status0;
        res = register_read(FUSB302_REGISTER_STATUS0, &reg_status0);
        if (res < 0) {
            return -EIO;
        }

        /* Return when transition found */
        if ((reg_status0 & (1 << 5)) == 0x00) {
            voltage_v = (i + 0.5) * 0.420;
            return 0;
        }
    }

    /* Return when out of range */
    voltage_v = 26.88;
    return 0;
}

/**
 * @brief
 * @param
 * @return
 */
int fusb302::pd_reset(void) {

    /* Reset the pd logic */
    uint8_t reg_reset = 0b00000010;
    int res = register_write(FUSB302_REGISTER_RESET, &reg_reset);
    if (res < 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 *
 * @param[in] enabled
 * @return 0 in case of success, or a negative error code otherwise.
 */
int fusb302::pd_autogoodcrc_set(const bool enabled) {
    int res;

    /* Read register */
    uint8_t reg_switches1;
    res = register_read(FUSB302_REGISTER_SWITCHES1, &reg_switches1);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to read register!");
        return -EIO;
    }

    /* Set flag */
    if (enabled) {
        reg_switches1 |= (1U << 2U);
    } else {
        reg_switches1 &= ~(1U << 2U);
    }

    /* Update register */
    res = register_write(FUSB302_REGISTER_SWITCHES1, &reg_switches1);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to write register!");
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * @brief
 * @param[in] retries The amount of times the fusb302 ic should retry to send a packet if a goodcrc was not received.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int fusb302::pd_autoretry_set(const int retries) {
    int res;

    /* Ensure argument is valid */
    if ((retries < 0) || (retries > 3)) {
        CONFIG_FUSB302_LOG_FUNCTION("Invalid number of retries!");
        return -EINVAL;
    }

    /*  */
    uint8_t reg_control3 = 0;
    res = register_read(FUSB302_REGISTER_CONTROL3, &reg_control3);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to flush rx fifo!");
        return -EIO;
    }
    reg_control3 &= 0xF8;
    if (retries > 0) {
        reg_control3 |= (retries << 1U);
        reg_control3 |= 1U;
    }
    res = register_write(FUSB302_REGISTER_CONTROL3, &reg_control3);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to flush rx fifo!");
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 *
 * @return
 */
int fusb302::pd_rx_flush(void) {
    int res;

    /* Request to clear the rx fifo */
    uint8_t reg_control1 = 0;
    res = register_read(FUSB302_REGISTER_CONTROL1, &reg_control1);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to flush rx fifo!");
        return -EIO;
    }
    reg_control1 |= (1U << 2U);
    res = register_write(FUSB302_REGISTER_CONTROL1, &reg_control1);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to flush rx fifo!");
        return -EIO;
    }

    /* Wait for the clear operation to be completed
     * @todo Implement a timeout mechanism */
    while (1) {
        res = register_read(FUSB302_REGISTER_CONTROL1, &reg_control1);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to flush rx fifo!");
            return -EIO;
        }
        if ((reg_control1 & (1U << 2U)) == 0x00) {
            break;
        }
    }

    /* Return success */
    return 0;
}

/**
 *
 * @return
 */
int fusb302::pd_tx_flush(void) {
    int res;

    /* Request to clear the tx fifo */
    uint8_t reg_control0 = 0;
    res = register_read(FUSB302_REGISTER_CONTROL0, &reg_control0);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to flush tx fifo!");
        return -EIO;
    }
    reg_control0 |= (1U << 6U);
    res = register_write(FUSB302_REGISTER_CONTROL0, &reg_control0);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to flush tx fifo!");
        return -EIO;
    }

    /* Wait for the clear operation to be completed
     * @todo Implement a timeout mechanism */
    while (1) {
        res = register_read(FUSB302_REGISTER_CONTROL0, &reg_control0);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to flush tx fifo!");
            return -EIO;
        }
        if ((reg_control0 & (1U << 6U)) == 0x00) {
            break;
        }
    }

    /* Return success */
    return 0;
}

/**
 * Tries to receive a message.
 * @param[out] msg A power delivery message structure.
 * @return 1 if a message was successfully received, 0 if no message was received, or a negative error code otherwise.
 */
int fusb302::pd_message_receive(struct usb_pd_message &msg) {
    int res;
    uint8_t buf[32];

    /* Read status1 register to see if the rx fifo contains data */
    uint8_t reg_status1 = 0;
    res = register_read(FUSB302_REGISTER_STATUS1, &reg_status1);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to check fifo status!");
        return -EIO;
    }

    /* Don't go further if the rx fifo does not contains data */
    if ((reg_status1 & (1U << 5U)) != 0) {
        return 0;
    }

    /* Read one byte to determine the destination of the message */
    res = register_read(FUSB302_REGISTER_FIFOS, &buf[0]);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to read destination from fifo!");
        return -EIO;
    }

    /* Discard the message if the destination is not sop */
    if ((buf[0] & 0xE0) != 0xE0) {
        CONFIG_FUSB302_LOG_FUNCTION("Dropping non sop message.");
        return 0;
    }

    /* Read two bytes to retrieve the header */
    res = register_read(FUSB302_REGISTER_FIFOS, &buf[0], 2);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to read header from fifo!");
        return -EIO;
    }

    /* Parse header */
    msg.header = (buf[1] << 8) | (buf[0] << 0);
    msg.object_count = (msg.header & 0x7000) >> 12;

    /* Read the payload */
    res = register_read(FUSB302_REGISTER_FIFOS, &buf[0], msg.object_count * 4 + 4);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to read payload from fifo!");
        return -EIO;
    }

    /* Retrieve objects */
    for (uint8_t i = 0; i < msg.object_count; i++) {
        msg.objects[i] = buf[i * 4 + 3];
        msg.objects[i] <<= 8;
        msg.objects[i] |= buf[i * 4 + 2];
        msg.objects[i] <<= 8;
        msg.objects[i] |= buf[i * 4 + 1];
        msg.objects[i] <<= 8;
        msg.objects[i] |= buf[i * 4 + 0];
    }

    /* Return success */
    return 1;
}

/**
 *
 * @param[out] msg A power delivery message structure.
 * @return 0 if the message was successfully sent, or a negative error code otherwise.
 * @see Section 5.6 of the USB Power Delivery Specification Revision 3.0, Version 1.1.
 * @see Table 29 of the FUSB302 datasheet, Revision 2.
 */
int fusb302::pd_message_send(const struct usb_pd_message msg) {
    int res;

    /* Ensure object count doesn't exceed limit */
    if (msg.object_count > 7) {
        return -EINVAL;
    }

    // /* Wait for the tx fifo to be empty
    //  * @todo Implement a timeout mechanism */
    // do {
    //     uint8_t reg_status1;
    //     res = register_read(FUSB302_REGISTER_STATUS1, &reg_status1);
    //     if (res != 0) {
    //         CONFIG_FUSB302_LOG_FUNCTION("Failed to check tx fifo status!");
    //         return -EIO;
    //     }
    //     if ((reg_status1 & 0x0C) == 0x08) {
    //         break;
    //     }
    // } while (1);

    // /* Return error if fifo is not empty */
    // uint8_t reg_status1;
    // res = register_read(FUSB302_REGISTER_STATUS1, &reg_status1);
    // if (res != 0) {
    //     CONFIG_FUSB302_LOG_FUNCTION("Failed to check tx fifo status!");
    //     return -EIO;
    // }
    // if (((reg_status1 & (1U << 3U)) == 0) ||  // !TX_EMPTY || TX_FULL
    //     ((reg_status1 & (1U << 2U)) != 0)) {
    //     CONFIG_FUSB302_LOG_FUNCTION("Tx fifo is busy!");
    //     return -EBUSY;
    // }

    /* Prepare buffer */
    uint8_t buf[5 + 2 + 4 * 7 + 4] = {0};
    uint8_t len = 0;

    /* Append start of packet sequence */
    buf[len++] = FUSB302_TOKEN_SYNC1;
    buf[len++] = FUSB302_TOKEN_SYNC1;
    buf[len++] = FUSB302_TOKEN_SYNC1;
    buf[len++] = FUSB302_TOKEN_SYNC2;

    /* Append payload */
    uint16_t header = msg.header;
    header &= ~(0x7000);
    header |= ((msg.object_count & 0b111) << 12);
    buf[len++] = FUSB302_TOKEN_PACKSYM | (0x1F & (2 + 4 * msg.object_count));
    buf[len++] = header >> 0;
    buf[len++] = header >> 8;
    for (uint8_t i = 0; i < msg.object_count; i++) {
        buf[len++] = msg.objects[i] >> 0;
        buf[len++] = msg.objects[i] >> 8;
        buf[len++] = msg.objects[i] >> 16;
        buf[len++] = msg.objects[i] >> 24;
    }

    /* Append crc and end of packet sequence */
    buf[len++] = FUSB302_TOKEN_JAMCRC;
    buf[len++] = FUSB302_TOKEN_EOP;
    buf[len++] = FUSB302_TOKEN_TXOFF;
    buf[len++] = FUSB302_TOKEN_TXON;

    /* For some reason, in my current setup, I have noticed that sometimes the i2c layer will only report that two bytes have been sent
     * So this retry mechanism is a workaround */
    for (unsigned int i = 0; i < CONFIG_FUSB302_I2C_ATTEMPTS; i++) {

        /* Flush tx fifo */
        res = pd_tx_flush();
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to flush fifo, retrying...");
            continue;
        }

        /* Send buffer */
        res = register_write(FUSB302_REGISTER_FIFOS, buf, len);
        if (res < 0) {
            CONFIG_FUSB302_LOG_FUNCTION("Failed to send message, retrying...");
            continue;
        }

        /* Return success */
        return 0;
    }

    /* Return error */
    CONFIG_FUSB302_LOG_FUNCTION("Giving up!");
    return -EIO;
}

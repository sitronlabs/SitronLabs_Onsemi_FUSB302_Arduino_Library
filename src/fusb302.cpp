/* Self header */
#include "fusb302.h"

/* Config */
#define CONFIG_FUSB302_RX_QUEUE_SIZE 5  //!<
#define CONFIG_FUSB302_I2C_ATTEMPTS 3   //!<
#ifndef CONFIG_FUSB302_LOG_FUNCTION
#define CONFIG_FUSB302_LOG_FUNCTION(...) (void)0  //!< Replace by { Serial.printf(__VA_ARGS__); Serial.println(); } to output on serial port
#endif

/**
 * @brief Initializes the FUSB302 device with the specified I2C configuration.
 *
 * Sets up the I2C communication interface for the FUSB302 device by storing the provided
 * I2C library instance and device address for future use.
 *
 * @param[in] i2c_library Reference to the TwoWire I2C library instance to use for communication
 * @param[in] i2c_address The 7-bit I2C address of the FUSB302 device
 * @return 0 on successful initialization, or a negative error code otherwise
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
 * @brief Controls the power state of the FUSB302 device.
 *
 * When powered on, all internal blocks are enabled. When powered off,
 * all internal blocks are disabled to minimize power consumption.
 *
 * @param[in] on true to power on the device, false to power off
 * @return 0 on successful power state change, or a negative error code otherwise
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
 * @brief Reads data from one or more FUSB302 registers.
 *
 * Performs a complete I2C read transaction including:
 * - Sending register address
 * - Reading requested number of bytes
 * - Validating received data length
 *
 * @param[in] address The starting register address to read from
 * @param[out] content Pointer to buffer where read data will be stored
 * @param[in] count Number of registers to read (defaults to 1)
 * @return 0 on successful read, or a negative error code otherwise
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
    m_i2c_library->requestFrom(m_i2c_address, (uint8_t)count, (uint8_t)true);
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
 * @brief Writes data to one or more FUSB302 registers.
 *
 * Performs a complete I2C write transaction including:
 * - Sending register address
 * - Writing provided data
 * - Verifying transaction completion
 *
 * @param[in] address The starting register address to write to
 * @param[in] content Pointer to buffer containing data to write
 * @param[in] count Number of registers to write (defaults to 1)
 * @return 0 on successful write, or a negative error code otherwise
 */
int fusb302::register_write(const enum fusb302_register address, const uint8_t *const content, const size_t count) {
    int res;

    /* Ensure library has been configured */
    if (m_i2c_library == NULL) {
        return -EINVAL;
    }

    /* Write register to device */
    m_i2c_library->beginTransmission(m_i2c_address);
    m_i2c_library->write((uint8_t)address);
    m_i2c_library->write(content, count);
    res = m_i2c_library->endTransmission(true);
    if (res != 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to write (%d)!", res);
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Configures the FUSB302 to present Rd (pull-down) resistors on both CC pins.
 *
 * This function:
 * - Disables pull-up resistors (PU_EN1 and PU_EN2)
 * - Enables pull-down resistors (PDWN1 and PDWN2)

 * @note This is the power-on default for FUSB302B but not for FUSB302T
 * @return 0 on successful configuration, or a negative error code otherwise
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
 * @brief Configures the FUSB302 to present Rp (pull-up) resistors on both CC pins.
 *
 * This function:
 * - Disables pull-down resistors (PDWN1 and PDWN2)
 * - Sets the HOST_CUR value based on desired current level
 * - Enables pull-up resistors (PU_EN1 and PU_EN2)
 *
 * The status parameter must be one of:
 * - USB_TYPEC_CC_STATUS_RP_DEF
 * - USB_TYPEC_CC_STATUS_RP_1_5
 * - USB_TYPEC_CC_STATUS_RP_3_0
 *
 * @param[in] status The current level to advertise via the Rp resistors
 * @return 0 on successful configuration, or a negative error code otherwise
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
 * @brief Measures the voltage levels on both CC pins to determine their status.
 *
 * This function performs different measurements based on whether Rd pull-down
 * or Rp pull-up resistors are enabled. For Rd mode, it measures voltage levels
 * to detect Rp advertisement. For Rp mode, it uses the comparator to detect Rd.
 *
 * @param[out] cc1 Reference to store the status of CC1 pin
 * @param[out] cc2 Reference to store the status of CC2 pin
 * @return 0 on successful measurement, or a negative error code otherwise
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

        /* Wait for at least 250 microseconds
         * @todo Doccument where this number comes from */
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

        /* Wait for at least 250 microseconds
         * @todo Doccument where this number comes from */
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

        /* Wait for at least 250 microseconds
         * @todo Doccument where this number comes from */
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

        /* Wait for at least 250 microseconds
         * @todo Doccument where this number comes from */
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
 * @brief Configures the FUSB302 for a specific USB Type-C cable orientation.
 *
 * This function:
 * - Configures RX by setting appropriate MEAS_CC1 or MEAS_CC2 bits
 * - Configures TX by setting appropriate TX_CC1 or TX_CC2 bits
 * - May interfere with DFP/SRC functionality
 *
 * The orientation parameter must be either USB_TYPEC_CC_ORIENTATION_NORMAL
 * or USB_TYPEC_CC_ORIENTATION_REVERSE.
 *
 * @param[in] orientation The cable orientation to configure (normal or reversed)
 * @return 0 on successful configuration, or a negative error code otherwise
 */
int fusb302::cc_orientation_set(const enum usb_typec_cc_orientation orientation) {
    int res;

    /* Ensure arguments are valid */
    if ((orientation != USB_TYPEC_CC_ORIENTATION_NORMAL) &&  //
        (orientation != USB_TYPEC_CC_ORIENTATION_REVERSE)) {
        return -EINVAL;
    }

    /* Handle RX by setting MEAS_CC1 or MEAS_CC2
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

    /* Handle TX by setting TX_CC1 or TX_CC2 */
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
 * Measures the VBUS voltage using the internal comparator and MDAC.
 *
 * This function temporarily modifies the SWITCHES0 register to disable CC measurements,
 * performs the VBUS measurement, then restores the original register state.
 *
 * @param[out] voltage_v The measured VBUS voltage (in volts).
 * @return 0 in case of success, or other negative error code otherwise.
 */
int fusb302::vbus_measure(float &voltage_v) {
    int res;

    /* Backup registers */
    uint8_t reg_switches0_before;
    res = register_read(FUSB302_REGISTER_SWITCHES0, &reg_switches0_before);
    if (res < 0) {
        return -EIO;
    }
    uint8_t reg_measure_before;
    res = register_read(FUSB302_REGISTER_MEASURE, &reg_measure_before);
    if (res < 0) {
        return -EIO;
    }

    /* Disable MEAS_CC */
    uint8_t reg_switches0 = reg_switches0_before & 0xF3;
    res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0);
    if (res < 0) {
        return -EIO;
    }

    /* Binary search through MDAC values (0-63) */
    int code_left = 0;
    int code_right = 63;
    int code_lower_last = -1;
    while (code_left <= code_right) {
        int code_mid = (code_left + code_right) / 2;

        /* Set MEAS_VBUS bit and MDAC value */
        uint8_t reg_measure = (1 << 6) | code_mid;
        res = register_write(FUSB302_REGISTER_MEASURE, &reg_measure);
        if (res < 0) {
            return -EIO;
        }

        /* Small delay to allow comparator to settle */
        delayMicroseconds(100);

        /* Look at COMP bit */
        uint8_t reg_status0;
        res = register_read(FUSB302_REGISTER_STATUS0, &reg_status0);
        if (res < 0) {
            return -EIO;
        }

        /* Look at COMP bit. If set, it means the measured voltage is higher than the one outputed by the DAC */
        if ((reg_status0 & (1 << 5)) == 0) {
            code_right = code_mid - 1;
        } else {
            code_lower_last = code_mid;
            code_left = code_mid + 1;
        }
    }

    /* Calculate voltage based on transition point
     * Each MDAC step is 0.42V, add 0.5 steps for better accuracy
     * If no transition found, voltage is below minimum measurable */
    if (code_lower_last >= 0) {
        voltage_v = (code_lower_last + 0.5) * 0.420;
    } else {
        voltage_v = 0.0;
    }

    /* Restore registers */
    res = register_write(FUSB302_REGISTER_SWITCHES0, &reg_switches0_before);
    if (res < 0) {
        return -EIO;
    }
    res = register_write(FUSB302_REGISTER_MEASURE, &reg_measure_before);
    if (res < 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Resets the USB Power Delivery logic of the FUSB302.
 *
 * Performs a soft reset of only the PD logic block, leaving other device
 * functionality unaffected. This is useful for recovering from protocol
 * errors or reinitializing PD communication.
 *
 * @return 0 on successful reset, or a negative error code otherwise
 */
int fusb302::pd_reset_logic(void) {

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
 * @brief Sends a USB Power Delivery Hard Reset sequence.
 *
 * @return 0 on successful transmission, or a negative error code otherwise
 * @see Section 6.6.3.3 of the USB Power Delivery Specification
 */
int fusb302::pd_reset_hard(void) {
    int res;

    /* Set hard reset bit */
    uint8_t reg_control3 = 0;
    res = register_read(FUSB302_REGISTER_CONTROL3, &reg_control3);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to read register!");
        return -EIO;
    }
    reg_control3 |= (1 << 6);
    res = register_write(FUSB302_REGISTER_CONTROL3, &reg_control3);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to write register!");
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Enables or disables automatic GoodCRC response for received USB PD messages.
 *
 * When enabled, the FUSB302 will automatically send a GoodCRC message in
 * response to any valid received PD message. This is required for normal
 * USB PD operation according to the specification.
 *
 * @param[in] enabled true to enable automatic GoodCRC response, false to disable
 * @return 0 on successful configuration, or a negative error code otherwise
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
 * @brief Configures automatic retry behavior for USB PD message transmission.
 *
 * This function configures how many times the FUSB302 will automatically
 * retry sending a message if no GoodCRC response is received. Setting
 * retries to 0 disables automatic retries.
 *
 * The retries parameter must be between 0 and 3 inclusive.
 *
 * @param[in] retries Number of retry attempts (0-3) for message transmission
 * @return 0 on successful configuration, or a negative error code otherwise
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
 * @brief Flushes the USB PD receive FIFO buffer.
 *
 * Clears any pending received messages from the RX FIFO and waits for
 * the clear operation to complete. A timeout mechanism should be
 * implemented in future versions.
 *
 * @return 0 on successful flush operation, or a negative error code otherwise
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
 * @brief Flushes the USB PD transmit FIFO buffer.
 *
 * Clears any pending messages from the TX FIFO and waits for the clear
 * operation to complete. A timeout mechanism should be implemented in
 * future versions.
 *
 * @return 0 on successful flush operation, or a negative error code otherwise
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
 * @brief Attempts to receive a USB Power Delivery message from the FUSB302.
 *
 * This function:
 * - Checks if data is available in the RX FIFO
 * - Reads the SOP type
 * - Extracts and parses the message header
 * - Retrieves the message payload if present
 *
 * @param[out] msg Reference to a power delivery message structure where the received message will be stored
 * @return 1 if a message was successfully received
 *         0 if no message was available to receive
 *         negative error code if an error occurred during reception
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

    /* Read the first three bytes to extract the sop and the header */
    res = register_read(FUSB302_REGISTER_FIFOS, &buf[0], 3);
    if (res < 0) {
        CONFIG_FUSB302_LOG_FUNCTION("Failed to read sop and header from fifo!");
        return -EIO;
    }

    /* Determine the type of sop */
    if ((buf[0] & 0xE0) == 0xE0) {
        msg.sop_type = USB_PD_SOP_TYPE_DEFAULT;
    } else if ((buf[0] & 0xE0) == 0xC0) {
        msg.sop_type = USB_PD_SOP_TYPE_PRIME;
    } else if ((buf[0] & 0xE0) == 0xA0) {
        msg.sop_type = USB_PD_SOP_TYPE_PRIME_DOUBLE;
    } else {
        msg.sop_type = USB_PD_SOP_TYPE_UNKNOWN;
    }

    /* Parse header */
    msg.header = (buf[2] << 8) | (buf[1] << 0);
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
 * @brief Sends a USB Power Delivery message through the FUSB302.
 *
 * This function handles the complete message transmission process including:
 * - Adding SOP sequence (supports SOP, SOP', and SOP'' based on msg.sop_type)
 * - Formatting header and payload
 * - Adding CRC and EOP sequence
 * - Retrying on transmission failures
 *
 * @param[in] msg The power delivery message structure to send
 * @return 0 if the message was successfully sent, or a negative error code otherwise
 * @see Section 5.6 of the USB Power Delivery Specification Revision 3.0, Version 1.1
 * @see Table 29 of the FUSB302 datasheet, Revision 2
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
    switch (msg.sop_type) {
        case USB_PD_SOP_TYPE_DEFAULT: {
            buf[len++] = FUSB302_TOKEN_SYNC1;
            buf[len++] = FUSB302_TOKEN_SYNC1;
            buf[len++] = FUSB302_TOKEN_SYNC1;
            buf[len++] = FUSB302_TOKEN_SYNC2;
            break;
        }
        case USB_PD_SOP_TYPE_PRIME: {
            buf[len++] = FUSB302_TOKEN_SYNC1;
            buf[len++] = FUSB302_TOKEN_SYNC1;
            buf[len++] = FUSB302_TOKEN_SYNC3;
            buf[len++] = FUSB302_TOKEN_SYNC3;
            break;
        }
        case USB_PD_SOP_TYPE_PRIME_DOUBLE: {
            buf[len++] = FUSB302_TOKEN_SYNC1;
            buf[len++] = FUSB302_TOKEN_SYNC3;
            buf[len++] = FUSB302_TOKEN_SYNC1;
            buf[len++] = FUSB302_TOKEN_SYNC3;
            break;
        }
        default: {
            CONFIG_FUSB302_LOG_FUNCTION("Invalid SOP type!");
            return -EINVAL;
        }
    }

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

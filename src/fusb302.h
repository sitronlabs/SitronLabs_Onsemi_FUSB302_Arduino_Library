#ifndef FUSB302_H
#define FUSB302_H

/* Other headers */
#include "usb_pd.h"
#include "usb_typec.h"

/* Arduino libraries */
#include <Arduino.h>
#include <Wire.h>

/* C/C++ libraries */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief FUSB302 register addresses
 *
 * List of all accessible registers in the FUSB302 device.
 * See FUSB302 datasheet for detailed register descriptions.
 */
enum fusb302_register {
    FUSB302_REGISTER_DEVICE_ID = 0x01,   //!< Device ID register
    FUSB302_REGISTER_SWITCHES0 = 0x02,   //!< Switches0 control register
    FUSB302_REGISTER_SWITCHES1 = 0x03,   //!< Switches1 control register
    FUSB302_REGISTER_MEASURE = 0x04,     //!< Measure control register
    FUSB302_REGISTER_SLICE = 0x05,       //!< Slice control register
    FUSB302_REGISTER_CONTROL0 = 0x06,    //!< Control0 register
    FUSB302_REGISTER_CONTROL1 = 0x07,    //!< Control1 register
    FUSB302_REGISTER_CONTROL2 = 0x08,    //!< Control2 register
    FUSB302_REGISTER_CONTROL3 = 0x09,    //!< Control3 register
    FUSB302_REGISTER_MASK1 = 0x0A,       //!< Mask1 register
    FUSB302_REGISTER_POWER = 0x0B,       //!< Power control register
    FUSB302_REGISTER_RESET = 0x0C,       //!< Reset control register
    FUSB302_REGISTER_MASKA = 0x0E,       //!< MaskA register
    FUSB302_REGISTER_MASKB = 0x0F,       //!< MaskB register
    FUSB302_REGISTER_STATUS0A = 0x3C,    //!< Status0A register
    FUSB302_REGISTER_STATUS1A = 0x3D,    //!< Status1A register
    FUSB302_REGISTER_INTERRUPTA = 0x3E,  //!< InterruptA register
    FUSB302_REGISTER_INTERRUPTB = 0x3F,  //!< InterruptB register
    FUSB302_REGISTER_STATUS0 = 0x40,     //!< Status0 register
    FUSB302_REGISTER_STATUS1 = 0x41,     //!< Status1 register
    FUSB302_REGISTER_INTERRUPT = 0x42,   //!< Interrupt register
    FUSB302_REGISTER_FIFOS = 0x43,       //!< TX/RX FIFO access register
};

/**
 * @brief FUSB302 TX FIFO token values
 *
 * Special tokens used when writing to the TX FIFO for message transmission.
 * These control packet framing and transmission behavior.
 */
enum fusb302_txfifo_tokens {
    FUSB302_TOKEN_TXON = 0xA1,     //!< Enable transmitter
    FUSB302_TOKEN_SYNC1 = 0x12,    //!< First sync token
    FUSB302_TOKEN_SYNC2 = 0x13,    //!< Second sync token
    FUSB302_TOKEN_SYNC3 = 0x1B,    //!< Third sync token
    FUSB302_TOKEN_RST1 = 0x15,     //!< First reset token
    FUSB302_TOKEN_RST2 = 0x16,     //!< Second reset token
    FUSB302_TOKEN_PACKSYM = 0x80,  //!< Packet symbol
    FUSB302_TOKEN_JAMCRC = 0xFF,   //!< JAM CRC token
    FUSB302_TOKEN_EOP = 0x14,      //!< End of packet token
    FUSB302_TOKEN_TXOFF = 0xFE,    //!< Disable transmitter
};

/**
 * @brief FUSB302 USB Type-C Port Controller - Main Control Class
 *
 * This class provides the main interface to the FUSB302 chip for USB Type-C
 * and Power Delivery operations.
 *
 * Capabilities:
 * - Detect USB-C cable orientation
 * - Measure VBUS voltage (5V, 9V, 12V, 15V, 20V, etc.)
 * - Send and receive USB Power Delivery messages
 * - Request specific voltages from USB-C chargers
 * - Manage chip power and communication
 *
 * Basic Usage:
 * 1. Call setup() to initialize with I2C
 * 2. Call detect() to verify chip is connected
 * 3. Call power_set(true) to turn on the chip
 * 4. Use cc_measure() to detect cables
 * 5. Use pd_* functions for Power Delivery
 */
class fusb302 {
   public:
    /** @name Setup Functions
     * Basic chip initialization and power control
     */
    /**@{*/

    /**
     * Initialize the FUSB302 chip
     * @param i2c_library Reference to Wire object (usually just 'Wire')
     * @param i2c_address I2C address of chip (typically 0x22)
     * @return 0 on success, negative error code on failure
     */
    int setup(TwoWire &i2c_library, const uint8_t i2c_address);

    /**
     * Check if FUSB302 is connected and responding
     * @return true if chip detected, false if not found
     */
    bool detect(void);

    /**
     * Perform a software reset of the chip
     * @return 0 on success, negative error code on failure
     */
    int reset(void);

    /**
     * Turn the power on or off for the internal circuitry of the chip
     * @param on true to power on, false to power off
     * @return 0 on success, negative error code on failure
     */
    int power_set(const bool on);

    /**@}*/

    /** @name Register Access Functions
     * Low-level register read/write operations (advanced usage)
     */
    /**@{*/

    /**
     * Read from chip registers
     * @param address Register to read from
     * @param content Pointer to store read data
     * @param count Number of bytes to read (default: 1)
     * @return 0 on success, negative error code on failure
     */
    int register_read(const enum fusb302_register address, uint8_t *const content, const size_t count = 1);

    /**
     * Write to chip registers
     * @param address Register to write to
     * @param content Pointer to data to write
     * @param count Number of bytes to write (default: 1)
     * @return 0 on success, negative error code on failure
     */
    int register_write(const enum fusb302_register address, const uint8_t *const content, const size_t count = 1);

    /**@}*/

    /** @name USB Type-C Functions
     * CC pin measurement and orientation control
     */
    /**@{*/

    /**
     * Enable pull-down resistors on CC pins (for acting as device/sink)
     * @return 0 on success, negative error code on failure
     */
    int cc_pull_down(void);

    /**
     * Enable pull-up resistors on CC pins (for acting as host/source)
     * @param status The pull-up resistor value to advertise power capability
     * @return 0 on success, negative error code on failure
     */
    int cc_pull_up(const enum usb_typec_cc_status status);

    /**
     * Measure CC pins to detect cable connection and orientation
     * Determines if a cable is plugged in and its orientation
     * @param cc1 Output: status of CC1 pin
     * @param cc2 Output: status of CC2 pin
     * @return 0 on success, negative error code on failure
     */
    int cc_measure(enum usb_typec_cc_status &cc1, enum usb_typec_cc_status &cc2);

    /**
     * Set which CC pin to use for communication
     * Call this after detecting cable orientation
     * @param orientation USB_TYPEC_CC_ORIENTATION_NORMAL or REVERSE
     * @return 0 on success, negative error code on failure
     */
    int cc_orientation_set(const enum usb_typec_cc_orientation orientation);

    /**@}*/

    /** @name Voltage Measurement
     * VBUS voltage monitoring
     */
    /**@{*/

    /**
     * Measure the VBUS voltage (the power line)
     * Useful for verifying what voltage your USB-C source is providing
     * @param voltage Output: measured voltage in Volts
     * @return 0 on success, negative error code on failure
     */
    int vbus_measure(float &voltage);

    /**@}*/

    /** @name USB Power Delivery Functions
     * PD message transmission and reception
     */
    /**@{*/

    /**
     * Reset the USB Power Delivery logic
     * Call this before starting PD communication
     * @return 0 on success, negative error code on failure
     */
    int pd_reset(void);

    /**
     * Enable/disable automatic GoodCRC message responses
     * When enabled, chip automatically responds to received messages
     * @param enabled true to enable auto-GoodCRC, false to handle manually
     * @return 0 on success, negative error code on failure
     */
    int pd_autogoodcrc_set(const bool enabled);

    /**
     * Set number of automatic transmission retries
     * @param retries Number of retry attempts (0-3)
     * @return 0 on success, negative error code on failure
     */
    int pd_autoretry_set(const int retries);

    /**
     * Clear the receive FIFO buffer
     * @return 0 on success, negative error code on failure
     */
    int pd_rx_flush(void);

    /**
     * Clear the transmit FIFO buffer
     * @return 0 on success, negative error code on failure
     */
    int pd_tx_flush(void);

    /**
     * Check for and receive a USB Power Delivery message
     * @param msg Output: received message structure
     * @return 1 if message received, 0 if no message, negative on error
     */
    int pd_message_receive(struct usb_pd_message &msg);

    /**
     * Send a USB Power Delivery message
     * Can be used to request different voltages or communicate with USB-C devices
     * @param msg The message to send
     * @return 0 on success, negative error code on failure
     */
    int pd_message_send(const struct usb_pd_message msg);

    /**@}*/

   protected:
    TwoWire *m_i2c_library = NULL;  //!< Pointer to the I2C library instance
    uint8_t m_i2c_address;          //!< I2C address of the FUSB302 device
};

#endif

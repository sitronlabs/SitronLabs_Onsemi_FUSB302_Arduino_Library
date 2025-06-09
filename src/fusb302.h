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
 * @brief FUSB302 USB Type-C Port Controller driver class
 *
 * This class provides an interface to control the FUSB302 USB Type-C Port Controller.
 * It handles:
 * - I2C communication with the device
 * - USB Type-C CC pin configuration and measurement
 * - USB Power Delivery message transmission and reception
 * - Device power management and reset control
 */
class fusb302 {
   public:
    //!@{
    //! Setup
    int setup(TwoWire &i2c_library, const uint8_t i2c_address);
    bool detect(void);
    int reset(void);
    int power_set(const bool on);
    //!@}

    //!@{
    //! Register access
    int register_read(const enum fusb302_register address, uint8_t *const content, const size_t count = 1);
    int register_write(const enum fusb302_register address, const uint8_t *const content, const size_t count = 1);
    //!@}

    //!@{
    //! CC logic
    int cc_pull_down(void);
    int cc_pull_up(const enum usb_typec_cc_status status);
    int cc_measure(enum usb_typec_cc_status &cc1, enum usb_typec_cc_status &cc2);
    int cc_orientation_set(const enum usb_typec_cc_orientation orientation);
    //!@}

    //!@{
    //! Vbus
    int vbus_measure(float &voltage);
    //!@}

    //!@{
    //! PD logic
    int pd_reset(void);
    int pd_autogoodcrc_set(const bool enabled);
    int pd_autoretry_set(const int retries);
    int pd_rx_flush(void);
    int pd_tx_flush(void);
    int pd_message_receive(struct usb_pd_message &msg);
    int pd_message_send(const struct usb_pd_message msg);
    //!@}

   protected:
    TwoWire *m_i2c_library = NULL;  //!< Pointer to the I2C library instance
    uint8_t m_i2c_address;          //!< I2C address of the FUSB302 device
};

#endif

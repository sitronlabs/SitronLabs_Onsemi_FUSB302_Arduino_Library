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

/** List of registers */
enum fusb302_register {
    FUSB302_REGISTER_DEVICE_ID = 0x01,
    FUSB302_REGISTER_SWITCHES0 = 0x02,
    FUSB302_REGISTER_SWITCHES1 = 0x03,
    FUSB302_REGISTER_MEASURE = 0x04,
    FUSB302_REGISTER_SLICE = 0x05,
    FUSB302_REGISTER_CONTROL0 = 0x06,
    FUSB302_REGISTER_CONTROL1 = 0x07,
    FUSB302_REGISTER_CONTROL2 = 0x08,
    FUSB302_REGISTER_CONTROL3 = 0x09,
    FUSB302_REGISTER_MASK1 = 0x0A,
    FUSB302_REGISTER_POWER = 0x0B,
    FUSB302_REGISTER_RESET = 0x0C,
    FUSB302_REGISTER_MASKA = 0x0E,
    FUSB302_REGISTER_MASKB = 0x0F,
    FUSB302_REGISTER_STATUS0A = 0x3C,
    FUSB302_REGISTER_STATUS1A = 0x3D,
    FUSB302_REGISTER_INTERRUPTA = 0x3E,
    FUSB302_REGISTER_INTERRUPTB = 0x3F,
    FUSB302_REGISTER_STATUS0 = 0x40,
    FUSB302_REGISTER_STATUS1 = 0x41,
    FUSB302_REGISTER_INTERRUPT = 0x42,
    FUSB302_REGISTER_FIFOS = 0x43,
};

/** List of tokens defined for the tx fifo */
enum fusb302_txfifo_tokens {
    FUSB302_TOKEN_TXON = 0xA1,
    FUSB302_TOKEN_SYNC1 = 0x12,
    FUSB302_TOKEN_SYNC2 = 0x13,
    FUSB302_TOKEN_SYNC3 = 0x1B,
    FUSB302_TOKEN_RST1 = 0x15,
    FUSB302_TOKEN_RST2 = 0x16,
    FUSB302_TOKEN_PACKSYM = 0x80,
    FUSB302_TOKEN_JAMCRC = 0xFF,
    FUSB302_TOKEN_EOP = 0x14,
    FUSB302_TOKEN_TXOFF = 0xFE,
};

/**
 *
 */
class fusb302 {
   public:
    /* Setup */
    int setup(TwoWire &i2c_library, const uint8_t i2c_address);
    bool detect(void);
    int reset(void);
    int power_set(const bool on);

    /* Register access */
    int register_read(const enum fusb302_register address, uint8_t *const content, const size_t count = 1);
    int register_write(const enum fusb302_register address, const uint8_t *const content, const size_t count = 1);

    /* CC logic */
    int cc_pull_down(void);
    int cc_pull_up(const enum usb_typec_cc_status status);
    int cc_measure(enum usb_typec_cc_status &cc1, enum usb_typec_cc_status &cc2);
    int cc_orientation_set(const enum usb_typec_cc_orientation orientation);

    /* Vbus */
    int vbus_measure(float &voltage);

    /* PD logic */
    int pd_reset(void);
    int pd_autogoodcrc_set(const bool enabled);
    int pd_autoretry_set(const int retries);
    int pd_rx_flush(void);
    int pd_tx_flush(void);
    int pd_message_receive(struct usb_pd_message &msg);
    int pd_message_send(const struct usb_pd_message msg);

   protected:
    TwoWire *m_i2c_library = NULL;
    uint8_t m_i2c_address;
};

#endif

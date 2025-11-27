[![Designed by Sitron Labs](https://img.shields.io/badge/Designed_by-Sitron_Labs-FCE477.svg)](https://www.sitronlabs.com/)
[![Join the Discord community](https://img.shields.io/discord/552242187665145866.svg?logo=discord&logoColor=white&label=Discord&color=%237289da)](https://discord.gg/btnVDeWhfW)
[![PayPal Donate](https://img.shields.io/badge/PayPal-Donate-00457C.svg?logo=paypal&logoColor=white)](https://www.paypal.com/donate/?hosted_button_id=QLX8VU9Q3PFFL)
![License](https://img.shields.io/github/license/sitronlabs/SitronLabs_Onsemi_FUSB302_Arduino_Library.svg)
![Latest Release](https://img.shields.io/github/release/sitronlabs/SitronLabs_Onsemi_FUSB302_Arduino_Library.svg)
[![Arduino Library Manager](https://www.ardu-badge.com/badge/Sitron%20Labs%20FUSB302%20Arduino%20Library.svg)](https://www.ardu-badge.com/Sitron%20Labs%20FUSB302%20Arduino%20Library)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/sitronlabs/library/Sitron_Labs_FUSB302_Arduino_Library.svg)](https://registry.platformio.org/libraries/sitronlabs/Sitron_Labs_FUSB302_Arduino_Library)

# Sitron Labs Onsemi FUSB302 Arduino Library

Arduino library for interfacing with the Onsemi FUSB302 USB Type-C Port Controller.

## Description

The FUSB302 is a programmable USB Type-C controller with USB Power Delivery support. It can detect USB Type-C cable orientation, measure VBUS voltage, and handle USB Power Delivery protocol communication. This library provides a simple interface to access these features via I2C, supporting both FUSB302B (default sink) and FUSB302T (default source) variants.

## Installation

### Arduino IDE

Install via the Arduino Library Manager by searching for "Sitron Labs FUSB302".

Alternatively, install manually:
1. Download or clone this repository
2. Place it in your Arduino `libraries` folder
3. Restart the Arduino IDE

### PlatformIO

Install via the PlatformIO Library Manager by searching for "Sitron Labs FUSB302".

Alternatively, add the library manually to your `platformio.ini` file:

```ini
lib_deps = 
    https://github.com/sitronlabs/SitronLabs_Onsemi_FUSB302_Arduino_Library.git
```

## Hardware Connections

Connect the FUSB302 to your Arduino using I2C:

- VDD → 3.3V or 5.0V
- GND → GND
- SDA → SDA (I2C Data, requires 4.7kΩ pull-up to VDD)
- SCL → SCL (I2C Clock, requires 4.7kΩ pull-up to VDD)
- CC1 → USB Type-C connector CC1 pin
- CC2 → USB Type-C connector CC2 pin
- VBUS → USB Type-C connector VBUS pin (up to 20V)
- INT → Any digital pin (optional, for interrupts)

**Important Notes:**
- The FUSB302 can operate at 3.3V or 5.0V. I2C pull-up resistors should be connected to the same voltage as VDD.
- I2C lines require 4.7kΩ pull-up resistors to VDD.
- The default I2C address is 0x22 (FUSB302BMPX) or 0x22/0x23 (FUSB302TMPX, configurable).

## Usage

### Basic Cable Detection

```cpp
#include <Wire.h>
#include <fusb302.h>

// Create FUSB302 device object
fusb302 usb_controller;

// I2C address (typically 0x22)
const uint8_t I2C_ADDRESS = 0x22;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin();
  
  // Setup the FUSB302 (I2C library, I2C address)
  if (usb_controller.setup(Wire, I2C_ADDRESS) != 0) {
    Serial.println("Failed to setup FUSB302");
    return;
  }
  
  // Detect the device
  if (!usb_controller.detect()) {
    Serial.println("FUSB302 not detected");
    return;
  }
  
  // Power on the device
  usb_controller.power_set(true);
  
  // Configure CC pins as pull-down (for acting as device/sink)
  usb_controller.cc_pull_down();
  
  Serial.println("FUSB302 initialized");
}

void loop() {
  // Measure CC pins to detect cable orientation
  enum usb_typec_cc_status cc1, cc2;
  if (usb_controller.cc_measure(cc1, cc2) == 0) {
    if (cc1 > USB_TYPEC_CC_STATUS_OPEN && cc2 == USB_TYPEC_CC_STATUS_OPEN) {
      Serial.println("Cable: Normal orientation");
      usb_controller.cc_orientation_set(USB_TYPEC_CC_ORIENTATION_NORMAL);
    } else if (cc1 == USB_TYPEC_CC_STATUS_OPEN && cc2 > USB_TYPEC_CC_STATUS_OPEN) {
      Serial.println("Cable: Flipped orientation");
      usb_controller.cc_orientation_set(USB_TYPEC_CC_ORIENTATION_REVERSE);
    } else {
      Serial.println("No cable detected");
    }
  }
  
  delay(1000);
}
```

### VBUS Voltage Monitoring

```cpp
void loop() {
  float voltage;
  if (usb_controller.vbus_measure(voltage) == 0) {
    Serial.print("VBUS: ");
    Serial.print(voltage, 2);
    Serial.println(" V");
  }
  delay(1000);
}
```

### USB Power Delivery Communication

```cpp
#include <Wire.h>
#include <fusb302.h>
#include <usb_pd.h>

fusb302 usb_controller;
const uint8_t I2C_ADDRESS = 0x22;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  usb_controller.setup(Wire, I2C_ADDRESS);
  usb_controller.detect();
  usb_controller.power_set(true);
  usb_controller.cc_pull_down();
  
  // Initialize Power Delivery
  usb_controller.pd_reset_logic();
  usb_controller.pd_autogoodcrc_set(true);
  usb_controller.pd_autoretry_set(3);
  usb_controller.pd_rx_flush();
  
  Serial.println("PD initialized");
}

void loop() {
  // Check for received PD messages
  struct usb_pd_message msg;
  int result = usb_controller.pd_message_receive(msg);
  
  if (result == 1) {
    Serial.print("Received PD message: Header=0x");
    Serial.println(msg.header, HEX);
    // Process message...
  } else if (result < 0) {
    Serial.println("Error receiving message");
  }
  
  delay(100);
}
```

## API Reference

### setup(TwoWire &i2c_library, uint8_t i2c_address)

Initializes the FUSB302 device.

- `i2c_library`: I2C library instance to use (typically `Wire`)
- `i2c_address`: I2C address (typically 0x22)

Returns 0 on success, or a negative error code otherwise.

### detect(void)

Detects if the FUSB302 device is present on the I2C bus by reading the device ID register.

Returns true if the device is detected, false otherwise.

### reset(void)

Performs a software reset of the entire chip.

Returns 0 on success, or a negative error code otherwise.

### power_set(bool on)

Controls the power state of the FUSB302 device.

- `on`: true to power on, false to power off

Returns 0 on success, or a negative error code otherwise.

### cc_pull_down(void)

Configures the FUSB302 to present Rd (pull-down) resistors on both CC pins. Use this when acting as a device/sink.

Returns 0 on success, or a negative error code otherwise.

### cc_pull_up(enum usb_typec_cc_status status)

Configures the FUSB302 to present Rp (pull-up) resistors on both CC pins. Use this when acting as a host/source.

- `status`: The current level to advertise (USB_TYPEC_CC_STATUS_RP_DEF, USB_TYPEC_CC_STATUS_RP_1_5, or USB_TYPEC_CC_STATUS_RP_3_0)

Returns 0 on success, or a negative error code otherwise.

### cc_measure(enum usb_typec_cc_status &cc1, enum usb_typec_cc_status &cc2)

Measures the voltage levels on both CC pins to determine their status and detect cable orientation.

- `cc1`: Output parameter for CC1 pin status
- `cc2`: Output parameter for CC2 pin status

Returns 0 on success, or a negative error code otherwise.

### cc_orientation_set(enum usb_typec_cc_orientation orientation)

Configures the FUSB302 for a specific USB Type-C cable orientation. Call this after detecting cable orientation.

- `orientation`: USB_TYPEC_CC_ORIENTATION_NORMAL or USB_TYPEC_CC_ORIENTATION_REVERSE

Returns 0 on success, or a negative error code otherwise.

### vbus_measure(float &voltage)

Measures the VBUS voltage using the internal comparator and MDAC.

- `voltage`: Output parameter for the measured voltage in Volts

Returns 0 on success, or a negative error code otherwise.

### pd_reset_logic(void)

Resets the USB Power Delivery logic of the FUSB302. Call this before starting PD communication.

Returns 0 on success, or a negative error code otherwise.

### pd_reset_hard(void)

Sends a USB Power Delivery Hard Reset sequence.

Returns 0 on success, or a negative error code otherwise.

### pd_autogoodcrc_set(bool enabled)

Enables or disables automatic GoodCRC response for received USB PD messages.

- `enabled`: true to enable automatic GoodCRC response, false to disable

Returns 0 on success, or a negative error code otherwise.

### pd_autoretry_set(int retries)

Configures automatic retry behavior for USB PD message transmission.

- `retries`: Number of retry attempts (0-3)

Returns 0 on success, or a negative error code otherwise.

### pd_rx_flush(void)

Flushes the USB PD receive FIFO buffer.

Returns 0 on success, or a negative error code otherwise.

### pd_tx_flush(void)

Flushes the USB PD transmit FIFO buffer.

Returns 0 on success, or a negative error code otherwise.

### pd_message_receive(struct usb_pd_message &msg)

Checks for and receives a USB Power Delivery message from the FUSB302.

- `msg`: Output parameter for the received message structure

Returns 1 if a message was successfully received, 0 if no message was available, or a negative error code otherwise.

### pd_message_send(const struct usb_pd_message msg)

Sends a USB Power Delivery message through the FUSB302.

- `msg`: The message structure to send

Returns 0 on success, or a negative error code otherwise.

**Message structure:**
```cpp
struct usb_pd_message {
    enum usb_pd_sop_type sop_type;  // SOP, SOP', or SOP''
    uint16_t header;                 // Message header
    uint32_t objects[7];             // Data objects
    uint8_t object_count;            // Number of objects (0-7)
};
```

### register_read(enum fusb302_register address, uint8_t *content, size_t count = 1)

Low-level register read operation (advanced usage).

- `address`: Register address to read from
- `content`: Pointer to buffer where read data will be stored
- `count`: Number of registers to read (default: 1)

Returns 0 on success, or a negative error code otherwise.

### register_write(enum fusb302_register address, const uint8_t *content, size_t count = 1)

Low-level register write operation (advanced usage).

- `address`: Register address to write to
- `content`: Pointer to buffer containing data to write
- `count`: Number of registers to write (default: 1)

Returns 0 on success, or a negative error code otherwise.

## Specifications

- Supported devices: FUSB302B (default sink), FUSB302T (default source)
- Communication interface: I2C
- I2C address: 0x22 (FUSB302BMPX) or 0x22/0x23 (FUSB302TMPX, configurable)
- Operating voltage: 3.3V or 5.0V
- VBUS measurement range: 0-22V
- USB Power Delivery: Full support for PD protocol
- SOP types: SOP, SOP', and SOP''
- CC pin configuration: Pull-up (Rp) or pull-down (Rd) resistors

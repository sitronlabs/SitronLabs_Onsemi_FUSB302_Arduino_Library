# Sitron Labs FUSB302 Arduino Library

[![Version](https://img.shields.io/badge/version-0.3.0-blue.svg)](https://github.com/sitronlabs/SitronLabs_Onsemi_FUSB302_Arduino_Library)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE.txt)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-orange.svg)](https://www.arduino.cc/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-blue.svg)](https://platformio.org/)

A professional, easy-to-use Arduino library for the Onsemi FUSB302 USB Type-C Port Controller. This library provides comprehensive support for USB Type-C cable detection, orientation sensing, VBUS voltage monitoring, and USB Power Delivery protocol communication.

## Features

- **USB Type-C Detection**: Automatic cable orientation detection (normal/flipped)
- **Power Delivery Support**: Full USB PD message transmission and reception
- **PD Hard Reset**: Send hard reset messages to force PD state machine reset
- **VBUS Monitoring**: Accurate voltage measurement (0-22V range)
- **Multiple SOP Types**: Support for SOP, SOP', and SOP'' messaging
- **Flexible Configuration**: CC pin configuration for source and sink modes
- **I2C Communication**: Simple I2C interface (typically at 0x22)
- **Comprehensive Examples**: Detailed example sketches included
- **Professional Documentation**: Extensive inline documentation and Doxygen support
- **Error Handling**: Robust error checking and reporting
- **Maker-Friendly**: Designed with ease of use and learning in mind

## Supported Devices

| Device | Description |
|--------|-------------|
| FUSB302B | Programmable USB Type-C Controller with PD (Default SNK) |
| FUSB302T | Programmable USB Type-C Controller with PD (Default SRC) |

## Installation

### Arduino IDE Library Manager
1. Open Arduino IDE
2. Go to **Tools** → **Manage Libraries**
3. Search for "SitronLabs FUSB302"
4. Click **Install**

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps = 
    sitronlabs/SitronLabs_FUSB302_Arduino_Library
```

### Manual Installation
1. Download the latest release from [GitHub](https://github.com/sitronlabs/SitronLabs_Onsemi_FUSB302_Arduino_Library/releases)
2. Extract the ZIP file
3. Copy the `SitronLabs_Onsemi_FUSB302_Arduino_Library` folder to your Arduino `libraries` directory
4. Restart Arduino IDE

## Quick Start

### Basic Cable Detection Example
```cpp
#include <Wire.h>
#include <fusb302.h>

fusb302 usb_controller;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    /* Initialize FUSB302 at I2C address 0x22 */
    usb_controller.setup(Wire, 0x22);
    
    if (!usb_controller.detect()) {
        Serial.println("FUSB302 not found!");
        while(1);
    }
    
    usb_controller.power_set(true);
    Serial.println("FUSB302 ready!");
}

void loop() {
    /* Read CC pins to detect cable orientation */
    enum usb_typec_cc_status cc1, cc2;
    usb_controller.cc_measure(cc1, cc2);
    
    if (cc1 > USB_TYPEC_CC_STATUS_OPEN && cc2 == USB_TYPEC_CC_STATUS_OPEN) {
        Serial.println("Cable: Normal orientation");
    } 
    else if (cc1 == USB_TYPEC_CC_STATUS_OPEN && cc2 > USB_TYPEC_CC_STATUS_OPEN) {
        Serial.println("Cable: Flipped orientation");
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

## Hardware Connections

### Basic I2C Connection
| FUSB302 Pin | Arduino Pin | Description |
|-------------|-------------|-------------|
| VDD | 3.3V | Power supply (3.3V only!) |
| GND | GND | Ground |
| SDA | SDA | I2C data line (4.7kΩ pull-up) |
| SCL | SCL | I2C clock line (4.7kΩ pull-up) |
| CC1 | USB-C CC1 | Configuration channel 1 |
| CC2 | USB-C CC2 | Configuration channel 2 |
| VBUS | USB-C VBUS | USB power line (up to 20V) |
| INT | Digital pin | Interrupt (optional) |

### Important Notes
- **Voltage**: FUSB302 operates at 3.3V only. Use level shifters with 5V Arduino boards.
- **Pull-ups**: I2C lines require 4.7kΩ pull-up resistors to 3.3V.
- **VBUS**: Can measure 0-22V directly. Use voltage divider for higher voltages.

### I2C Address Configuration
| Device | Default Address |
|--------|----------------|
| FUSB302BMPX | 0x22 |
| FUSB302TMPX | 0x22 or 0x23 (configurable) |

## API Reference

### Initialization
```cpp
/* Setup I2C communication */
int setup(TwoWire &i2c_library, uint8_t i2c_address);

/* Detect if chip is present */
bool detect();

/* Software reset */
int reset();

/* Power control */
int power_set(bool on);
```

### USB Type-C Functions
```cpp
/* Measure CC pin status */
int cc_measure(enum usb_typec_cc_status &cc1, enum usb_typec_cc_status &cc2);

/* Set cable orientation */
int cc_orientation_set(enum usb_typec_cc_orientation orientation);

/* Configure CC pull resistors */
int cc_pull_down();
int cc_pull_up(enum usb_typec_cc_status status);
```

### Voltage Measurement
```cpp
/* Measure VBUS voltage */
int vbus_measure(float &voltage);
```

### USB Power Delivery
```cpp
/* PD initialization */
int pd_reset();
int pd_autogoodcrc_set(bool enabled);
int pd_autoretry_set(int retries);

/* Message handling */
int pd_message_receive(struct usb_pd_message &msg);
int pd_message_send(const struct usb_pd_message msg);

/* Buffer management */
int pd_rx_flush();
int pd_tx_flush();
```

### USB PD Message Structure
```cpp
struct usb_pd_message {
    enum usb_pd_sop_type sop_type;  /* SOP, SOP', or SOP'' */
    uint16_t header;                 /* Message header */
    uint32_t objects[7];             /* Data objects */
    uint8_t object_count;            /* Number of objects (0-7) */
};
```

## Error Handling

All methods return an integer error code:
- `0`: Success
- `1`: Success with data (for `pd_message_receive()`)
- Negative values: Error codes

Common error codes:
- `-EIO`: I2C communication error
- `-EINVAL`: Invalid parameter
- `-EBUSY`: Device busy
- `-ENODEV`: Device not found

## Troubleshooting

### Device Not Detected
1. Check power supply is 3.3V (not 5V!)
2. Verify I2C wiring (SDA, SCL, GND)
3. Confirm I2C address (use I2C scanner)
4. Check pull-up resistors on I2C lines (4.7kΩ)
5. Ensure chip has proper power (check VDD pin)

### Cable Not Detected
1. Verify CC1 and CC2 are connected to USB-C connector
2. Test with a different USB-C cable
3. Ensure cable is fully inserted
4. Check for proper CC pin termination

### No Power Delivery Messages
1. Reset PD logic: `usb_controller.pd_reset()`
2. Verify cable orientation is set correctly
3. Confirm connected device supports USB PD
4. Enable auto-GoodCRC: `usb_controller.pd_autogoodcrc_set(true)`
5. Check that VBUS is present

### Incorrect VBUS Reading
1. Verify VBUS connection to chip
3. Ensure measurement range (0-22V)
4. Test with known voltage source

## License

This library is released under the MIT License. See [LICENSE.txt](LICENSE.txt) for details.

## Additional Resources

- [FUSB302 Datasheet](doc/onsemi%20FUSB302B%20Datasheet.pdf)
- [USB Type-C Specification](https://www.usb.org/document-library/usb-type-cr-cable-and-connector-specification-release-24)
- [USB Power Delivery Specification](https://www.usb.org/document-library/usb-power-delivery)

## Support

For support, please:
1. Check the [Issues](https://github.com/sitronlabs/SitronLabs_Onsemi_FUSB302_Arduino_Library/issues) page
2. Join our [Discord community](https://discord.gg/b6VzayWAMZ) for live support and discussion

---

**Sitron Labs** - Helping makers build what matters

[Website](https://sitronlabs.com) | [Store](https://www.sitronlabs.com/store) | [Discord](https://discord.gg/b6VzayWAMZ) | [GitHub](https://github.com/sitronlabs)

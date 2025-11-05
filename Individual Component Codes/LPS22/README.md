# LPS22

This repository contains a simple test sketch for using the LPS22 pressure sensor with an ESP32 via the I2C protocol. This guide clarifies the specific wiring for boards that use `SCK`, `SDI`, and `SDO` pin labels for I2C communication.

## Libraries Needed

To use this sensor, you must install the following libraries through the Arduino IDE Library Manager:

  * `Adafruit LPS2X`
  * `Adafruit Unified Sensor`
  * `Adafruit BusIO`

## Pinout and Wiring

This sensor uses I2C communication. Connect the pins to your ESP32 as follows.

| LPS22 Pin | ESP32 Pin   | Description      |
| :-------- | :---------- | :--------------- |
| **VIN** | **3.3V** | Power            |
| **GND** | **GND** | Ground           |
| **SCK** | **GPIO 22** | I2C Clock (`SCL`)  |
| **SDI** | **GPIO 21** | I2C Data (`SDA`)   |

**Important Note on Pin Names:**
Some breakout boards, like the one this guide is based on, use non-standard names for I2C pins:

  * **SCK** is used for the I2C clock line (connect to `SCL`).
  * **SDI** is used for the I2C data line (connect to `SDA`).
  * **SDO** is **not** for data; it is the **I2C Address selector pin**. See the "Notes" section for details.

-----

## Expected Output

After uploading the code, open the Arduino Serial Monitor with the baud rate set to **115200**. You should see the following output, updating every second:

```
LPS22 Test using I2C
LPS22 Found!
Temperature: 26.50 C
Pressure: 1012.34 hPa

Temperature: 26.51 C
Pressure: 1012.33 hPa

...
```

-----

## Notes

  * **I2C Address**: The LPS22 has a default I2C address. On boards with an `SDO` pin, you can change this address.
      * Leave `SDO` **unconnected** for the default address (often `0x5D`).
      * Connect `SDO` to **GND** to switch to the alternate address (`0x5C`).
  * **3.3V Logic**: The sensor is a 3.3V device, which is perfectly compatible with the ESP32's logic level.

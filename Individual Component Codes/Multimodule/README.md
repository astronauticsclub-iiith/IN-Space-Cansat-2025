# Multimodule


This repository contains a test sketch for the Bosch BME680, a 4-in-1 environmental sensor that measures temperature, humidity, barometric pressure, and volatile organic compounds (VOCs) for air quality monitoring. This guide provides the necessary code and wiring instructions for use with an ESP32 via the I2C protocol.

## Libraries Needed

You must install the following libraries through the Arduino IDE Library Manager (**Sketch** -\> **Include Library** -\> **Manage Libraries...**) to use this sensor:

  * `Adafruit BME680 Library`
  * `Adafruit Unified Sensor`
  * `Adafruit BusIO`

-----

## ESP32 Pinout

The BME680 is a 3.3V device, making it directly compatible with the ESP32's power and logic levels. Connect the sensor using the default I2C pins.

| BME680 Pin | ESP32 Pin   |
| :--------- | :---------- |
| **VIN** | **3.3V** |
| **GND** | **GND** |
| **SCL** | **GPIO 22** |
| **SDA** | **GPIO 21** |

-----

## Expected Output

After uploading the code, open the Arduino Serial Monitor with the baud rate set to **115200**. You should see a continuous stream of data updating every two seconds:

```
BME680 Test
Temperature = 25.40 *C
Pressure = 1013.25 hPa
Humidity = 45.50 %
Gas = 50.12 KOhms
Approx. Altitude = 0.00 m

...
```

-----

## Notes

  * **I2C Address**: The default I2C address for the BME680 is **0x77**. If the scanner or code fails to find it, check if your board has an address jumper that might have it set to the alternate address, **0x76**.
  * **Gas Sensor (VOC)**: The gas sensor reading is given in Ohms of resistance. It does not measure a specific gas but rather the total concentration of Volatile Organic Compounds (VOCs).
      * **Higher resistance** generally indicates **cleaner air**.
      * **Lower resistance** indicates the presence of pollutants like ethanol, acetone, or carbon monoxide.
  * **Warm-up Period**: The gas sensor requires a "burn-in" period to achieve stable readings. For the first use, let it run for several minutes. For high-accuracy applications, a burn-in of up to 20 minutes is recommended by Bosch.

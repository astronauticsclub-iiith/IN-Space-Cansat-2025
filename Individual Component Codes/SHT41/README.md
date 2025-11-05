# SHT41
A high-precision I2C sensor for measuring temperature and relative humidity. This sketch demonstrates how to initialize the sensor and read its values.

---

### Libraries Needed
* `Adafruit SHT4x Library`
* `Adafruit BusIO`
* `Adafruit Unified Sensor`

---

### ESP32 Pinout
| SHT41 Pin | ESP32 Pin   |
| :-------- | :---------- |
| **VIN** | **3.3V** |
| **GND** | **GND** |
| **SCL** | **GPIO 22** |
| **SDA** | **GPIO 21** |

---

### Expected Output

```
Adafruit SHT41 Test 
Found SHT41 sensor 
Temperature: 25.60 °C 
Humidity: 45.80 % RH
Temperature: 25.62 °C 
Humidity: 45.75 % RH 
...
```

---

### Notes
* **I2C Communication**: This is an I2C device. The default SCL and SDA pins on most ESP32 boards are GPIO 22 and 21, respectively.
* **3.3V Logic**: The SHT41 is a 3.3V device, making it directly compatible with the ESP32's logic levels and power output.
* **Address**: The default I2C address for the SHT41 sensor is **0x44**. Most breakout boards are fixed to this address and cannot be changed.

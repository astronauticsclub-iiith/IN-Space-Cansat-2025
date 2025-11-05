# MQ5

A basic test sketch for the MQ-5 sensor to detect LPG, natural gas, and coal gas. This code reads the raw analog value, which increases in the presence of detectable gases.

---

### Libraries Needed
* None. This test uses the standard `analogRead()` function.

---

### ESP32 Pinout
| MQ-5 Pin | ESP32 Pin      |
| :------- | :------------- |
| **VCC** | **VIN (5V)** |
| **GND** | **GND** |
| **A0** | **GPIO 34** |

---

### Expected Output

```
MQ-5 Sensor Test on ESP32 - Analog
Sensor warming up for 20 seconds...
Warm-up complete.
Starting readings.
Raw Sensor Value: 350
Raw Sensor Value: 352
...
```

---

### Notes
* **Warm-up is critical**: Like other MQ sensors, the MQ-5 requires a 20-second warm-up period for its heater to stabilize.
* **5V Power**: This sensor must be powered with 5V for its heater to function correctly.
* **Digital Pin**: The module also has a `D0` digital output pin that can be used to trigger an alarm. Its sensitivity can be adjusted using the onboard blue potentiometer.
* **Safety**: When testing, use an **unlit** butane lighter or another safe, controlled gas source. Never use this sensor in an environment where a spark could cause an ignition.

File

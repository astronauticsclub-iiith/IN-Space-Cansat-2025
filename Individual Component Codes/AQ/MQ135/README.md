# MQ135

A simple test sketch to read approximate CO₂ PPM (Parts Per Million) values from an MQ-135 Air Quality Sensor using an ESP32.

---

### Libraries Needed
* `MQ135` by GeorgK (G. Krocker)

---

### ESP32 Pinout
| MQ-135 Pin | ESP32 Pin      |
| :--------- | :------------- |
| **VCC** | **VIN (5V)** |
| **GND** | **GND** |
| **A0** | **GPIO 34** |

---

### Expected Output
```
MQ-135 PPM Test on ESP32 Allowing sensor to warm up for 20 seconds... 
Warm-up complete. 
Starting readings. 
CO2 PPM (approx): 415.32 
CO2 PPM (approx): 416.10 
...
```


---

### Notes
* **Warm-up is critical**: The sensor requires at least 20 seconds for its internal heater to reach a stable temperature. For long-term projects, it should remain continuously powered.
* **5V Power**: The sensor's heater needs 5V for optimal performance. Use the `VIN` pin when powering the ESP32 via USB.
* **ADC1 Pin Required**: Analog readings must be taken on an **ADC1** pin (e.g., GPIO 32-39). This is crucial if you plan to use Wi-Fi, as the Wi-Fi module conflicts with ADC2.
* **Calibration**: The PPM value is an **approximation**. For accurate results, the sensor needs to be calibrated by measuring its resistance (`RZERO`) in air with a known CO₂ concentration (typically 400 PPM).

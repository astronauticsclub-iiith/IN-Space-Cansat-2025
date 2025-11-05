# Data (SD Card)

This project provides an Arduino sketch to benchmark the performance and verify the data integrity of an SD card with an ESP32. The script performs a sustained write-read-verify cycle, making it a useful tool for evaluating SD card suitability for data logging applications.

The script repeatedly performs the following cycle:

1.  **Generates** 20 lines of sample CSV data in memory.
2.  **Writes** this data to a file on the SD card as quickly as possible and measures the write speed.
3.  **Reads** the data back from the file and measures the read speed.
4.  **Verifies** that the data read from the card exactly matches the original data, reporting a pass or fail status.

## Hardware Requirements

  * An ESP32 Development Board
  * A MicroSD Card Module (with SPI interface)
  * A FAT32-formatted MicroSD Card
  * Jumper Wires

-----

## Software & Libraries

  * **Arduino IDE** with the [ESP32 board manager](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html) installed.
  * **SPI.h** (Included with the ESP32 core)
  * **SD.h** (Included with the ESP32 core)

-----

## Wiring 🔌

The test uses the ESP32's default **VSPI** (Virtual SPI) pins.

| SD Card Pin | ESP32 Pin      |
| :---------- | :------------- |
| **CS** | **GPIO 5** |
| **SCK** | **GPIO 18** |
| **MISO** | **GPIO 19** |
| **MOSI** | **GPIO 23** |
| **VCC** | **3.3V** |
| **GND** | **GND** |

-----

## Installation & Usage

1.  **Format SD Card**: Ensure your MicroSD card is formatted with a **FAT32** file system.
2.  **Connect Hardware**: Wire the SD card module to your ESP32 according to the pinout table above.
3.  **Upload Code**: Open the `.ino` sketch in the Arduino IDE, select your ESP32 board and port, and upload the code.
4.  **Monitor Output**: Open the **Serial Monitor** and set the baud rate to **115200**. The test will start automatically and repeat every 5 seconds.

-----

## Expected Output

The Serial Monitor will display the results for each test cycle, showing the write/read speeds and the final data integrity status.

```
Initializing SD card...initialization done.
Generating test data...

--- Starting Write Test ---
Wrote 20 lines in 15 ms.
Write Speed: 1333.33 lines/second.

--- Starting Read & Verification Test ---
Read and verified 20 lines in 2 ms.
Read Speed: 10000.00 lines/second.
✅ Data Integrity: PASSED

---------------------------------
Test cycle complete. Starting new test in 5 seconds...
```

If the data read from the card does not match the original data, the test will immediately fail and print the mismatched lines:

```
--- Starting Read & Verification Test ---
!!! DATA MISMATCH! !!!
Original Data: 12345,42.10,1013.25,10
Data from SD : 12345,42.10,1013.25,1
Read and verified 11 lines in 1 ms.
Read Speed: 11000.00 lines/second.
❌ Data Integrity: FAILED
```

# 🚀 ESP32 + Python API Integration Guide

## 📋 Quick Summary

You now have **two working systems**:

### 🎲 **main.py** (Port 8001) - Random Data
- **Purpose**: Testing & development with simulated data
- **Use when**: No hardware available, testing GUI, demonstrations

### 📡 **parser.py** (Port 8002) - Real ESP32 Data  
- **Purpose**: Live data from your ESP32 hardware
- **Use when**: Connected to real ESP32 CanSat device

---

## 🔧 **Setup Steps**

### 1. **Upload ESP32 Code**
- Upload the updated `ESP_GCS.ino` to your ESP32
- Make sure `#define CANSAT_MODE` is uncommented
- The code now sends: `PING:Packet:123,TS:54321,Alt:150.5,Temp:25.3,...`

### 2. **Install Dependencies**
```bash
cd /Users/va/GCS-3D/API
pip install -r requirements.txt
```

### 3. **Find Your ESP32 Port**
```bash
python find_ports.py
# Look for ports with "CP2102" or "Silicon Labs" in description
# Likely ports: /dev/cu.usbserial-* or /dev/cu.SLAB_USBtoUART
```

### 4. **Start the Parser API**
```bash
./start.sh
# Choose option 2: Parser API (Serial Data)
```

### 5. **Connect to ESP32**
The API will show available ports on startup. Connect via:
- API endpoint: `POST /serial/connect` with your port
- Or use the helper: `python connect_esp32.py`

---

## 📊 **Port Connection Logging**

The parser now shows **detailed connection information**:

```
============================================================
🔌 ATTEMPTING SERIAL CONNECTION
============================================================
📍 Port: /dev/cu.usbserial-0001
⚡ Baud Rate: 115200
============================================================
🔗 Opening serial connection...
✅ SERIAL CONNECTION SUCCESSFUL!
📡 Connected to: /dev/cu.usbserial-0001  
⚡ Baud rate: 115200
🎯 Status: ACTIVE - Listening for data...
============================================================
```

---

## 🧪 **Testing & Verification**

### Test Format Compatibility:
```bash
python simple_test.py
```

### Quick ESP32 Connection:
```bash  
python connect_esp32.py
```

### Find Available Ports:
```bash
python find_ports.py
```

---

## 📡 **Expected Data Flow**

1. **ESP32 sends**: `PING:Packet:123,TS:54321,Alt:150.5,Temp:25.3,Pressure:1015.2,Lat:28.6140,Lon:77.2095,Voltage:3.3,State:1`

2. **Parser receives & logs**: 
   ```
   <<< Received Telemetry: PING:Packet:123,TS:54321,Alt:150.5,...
   ```

3. **API converts to JSON**:
   ```json
   {
     "timestamp": 1696291200,
     "altitude": 150.5,
     "temperature": 25.3,
     "pressure": 1015.2,
     "battery": 47.1,
     "gnss": {"lat": 28.6140, "lng": 77.2095},
     "system_state": 1
   }
   ```

4. **GUI fetches**: `GET http://localhost:8002/telemetry`

---

## 🔍 **Troubleshooting**

### **"No ports found"**
- Check USB connection to ESP32
- Install ESP32 drivers (CP2102/Silicon Labs)
- Try different USB cables/ports

### **"Port in use"** 
- Close Arduino IDE or other serial monitors
- Disconnect other applications using the port

### **"No data received"**
- Verify ESP32 is in `CANSAT_MODE`
- Check baud rate (115200)
- Confirm ESP32 is running and sending data

### **Wrong data format**
- Upload the updated `ESP_GCS.ino` code
- Check ESP32 serial monitor output matches expected format

---

## 🌐 **API Endpoints**

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/telemetry` | GET | Get latest telemetry data |
| `/serial/ports` | GET | List available serial ports |
| `/serial/connect` | POST | Connect to ESP32 port |
| `/serial/status` | GET | Check connection status |
| `/serial/disconnect` | POST | Disconnect from port |
| `/command` | POST | Send commands to ESP32 |

---

## 🎯 **Final Result**

- ✅ **Port connection clearly displayed**
- ✅ **ESP32 format fully compatible** 
- ✅ **Real-time data parsing working**
- ✅ **Easy port discovery tools**
- ✅ **Comprehensive logging & debugging**

Your GUI can now connect to `http://localhost:8002/telemetry` to get live data from your ESP32 CanSat! 🚀
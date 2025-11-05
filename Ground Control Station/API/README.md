# Telemetry API Documentation

This project contains two separate APIs for handling telemetry data:

## 🎲 main.py - Random Data API (Port 8001)
**Purpose**: Serves random/simulated telemetry data for testing and development
- **Port**: 8001
- **Data Source**: Random generation or CSV files
- **Use Case**: Testing GUI, demonstrations, development when hardware isn't available

```bash
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8001
```

## 📡 parser.py - Serial Data API (Port 8002)
**Purpose**: Reads real telemetry data from serial/COM ports
- **Port**: 8002  
- **Data Source**: Serial communication with hardware devices
- **Use Case**: Live data from CanSat, Arduino, ESP32, or other serial devices

```bash
python -m uvicorn parser:app --reload --host 0.0.0.0 --port 8002
```

## 🔍 Port Discovery Tool
Use the `find_ports.py` script to easily identify the correct COM port:

```bash
python find_ports.py
```

This tool will:
- List all available serial ports
- Show device descriptions and manufacturers
- Test ports for incoming data
- Help you identify the correct port for your device

## 📋 Setup Instructions

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Find Your COM Port
```bash
python find_ports.py
```

### 3. Start the APIs

**For Random Data (Testing):**
```bash
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8001
```

**For Serial Data (Live Hardware):**
```bash
python -m uvicorn parser:app --reload --host 0.0.0.0 --port 8002
```

## 🔌 Serial Connection API Endpoints

### Connect to Serial Port
```http
POST /serial/connect
Content-Type: application/json

{
    "port": "/dev/tty.usbmodem11301",
    "baud_rate": 115200
}
```

### List Available Ports
```http
GET /serial/ports
```

### Get Connection Status
```http
GET /serial/status
```

### Send Commands to Device
```http
POST /command
Content-Type: application/json

{
    "command": "START_MISSION",
    "source": "GUI"
}
```

## 📊 Common Telemetry Endpoints (Both APIs)

### Get Telemetry Data
```http
GET /telemetry
```

### Get CSV Status
```http
GET /csv-status
```

## 🔧 Expected Serial Data Format

The parser expects data in this format:
```
PING:Packet:123,TS:54321,Alt:300.5,Temp:25.3,Lat:28.6139,Lon:77.209,Pressure:1013.25
```

Key mappings:
- `Alt` or `Altitude` → Altitude
- `Temp` → Temperature  
- `Lat` → Latitude
- `Lon` → Longitude
- `Pressure` or `Press` → Pressure
- `Voltage` → Battery (converted to percentage)
- `State` → System State
- `TS` → Timestamp

## 🗂️ File Structure

```
API/
├── main.py              # Random data API (Port 8001)
├── parser.py            # Serial data API (Port 8002)
├── find_ports.py        # Port discovery tool
├── demo.py              # API demonstration script
├── start.sh             # Interactive startup script
├── requirements.txt     # Dependencies
├── README.md           # This file
└── Data/
    ├── data_log.csv    # Serial data logging
    └── telemetry_example*.csv  # Sample data files
```

## 🛠️ Helper Scripts

- **`start.sh`**: Interactive menu to start APIs, find ports, or install dependencies
- **`find_ports.py`**: Discover and test serial ports with your devices
- **`demo.py`**: Test both APIs and see the differences

## 🚀 Quick Start

### Option 1: Use the startup script (easiest)
```bash
./start.sh
```

### Option 2: Manual setup
1. **Install dependencies**: `pip install -r requirements.txt`
2. **Find your port**: `python find_ports.py`
3. **Start serial API**: `python -m uvicorn parser:app --reload --host 0.0.0.0 --port 8002`
4. **Connect via API**: POST to `/serial/connect` with your port
5. **Get telemetry**: GET `/telemetry`

### Option 3: Test both APIs
```bash
python demo.py
```

## 🐛 Troubleshooting

- **No ports found**: Check USB connections and drivers
- **Permission denied**: On Linux/macOS, you may need `sudo` or add user to dialout group
- **Port in use**: Close other applications using the serial port
- **No data received**: Check baud rate, device is sending data, and data format

## 📱 GUI Integration

Both APIs expose the same telemetry data format, so your GUI can easily switch between:
- `localhost:8001/telemetry` (Random data)
- `localhost:8002/telemetry` (Serial data)
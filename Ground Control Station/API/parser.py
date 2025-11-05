''' Serial Telemetry Parser API
Command to run:
python -m uvicorn parser:app --reload --host 0.0.0.0 --port 8002
'''
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import serial
import serial.tools.list_ports
import threading
import csv
import time
import os
import random
from typing import Optional, Dict, Any, List

app = FastAPI(title="Serial Telemetry Parser API")

# Add CORS middleware for GUI integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify actual origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- CONFIGURATION ---
BAUD_RATE = 115200
CSV_FILENAME = 'Data/data_log.csv'
TEAM_ID = '2025'
# --------------------

# Global variables
ser = None
serial_lock = threading.Lock()
latest_telemetry = None
telemetry_lock = threading.Lock()
receiver_thread = None
is_connected = False

# Define models for parts of the telemetry data
class GNSS(BaseModel):
    lat: float
    lng: float

class Gyro(BaseModel):
    pitch: float
    yaw: float
    roll: float

class Acceleration(BaseModel):
    x: float
    y: float
    z: float

class SensorStatus(BaseModel):
    gnss: str
    altimetry: str
    pressure: str
    temperature: str
    gyro: str
    power: str
    airQuality: str

class Telemetry(BaseModel):
    timestamp: float
    altitude: float
    temperature: float
    pressure: float
    humidity: float
    battery: float
    gnss: GNSS
    gyro: Gyro
    acceleration: Acceleration
    airQuality: float
    sensor_status: SensorStatus
    system_state: int

# Configuration models
class SerialConfig(BaseModel):
    port: str
    baud_rate: Optional[int] = 115200

class SerialPortInfo(BaseModel):
    port: str
    description: str
    manufacturer: Optional[str] = None
    is_connected: bool = False

class Command(BaseModel):
    command: str
    timestamp: Optional[float] = None
    source: Optional[str] = "GUI"

class CommandResponse(BaseModel):
    message: str
    command_received: str
    timestamp: float
    status: str

# CSV Header for logging
CSV_HEADER = [
    "TeamID", "Timestamp", "Packet", "State", "Altitude", "Pressure", "Temp", 
    "Voltage", "Lat", "Lon", "Sats", "BNO_qi", "BNO_qj", "BNO_qk", "BNO_qr", 
    "SHT_Temp", "SHT_Hum", "BME_Press", "BME_Temp", "BME_Hum", "BME_Gas", 
    "BME_IAQ", "BMP_Press", "BMP_Temp", "ADS_V"
]

def setup_csv():
    """Creates the CSV file and writes the header if it doesn't exist."""
    os.makedirs(os.path.dirname(CSV_FILENAME), exist_ok=True)
    if not os.path.exists(CSV_FILENAME):
        with open(CSV_FILENAME, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(CSV_HEADER)
        print(f"Created CSV file: {CSV_FILENAME}")

def parse_serial_data_to_telemetry(data_string: str) -> Telemetry:
    """Parse telemetry data from serial and convert to Telemetry object"""
    print(f"<<< Received Telemetry: {data_string}")
    
    # Default values
    timestamp = time.time()
    altitude = 0.0
    temperature = 25.0
    pressure = 1013.25
    humidity = 50.0
    battery = 100.0
    lat = 28.6139
    lng = 77.209
    pitch = 0.0
    roll = 0.0
    yaw = 0.0
    airQuality = 10.0
    system_state = 0
    packet_count = 0
    
    try:
        # Handle different message formats
        if data_string.startswith("PING:"):
            data_string = data_string[5:]
        elif data_string.startswith("Receiving <-") or data_string.startswith("Received <-"):
            # Handle LoRa receiver format: "Received <- 'PING:...'" 
            start_quote = data_string.find("'")
            end_quote = data_string.rfind("'")
            if start_quote != -1 and end_quote != -1:
                data_string = data_string[start_quote+1:end_quote]
                if data_string.startswith("PING:"):
                    data_string = data_string[5:]
        
        # Parse key-value pairs
        pairs = data_string.split(',')
        for pair in pairs:
            if ':' in pair:
                key, value = pair.split(':', 1)
                try:
                    if key == 'Packet':
                        packet_count = int(value)
                    elif key == 'Altitude' or key == 'Alt':
                        altitude = float(value)
                    elif key == 'Temp' or key == 'Temperature':
                        temperature = float(value)
                    elif key == 'Pressure' or key == 'Press':
                        pressure = float(value)
                    elif key == 'Humidity' or key == 'Hum':
                        humidity = float(value)
                    elif key == 'Voltage':
                        # Convert voltage to battery percentage
                        voltage = float(value)
                        if voltage <= 5.0:  # Assume 3.3V system
                            battery = max(0, min(100, ((voltage - 2.5) / (4.2 - 2.5)) * 100))
                        else:  # Higher voltage system
                            battery = max(0, min(100, (voltage / 12.0) * 100))
                    elif key == 'Lat':
                        lat = float(value)
                    elif key == 'Lon':
                        lng = float(value)
                    elif key == 'Pitch':
                        pitch = float(value)
                    elif key == 'Roll':
                        roll = float(value)
                    elif key == 'Yaw':
                        yaw = float(value)
                    elif key == 'AirQuality' or key == 'Gas':
                        airQuality = float(value)
                    elif key == 'State':
                        system_state = int(value)
                    elif key == 'TS':
                        # Use device timestamp if provided (convert from millis to seconds)
                        device_ts = float(value)
                        if device_ts > 1000000000:  # Looks like Unix timestamp
                            timestamp = device_ts
                        else:  # Looks like millis since boot
                            timestamp = time.time()  # Use current time
                except (ValueError, TypeError) as e:
                    print(f"[WARNING] Could not parse {key}={value}: {e}")
                    continue  # Skip invalid values
        
        # Save to CSV
        save_to_csv(data_string, {
            'TeamID': TEAM_ID,
            'Timestamp': timestamp,
            'Packet': packet_count,
            'Altitude': altitude,
            'Temp': temperature,
            'Pressure': pressure,
            'Voltage': battery if battery <= 5.0 else battery / 8.33,  # Keep original voltage for CSV
            'Lat': lat,
            'Lon': lng,
            'State': system_state
        })
        
    except Exception as e:
        print(f"[ERROR] Could not parse data: {data_string}. Reason: {e}")
    
    # Create telemetry object
    gnss = GNSS(lat=lat, lng=lng)
    gyro = Gyro(pitch=pitch, yaw=yaw, roll=roll)
    acceleration = Acceleration(
        x=random.uniform(-2, 2),
        y=random.uniform(-2, 2),
        z=random.uniform(-2, 2)
    )
    
    # Calculate sensor statuses
    sensor_status = SensorStatus(
        gnss="ACTIVE" if lat != 0 and lng != 0 else "ERROR",
        altimetry="ERROR" if altitude < 50 else "ACTIVE",
        pressure="ERROR" if pressure < 1000 or pressure > 1020 else "ACTIVE",
        temperature="ERROR" if temperature < 0 or temperature > 40 else "ACTIVE",
        gyro="ACTIVE",
        power="ERROR" if battery < 20 else "ACTIVE",
        airQuality="ERROR" if airQuality > 35 else "ACTIVE"
    )
    
    return Telemetry(
        timestamp=timestamp,
        altitude=altitude,
        temperature=temperature,
        pressure=pressure,
        humidity=humidity,
        battery=battery,
        gnss=gnss,
        gyro=gyro,
        acceleration=acceleration,
        airQuality=airQuality,
        sensor_status=sensor_status,
        system_state=system_state
    )

def save_to_csv(raw_data: str, parsed_data: Dict[str, Any]):
    """Save telemetry data to CSV file"""
    try:
        # Create a dictionary with default values for all header fields
        data_row = {key: 'N/A' for key in CSV_HEADER}
        
        # Fill in the parsed data
        for key, value in parsed_data.items():
            if key in CSV_HEADER:
                data_row[key] = value
        
        # Write to CSV
        with open(CSV_FILENAME, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=CSV_HEADER)
            writer.writerow(data_row)
            
    except Exception as e:
        print(f"[ERROR] Could not save to CSV: {e}")

def read_from_port():
    """Thread function to continuously read data from the serial port."""
    global latest_telemetry, is_connected
    print("Receiver thread started. Listening for data...")
    
    while True:
        try:
            with serial_lock:
                if ser and ser.is_open and ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        # Check if line contains telemetry data
                        if ("PING:" in line) or ("Packet:" in line and "TS:" in line):
                            # Parse and store latest telemetry
                            telemetry = parse_serial_data_to_telemetry(line)
                            with telemetry_lock:
                                latest_telemetry = telemetry
                        else:
                            # Print any other non-empty messages
                            print(f"<<< Received Msg: {line}")
        except serial.SerialException as e:
            print(f"Serial error: {e}. Disconnecting.")
            is_connected = False
            break
        except UnicodeDecodeError as e:
            print(f"Unicode decode error: {e}")
            continue
        except Exception as e:
            print(f"An error occurred in receiver thread: {e}")
            time.sleep(1)
        
        time.sleep(0.1)  # Small delay to prevent excessive CPU usage

def generate_fallback_telemetry() -> Telemetry:
    """Generate fallback telemetry data when no serial data is available"""
    timestamp = time.time()
    
    gnss = GNSS(lat=28.6139, lng=77.209)
    gyro = Gyro(pitch=0.0, yaw=0.0, roll=0.0)
    acceleration = Acceleration(x=0.0, y=0.0, z=0.0)
    sensor_status = SensorStatus(
        gnss="ERROR",
        altimetry="ERROR",
        pressure="ERROR",
        temperature="ERROR",
        gyro="ERROR",
        power="ERROR",
        airQuality="ERROR"
    )
    
    return Telemetry(
        timestamp=timestamp,
        altitude=0.0,
        temperature=0.0,
        pressure=0.0,
        humidity=0.0,
        battery=0.0,
        gnss=gnss,
        gyro=gyro,
        acceleration=acceleration,
        airQuality=0.0,
        sensor_status=sensor_status,
        system_state=0
    )

# API Endpoints

def interactive_port_selection():
    """Interactive port selection on startup"""
    try:
        available_ports = serial.tools.list_ports.comports()
        
        if not available_ports:
            print(f"⚠️  No serial ports detected")
            print(f"💡 You can connect later via API: POST /serial/connect")
            return None
        
        print(f"\n� AVAILABLE SERIAL PORTS:")
        print(f"{'='*50}")
        
        esp32_ports = []
        other_ports = []
        
        for i, port in enumerate(available_ports, 1):
            desc = port.description.lower() if port.description else ""
            mfg = port.manufacturer.lower() if port.manufacturer else ""
            
            # Identify likely ESP32 ports
            is_esp32 = any(keyword in desc or keyword in mfg for keyword in 
                          ['cp210', 'cp2102', 'silicon labs', 'esp32', 'uart bridge'])
            
            if is_esp32:
                esp32_ports.append((i, port))
                print(f"   {i}. {port.device} ⭐ (ESP32-like)")
            else:
                other_ports.append((i, port))
                print(f"   {i}. {port.device}")
            
            print(f"      {port.description or 'Unknown device'}")
        
        print(f"\n📌 OPTIONS:")
        print(f"   1-{len(available_ports)} - Connect to specific port")
        print(f"   s - Skip connection (connect later via API)")
        print(f"   q - Quit application")
        
        if esp32_ports:
            recommended = esp32_ports[0][1].device
            print(f"\n💡 Recommended: {recommended} (ESP32-like device)")
        
        while True:
            choice = input(f"\nSelect option: ").strip().lower()
            
            if choice == 's':
                print(f"⏭️  Skipping connection - you can connect later via API")
                return None
            elif choice == 'q':
                print(f"👋 Goodbye!")
                exit(0)
            else:
                try:
                    port_num = int(choice)
                    if 1 <= port_num <= len(available_ports):
                        selected_port = available_ports[port_num - 1]
                        return selected_port.device
                    else:
                        print(f"❌ Invalid selection. Please choose 1-{len(available_ports)}, 's', or 'q'")
                except ValueError:
                    print(f"❌ Invalid input. Please enter a number, 's', or 'q'")
    
    except Exception as e:
        print(f"❌ Error during port selection: {e}")
        return None

def auto_connect_to_port(port_name):
    """Automatically connect to a port during startup"""
    global ser, receiver_thread, is_connected
    
    print(f"\n🔌 Auto-connecting to: {port_name}")
    
    try:
        with serial_lock:
            ser = serial.Serial(port_name, BAUD_RATE, timeout=1)
            is_connected = True
        
        # Start the receiver thread
        receiver_thread = threading.Thread(target=read_from_port, daemon=True)
        receiver_thread.start()
        
        print(f"✅ Successfully connected to {port_name}")
        print(f"📡 Listening for data...")
        
        return True
        
    except serial.SerialException as e:
        print(f"❌ Failed to connect to {port_name}: {e}")
        is_connected = False
        return False

@app.on_event("startup")
async def startup_event():
    setup_csv()
    print(f"\n{'='*60}")
    print(f"🚀 SERIAL TELEMETRY PARSER API STARTED")
    print(f"{'='*60}")
    print(f"� Data Source: Serial Communication")
    print(f"🌐 Server running on port 8002")
    print(f"� CSV Logging: {CSV_FILENAME}")
    print(f"{'='*60}")
    
    # Interactive port selection
    import threading
    import time
    
    def delayed_port_selection():
        time.sleep(1)  # Give server time to fully start
        selected_port = interactive_port_selection()
        if selected_port:
            auto_connect_to_port(selected_port)
        print(f"\n{'='*60}")
        print(f"🌐 API Ready at: http://localhost:8002")
        print(f"📊 Telemetry endpoint: http://localhost:8002/telemetry")
        print(f"{'='*60}\n")
    
    # Run port selection in background
    selection_thread = threading.Thread(target=delayed_port_selection, daemon=True)
    selection_thread.start()

@app.get("/telemetry", response_model=Telemetry)
def get_telemetry():
    """Get latest telemetry data from serial connection"""
    global latest_telemetry
    
    with telemetry_lock:
        if latest_telemetry is None:
            return generate_fallback_telemetry()
        return latest_telemetry

@app.get("/serial/ports", response_model=List[SerialPortInfo])
def list_serial_ports():
    """List all available serial ports with descriptions"""
    ports = []
    available_ports = serial.tools.list_ports.comports()
    
    for port in available_ports:
        is_current = False
        if ser and ser.is_open:
            is_current = (port.device == ser.port)
            
        ports.append(SerialPortInfo(
            port=port.device,
            description=port.description or "Unknown device",
            manufacturer=port.manufacturer,
            is_connected=is_current
        ))
    
    return ports

@app.post("/serial/connect")
def connect_serial(config: SerialConfig):
    """Connect to a serial port"""
    global ser, receiver_thread, is_connected
    
    print(f"\n{'='*60}")
    print(f"🔌 ATTEMPTING SERIAL CONNECTION")
    print(f"{'='*60}")
    print(f"📍 Port: {config.port}")
    print(f"⚡ Baud Rate: {config.baud_rate}")
    print(f"{'='*60}")
    
    # Disconnect from current port if connected
    if ser and ser.is_open:
        print(f"🔄 Disconnecting from current port: {ser.port}")
        disconnect_serial()
    
    try:
        with serial_lock:
            print(f"🔗 Opening serial connection...")
            ser = serial.Serial(config.port, config.baud_rate, timeout=1)
            is_connected = True
        
        # Start the receiver thread
        receiver_thread = threading.Thread(target=read_from_port, daemon=True)
        receiver_thread.start()
        
        print(f"✅ SERIAL CONNECTION SUCCESSFUL!")
        print(f"📡 Connected to: {config.port}")
        print(f"⚡ Baud rate: {config.baud_rate}")
        print(f"🎯 Status: ACTIVE - Listening for data...")
        print(f"{'='*60}\n")
        
        return {
            "message": f"Connected to {config.port}",
            "port": config.port,
            "baud_rate": config.baud_rate,
            "status": "connected"
        }
        
    except serial.SerialException as e:
        print(f"❌ SERIAL CONNECTION FAILED!")
        print(f"📍 Port: {config.port}")
        print(f"💥 Error: {e}")
        print(f"{'='*60}\n")
        is_connected = False
        raise HTTPException(status_code=400, detail=f"Could not open serial port {config.port}: {str(e)}")

@app.post("/serial/disconnect")
def disconnect_serial():
    """Disconnect from current serial port"""
    global ser, is_connected
    
    try:
        with serial_lock:
            if ser and ser.is_open:
                port_name = ser.port
                print(f"\n{'='*50}")
                print(f"🔌 DISCONNECTING FROM SERIAL PORT")
                print(f"{'='*50}")
                print(f"📍 Port: {port_name}")
                ser.close()
                ser = None
                is_connected = False
                print(f"✅ Successfully disconnected from {port_name}")
                print(f"{'='*50}\n")
                return {"message": f"Disconnected from {port_name}", "status": "disconnected"}
            else:
                print(f"ℹ️  No active serial connection to disconnect")
                return {"message": "No active connection", "status": "not_connected"}
    except Exception as e:
        print(f"❌ Error during disconnection: {e}")
        raise HTTPException(status_code=500, detail=f"Error disconnecting: {str(e)}")

@app.get("/serial/status")
def get_serial_status():
    """Get current serial connection status"""
    global ser, is_connected
    
    if ser and ser.is_open:
        return {
            "connected": is_connected,
            "port": ser.port,
            "baud_rate": ser.baudrate,
            "status": "connected"
        }
    else:
        return {
            "connected": False,
            "port": None,
            "baud_rate": None,
            "status": "disconnected"
        }

@app.post("/command", response_model=CommandResponse)
def send_command(command: Command):
    """Send command to connected serial device"""
    global ser
    
    if not ser or not ser.is_open:
        raise HTTPException(status_code=400, detail="No active serial connection")
    
    timestamp = command.timestamp if command.timestamp else time.time()
    
    try:
        with serial_lock:
            ser.write((command.command + '\n').encode('utf-8'))
        
        # Print command to server console
        print(f"\n{'='*50}")
        print(f"🚀 COMMAND SENT TO DEVICE")
        print(f"{'='*50}")
        print(f"⏰ Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))}")
        print(f"📝 Command: {command.command}")
        print(f"{'='*50}\n")
        
        return CommandResponse(
            message="Command sent successfully",
            command_received=command.command,
            timestamp=timestamp,
            status="success"
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error sending command: {str(e)}")

@app.get("/csv-status")
def get_csv_status():
    """Get CSV file status"""
    csv_exists = os.path.exists(CSV_FILENAME)
    rows = 0
    
    if csv_exists:
        try:
            with open(CSV_FILENAME, 'r') as f:
                rows = sum(1 for line in f) - 1  # Subtract header row
        except:
            rows = 0
    
    return {
        "csv_exists": csv_exists,
        "csv_path": CSV_FILENAME,
        "csv_rows": rows
    }

@app.get("/")
def read_root():
    """API status and information"""
    return {
        "message": "Serial Telemetry Parser API", 
        "version": "1.0",
        "data_source": "Serial Communication",
        "endpoints": {
            "GET /telemetry": "Get current telemetry data from serial",
            "GET /serial/ports": "List available serial ports",
            "POST /serial/connect": "Connect to a serial port",
            "POST /serial/disconnect": "Disconnect from current serial port",
            "GET /serial/status": "Get current serial connection status",
            "POST /command": "Send command to connected device",
            "GET /csv-status": "Get CSV logging status"
        }
    }
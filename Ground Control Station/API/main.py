''' Telemetry API with Random Data Generation
This serves random telemetry data for testing and development.
For real serial data from hardware, use parser.py instead.

Command to run:
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8001
'''
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import random
import time
import io
from typing import Optional, Dict, Any
import threading
import os

# Try to import pandas, but don't fail if it's not available
try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False
    print("⚠️  Warning: pandas not installed. CSV functionality will use fallback method.")

app = FastAPI(title="Telemetry API with Sensor Status")

# Print startup information
@app.on_event("startup")
async def startup_event():
    global csv_data, current_csv_path
    
    print(f"\n{'='*60}")
    print(f"🚀 TELEMETRY API SERVER STARTED")
    print(f"{'='*60}")
    print(f"📊 Current Data Mode: {data_mode.upper()}")
    
    # Try to load default CSV on startup
    default_csv_path = os.path.join(os.path.dirname(__file__), "Data/data_log.csv")
    
    if os.path.exists(default_csv_path) and csv_data is None:
        try:
            if PANDAS_AVAILABLE:
                csv_data = pd.read_csv(default_csv_path)
                current_csv_path = default_csv_path
                print(f"📁 Default CSV Auto-loaded: {default_csv_path}")
                print(f"📄 CSV Rows: {len(csv_data)}")
            else:
                raise ImportError("pandas not available")
        except ImportError:
            try:
                import csv
                with open(default_csv_path, 'r') as f:
                    csv_reader = csv.DictReader(f)
                    rows = list(csv_reader)
                csv_data = type('SimpleDataFrame', (), {
                    'iloc': type('Indexer', (), {
                        '__getitem__': lambda self, idx: type('Row', (), {
                            'to_dict': lambda: rows[idx]
                        })()
                    })(),
                    '__len__': lambda: len(rows)
                })()
                current_csv_path = default_csv_path
                print(f"📁 Default CSV Auto-loaded: {default_csv_path}")
                print(f"📄 CSV Rows: {len(csv_data)}")
            except Exception as e:
                print(f"⚠️  Could not auto-load default CSV: {str(e)}")
        except Exception as e:
            print(f"⚠️  Could not auto-load default CSV: {str(e)}")
    else:
        print(f"📁 CSV Data Loaded: {'Yes' if csv_data is not None else 'No'}")
        if csv_data is not None:
            print(f"📄 CSV Rows: {len(csv_data)}")
            print(f"📂 CSV Path: {current_csv_path}")
    
    print(f"🌐 Server running on port 8001")
    print(f"{'='*60}\n")

# Add CORS middleware for GUI integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify actual origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global variables for data mode and CSV data
data_mode = "csv"  # "random" or "csv"
csv_data = None
csv_index = 0
csv_start_time = None
csv_lock = threading.Lock()
current_csv_path = "API/telemetry_example_long.csv"  # Track current CSV file path

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

# You can also define a dedicated model for sensor statuses if desired:
class SensorStatus(BaseModel):
    gnss: str
    altimetry: str
    pressure: str
    temperature: str
    gyro: str
    power: str
    airQuality: str

# Configuration models
class DataModeConfig(BaseModel):
    mode:    str  # "random" or "csv"

class CSVUploadResponse(BaseModel):
    message: str
    rows_loaded: int

class CSVPathConfig(BaseModel):
    file_path: str

# Command models
class Command(BaseModel):
    command: str
    timestamp: Optional[float] = None
    source: Optional[str] = "GUI"

class CommandResponse(BaseModel):
    message: str
    command_received: str
    timestamp: float
    status: str

# The main telemetry model includes sensor status and a system state.
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

# CSV column mapping to our telemetry fields
CSV_COLUMN_MAPPING = {
    "TeamID": None,  # Not used in our model
    "Timestamp": "timestamp",
    "Packet": None,  # Not used
    "State": "system_state",
    "Altitude": "altitude",
    "Pressure": "pressure",
    "Temp": "temperature",
    "Voltage": "battery",
    "Lat": "gnss.lat",
    "Lon": "gnss.lng",
    "Sats": None,  # Not used
    "BNO_qi": "gyro.pitch",  # Map quaternion to Euler angles (simplified)
    "BNO_qj": "gyro.roll",
    "BNO_qk": "gyro.yaw",
    "BNO_qr": None,  # Not used
    "SHT_Temp": None,  # Use main temp
    "SHT_Hum": "humidity",
    "BME_Press": None,  # Use main pressure
    "BME_Temp": None,  # Use main temp
    "BME_Hum": None,  # Use main humidity
    "BME_Gas": "airQuality",
    "BME_IAQ": None,  # Not used
    "BMP_Press": None,  # Use main pressure
    "BMP_Temp": None,  # Use main temp
    "ADS_V": None  # Not used
}

def parse_csv_row_to_telemetry(row: Dict[str, Any]) -> Telemetry:
    """Convert CSV row to Telemetry object"""
    try:
        # Handle timestamp - if it's already a Unix timestamp, use it; otherwise convert
        timestamp_val = float(row.get("Timestamp", time.time()))
        
        # Extract basic values
        altitude = float(row.get("Altitude", 0))
        temperature = float(row.get("Temp", 25))
        pressure = float(row.get("Pressure", 1013))
        humidity = float(row.get("SHT_Hum", 50))
        battery = float(row.get("Voltage", 12)) * 8.33  # Convert voltage to percentage (assuming 12V max)
        airQuality = float(row.get("BME_Gas", 10))
        
        # GNSS data
        gnss = GNSS(
            lat=float(row.get("Lat", 28.6139)),
            lng=float(row.get("Lon", 77.209))
        )
        
        # Gyro data (simplified quaternion to Euler conversion)
        gyro = Gyro(
            pitch=float(row.get("BNO_qi", 0)) * 180,  # Convert normalized values to degrees
            roll=float(row.get("BNO_qj", 0)) * 180,
            yaw=float(row.get("BNO_qk", 0)) * 180
        )
        
        # Acceleration (generate random if not available)
        acceleration = Acceleration(
            x=random.uniform(-2, 2),
            y=random.uniform(-2, 2),
            z=random.uniform(-2, 2)
        )
        
        # Calculate sensor statuses
        sensor_status = SensorStatus(
            gnss="ACTIVE" if gnss.lat != 0 and gnss.lng != 0 else "ERROR",
            altimetry="ERROR" if altitude < 50 else "ACTIVE",
            pressure="ERROR" if pressure < 1000 or pressure > 1020 else "ACTIVE",
            temperature="ERROR" if temperature < 0 or temperature > 40 else "ACTIVE",
            gyro="ACTIVE",
            power="ERROR" if battery < 20 else "ACTIVE",
            airQuality="ERROR" if airQuality > 35 else "ACTIVE"
        )
        
        system_state = int(row.get("State", 0))
        
        return Telemetry(
            timestamp=timestamp_val,
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
    except Exception as e:
        # If parsing fails, return default telemetry
        return generate_random_telemetry()

def generate_random_telemetry() -> Telemetry:
    """Generate random telemetry data (original functionality)"""
    timestamp = time.time()
    altitude = 100 - random.uniform(0, 10)
    temperature = 25 + random.uniform(-1, 1)
    pressure = 1013 + random.uniform(-5, 5)
    humidity = 50 + random.uniform(-3, 3)
    battery = 100 - random.uniform(0, 1)
    airQuality = 10 + random.uniform(-2, 2)
    
    gnss = GNSS(
        lat=28.6139 + random.uniform(-0.0001, 0.0001),
        lng=77.209 + random.uniform(-0.0001, 0.0001)
    )
    
    gyro = Gyro(
        pitch=random.uniform(0, 360),
        yaw=random.uniform(0, 360),
        roll=random.uniform(0, 360)
    )
    
    acceleration = Acceleration(
        x=random.uniform(-2, 2),
        y=random.uniform(-2, 2),
        z=random.uniform(-2, 2)
    )
    
    sensor_status = SensorStatus(
        gnss="ACTIVE",
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
        system_state=0
    )

def get_csv_telemetry() -> Telemetry:
    """Get telemetry data from CSV file"""
    global csv_data, csv_index, csv_start_time
    
    with csv_lock:
        if csv_data is None or len(csv_data) == 0:
            return generate_random_telemetry()
        
        if csv_start_time is None:
            csv_start_time = time.time()
            csv_index = 0
        
        # Calculate how many seconds have passed since we started
        elapsed_time = time.time() - csv_start_time
        target_index = int(elapsed_time)  # 1 second intervals
        
        # Use modulo to loop through the data
        current_index = target_index % len(csv_data)
        
        if current_index != csv_index:
            csv_index = current_index
        
        # Get the current row and parse it
        current_row = csv_data.iloc[csv_index].to_dict()
        return parse_csv_row_to_telemetry(current_row)

@app.get("/telemetry", response_model=Telemetry)
def get_telemetry():
    """Get telemetry data based on current mode (random or CSV)"""
    global data_mode
    
    if data_mode == "csv":
        return get_csv_telemetry()
    else:
        return generate_random_telemetry()

@app.post("/config/data-mode")
def set_data_mode(config: DataModeConfig):
    """Set the data mode to 'random' or 'csv'"""
    global data_mode, csv_start_time
    
    if config.mode not in ["random", "csv"]:
        raise HTTPException(status_code=400, detail="Mode must be 'random' or 'csv'")
    
    data_mode = config.mode
    
    # Reset CSV timing when switching modes
    if data_mode == "csv":
        csv_start_time = None
    
    return {"message": f"Data mode set to {data_mode}", "mode": data_mode}

@app.get("/config/data-mode")
def get_data_mode():
    """Get current data mode"""
    global data_mode, csv_data, current_csv_path
    csv_loaded = csv_data is not None and len(csv_data) > 0
    return {
        "mode": data_mode, 
        "csv_loaded": csv_loaded,
        "csv_rows": len(csv_data) if csv_loaded else 0,
        "current_csv_path": current_csv_path
    }

@app.post("/upload-csv", response_model=CSVUploadResponse)
async def upload_csv(file: UploadFile = File(...)):
    """Upload CSV file with telemetry data"""
    global csv_data, csv_start_time
    
    if not file.filename.endswith('.csv'):
        raise HTTPException(status_code=400, detail="File must be a CSV")
    
    try:
        # Read the uploaded file
        contents = await file.read()
        
        # Try to parse with pandas
        try:
            if PANDAS_AVAILABLE:
                csv_data = pd.read_csv(io.StringIO(contents.decode('utf-8')))
            else:
                raise ImportError("pandas not available")
        except ImportError:
            # Fallback to basic CSV parsing if pandas not available
            import csv
            csv_reader = csv.DictReader(io.StringIO(contents.decode('utf-8')))
            rows = list(csv_reader)
            # Create a simple DataFrame-like structure
            csv_data = type('SimpleDataFrame', (), {
                'iloc': type('Indexer', (), {
                    '__getitem__': lambda self, idx: type('Row', (), {
                        'to_dict': lambda: rows[idx]
                    })()
                })(),
                '__len__': lambda: len(rows)
            })()
        
        # Reset timing
        csv_start_time = None
        
        return CSVUploadResponse(
            message="CSV file uploaded successfully", 
            rows_loaded=len(csv_data)
        )
        
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Error processing CSV: {str(e)}")

@app.post("/load-csv-file")
def load_csv_file(config: CSVPathConfig):
    """Load CSV file from specified path"""
    global csv_data, csv_start_time, current_csv_path
    
    file_path = config.file_path
    
    # Print the path being loaded
    print(f"\n📁 Loading CSV file from: {file_path}")
    
    if not os.path.exists(file_path):
        print(f"❌ File not found: {file_path}")
        raise HTTPException(status_code=404, detail=f"File not found: {file_path}")
    
    if not file_path.endswith('.csv'):
        print(f"❌ Invalid file type: {file_path}")
        raise HTTPException(status_code=400, detail="File must be a CSV")
    
    try:
        # Try to parse with pandas
        try:
            if PANDAS_AVAILABLE:
                csv_data = pd.read_csv(file_path)
            else:
                raise ImportError("pandas not available")
        except ImportError:
            # Fallback to basic CSV parsing
            import csv
            with open(file_path, 'r') as f:
                csv_reader = csv.DictReader(f)
                rows = list(csv_reader)
            # Create a simple DataFrame-like structure
            csv_data = type('SimpleDataFrame', (), {
                'iloc': type('Indexer', (), {
                    '__getitem__': lambda self, idx: type('Row', (), {
                        'to_dict': lambda: rows[idx]
                    })()
                })(),
                '__len__': lambda: len(rows)
            })()
        
        # Reset timing and update current path
        csv_start_time = None
        current_csv_path = file_path
        
        return {
            "message": "CSV file loaded successfully", 
            "rows_loaded": len(csv_data),
            "file_path": file_path
        }
        
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Error loading CSV: {str(e)}")

@app.post("/load-default-csv")
def load_default_csv():
    """Load the default telemetry CSV file"""
    global csv_data, csv_start_time, current_csv_path
    
    # Path to the default CSV file
    default_csv_path = os.path.join(os.path.dirname(__file__), "telemetry_example_long.csv")
    
    print(f"\n📁 Loading default CSV file from: {default_csv_path}")
    
    if not os.path.exists(default_csv_path):
        print(f"❌ Default CSV file not found: {default_csv_path}")
        raise HTTPException(status_code=404, detail=f"Default CSV file not found: {default_csv_path}")
    
    try:
        # Try to parse with pandas
        try:
            if PANDAS_AVAILABLE:
                csv_data = pd.read_csv(default_csv_path)
            else:
                raise ImportError("pandas not available")
        except ImportError:
            # Fallback to basic CSV parsing
            import csv
            with open(default_csv_path, 'r') as f:
                csv_reader = csv.DictReader(f)
                rows = list(csv_reader)
            # Create a simple DataFrame-like structure
            csv_data = type('SimpleDataFrame', (), {
                'iloc': type('Indexer', (), {
                    '__getitem__': lambda self, idx: type('Row', (), {
                        'to_dict': lambda: rows[idx]
                    })()
                })(),
                '__len__': lambda: len(rows)
            })()
        
        # Reset timing and update current path
        csv_start_time = None
        current_csv_path = default_csv_path
        
        print(f"✅ Default CSV loaded successfully with {len(csv_data)} rows")
        
        return {
            "message": "Default CSV file loaded successfully", 
            "rows_loaded": len(csv_data),
            "file_path": default_csv_path
        }
        
    except Exception as e:
        print(f"❌ Error loading default CSV: {str(e)}")
        raise HTTPException(status_code=400, detail=f"Error loading default CSV: {str(e)}")

@app.get("/csv-status")
def get_csv_status():
    """Get current CSV loading status and path"""
    global csv_data, current_csv_path
    
    csv_loaded = csv_data is not None and len(csv_data) > 0
    
    return {
        "csv_loaded": csv_loaded,
        "csv_rows": len(csv_data) if csv_loaded else 0,
        "current_csv_path": current_csv_path,
        "default_csv_available": os.path.exists(os.path.join(os.path.dirname(__file__), "telemetry_example_long.csv"))
    }

@app.post("/command", response_model=CommandResponse)
def receive_command(command: Command):
    """Receive command from GUI and print to server console"""
    timestamp = command.timestamp if command.timestamp else time.time()
    
    # Print command to server console with formatting
    print(f"\n{'='*50}")
    print(f"🚀 COMMAND RECEIVED FROM {command.source}")
    print(f"{'='*50}")
    print(f"⏰ Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))}")
    print(f"📝 Command: {command.command}")
    print(f"{'='*50}\n")
    
    return CommandResponse(
        message="Command received and logged",
        command_received=command.command,
        timestamp=timestamp,
        status="success"
    )

@app.get("/")
def read_root():
    """API status and information"""
    return {
        "message": "Telemetry API with CSV and Command support", 
        "version": "2.2",
        "endpoints": {
            "GET /telemetry": "Get current telemetry data",
            "GET /config/data-mode": "Get current data mode and CSV status",
            "POST /config/data-mode": "Set data mode (random/csv)",
            "POST /upload-csv": "Upload CSV file",
            "POST /load-csv-file": "Load CSV from custom file path",
            "POST /load-default-csv": "Load default telemetry CSV file",
            "GET /csv-status": "Get CSV loading status and current path",
            "POST /command": "Send command to server"
        }
    }

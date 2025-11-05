#!/usr/bin/env python3
"""
Simple ESP32 Format Test
Tests ESP32 data parsing without requiring FastAPI dependencies
"""

import time
import random

# Simulate the telemetry parsing logic without FastAPI dependencies
def simple_parse_test(data_string):
    """Simple version of the parsing logic for testing"""
    print(f"<<< Testing: {data_string}")
    
    # Default values
    timestamp = time.time()
    altitude = 0.0
    temperature = 25.0
    pressure = 1013.25
    battery = 100.0
    lat = 28.6139
    lng = 77.209
    system_state = 0
    packet_count = 0
    
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
    parsed_data = {}
    
    for pair in pairs:
        if ':' in pair:
            key, value = pair.split(':', 1)
            try:
                if key == 'Packet':
                    packet_count = int(value)
                    parsed_data[key] = packet_count
                elif key == 'Altitude' or key == 'Alt':
                    altitude = float(value)
                    parsed_data[key] = altitude
                elif key == 'Temp' or key == 'Temperature':
                    temperature = float(value)
                    parsed_data[key] = temperature
                elif key == 'Pressure' or key == 'Press':
                    pressure = float(value)
                    parsed_data[key] = pressure
                elif key == 'Voltage':
                    voltage = float(value)
                    # Convert voltage to battery percentage
                    if voltage <= 5.0:  # 3.3V system
                        battery = max(0, min(100, ((voltage - 2.5) / (4.2 - 2.5)) * 100))
                    else:  # Higher voltage system
                        battery = max(0, min(100, (voltage / 12.0) * 100))
                    parsed_data[key] = f"{voltage}V -> {battery:.1f}%"
                elif key == 'Lat':
                    lat = float(value)
                    parsed_data[key] = lat
                elif key == 'Lon':
                    lng = float(value)
                    parsed_data[key] = lng
                elif key == 'State':
                    system_state = int(value)
                    parsed_data[key] = system_state
                elif key == 'TS':
                    # Use device timestamp if provided
                    device_ts = float(value)
                    parsed_data[key] = f"{device_ts} (millis)"
                else:
                    parsed_data[key] = value
            except (ValueError, TypeError) as e:
                print(f"   ⚠️  Could not parse {key}={value}: {e}")
                continue
    
    return parsed_data

def test_esp32_formats():
    """Test different ESP32 output formats"""
    
    print("🧪 ESP32 Format Compatibility Test")
    print("=" * 60)
    
    # Test formats that your ESP32 will produce
    test_cases = [
        # Updated ESP32 format (what your code will send)
        "PING:Packet:123,TS:54321,Alt:150.5,Temp:25.3,Pressure:1015.2,Lat:28.6140,Lon:77.2095,Voltage:3.3,State:1",
        
        # Ground station received format (LoRa)
        "Received <- 'PING:Packet:124,TS:54400,Alt:148.2,Temp:24.8,Pressure:1014.8,Lat:28.6141,Lon:77.2096,Voltage:3.2,State:2' with RSSI -45",
        
        # Partial data (some sensors might fail)
        "PING:Packet:125,TS:54500,Alt:152.1,Temp:26.0,Voltage:3.1",
        
        # Original simple format
        "PING:126,TS:54600"
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n📋 Test Case {i}:")
        print(f"Input: {test_case}")
        print("-" * 40)
        
        try:
            parsed_data = simple_parse_test(test_case)
            
            if parsed_data:
                print("✅ Parsed successfully!")
                for key, value in parsed_data.items():
                    print(f"   {key}: {value}")
            else:
                print("⚠️  No data parsed (but no errors)")
                
        except Exception as e:
            print(f"❌ Parsing failed: {e}")

def test_port_discovery():
    """Test port discovery"""
    print(f"\n🔍 Port Discovery Test")
    print("=" * 30)
    
    try:
        import serial.tools.list_ports
        
        ports = serial.tools.list_ports.comports()
        
        if ports:
            print(f"✅ Found {len(ports)} serial ports:")
            for i, port in enumerate(ports, 1):
                device_type = "Unknown"
                desc = port.description.lower() if port.description else ""
                mfg = port.manufacturer.lower() if port.manufacturer else ""
                
                if "arduino" in desc or "arduino" in mfg:
                    device_type = "🔧 Arduino"
                elif "esp32" in desc or "silicon labs" in mfg or "cp210" in desc:
                    device_type = "📡 ESP32/ESP8266"
                elif "ftdi" in mfg:
                    device_type = "🔌 FTDI Device"
                elif "usb" in desc:
                    device_type = "🔗 USB Device"
                
                print(f"   {i}. {port.device}")
                print(f"      Description: {port.description}")
                print(f"      Likely Type: {device_type}")
                print()
        else:
            print("⚠️  No serial ports found")
            
    except ImportError:
        print("❌ pyserial not installed. Run: pip install pyserial")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    test_esp32_formats()
    test_port_discovery()
    
    print(f"\n🚀 NEXT STEPS:")
    print(f"=" * 40)
    print("1. Upload updated ESP_GCS.ino to your ESP32")
    print("2. Connect ESP32 to your computer via USB")
    print("3. Run: python find_ports.py")
    print("4. Start parser API: ./start.sh (option 2)")
    print("5. Connect to your ESP32 port via API")
    print("6. Watch live telemetry data!")
    
    print(f"\n📡 ESP32 Connection Tips:")
    print("- Look for ports with 'Silicon Labs' or 'CP210x' in description")
    print("- ESP32 usually appears as /dev/cu.usbserial-* on macOS") 
    print("- Try different baud rates: 115200, 9600, 57600")
    print("- Make sure ESP32 is in the correct mode (CANSAT_MODE for data)")
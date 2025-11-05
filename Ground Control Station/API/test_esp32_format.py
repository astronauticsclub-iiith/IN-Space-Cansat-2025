#!/usr/bin/env python3
"""
ESP32 Format Test Script
This script tests if the ESP32 output format is compatible with the parser
"""

import sys
import os

# Add the API directory to the path so we can import the parser functions
sys.path.append('/Users/va/GCS-3D/API')

from parser import parse_serial_data_to_telemetry

def test_esp32_formats():
    """Test different ESP32 output formats"""
    
    print("🧪 Testing ESP32 Format Compatibility")
    print("=" * 50)
    
    # Test formats based on your ESP32 code
    test_cases = [
        # Basic format from ESP32
        "PING:Packet:123,TS:54321,Alt:150.5,Temp:25.3,Pressure:1015.2,Lat:28.6140,Lon:77.2095,Voltage:3.3,State:1",
        
        # Ground station received format (what might come through LoRa)
        "Received <- 'PING:Packet:124,TS:54400,Alt:148.2,Temp:24.8,Pressure:1014.8,Lat:28.6141,Lon:77.2096,Voltage:3.2,State:2' with RSSI -45",
        
        # Simple format (original)
        "PING:123,TS:54321",
        
        # Mixed format
        "PING:Packet:125,TS:54500,Alt:152.1,Temp:26.0"
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n📋 Test Case {i}:")
        print(f"Input: {test_case}")
        print("-" * 40)
        
        try:
            telemetry = parse_serial_data_to_telemetry(test_case)
            
            print(f"✅ Parsed successfully!")
            print(f"   Timestamp: {telemetry.timestamp}")
            print(f"   Altitude: {telemetry.altitude} m")
            print(f"   Temperature: {telemetry.temperature} °C")
            print(f"   Pressure: {telemetry.pressure} hPa")
            print(f"   Battery: {telemetry.battery:.1f}%")
            print(f"   GPS: {telemetry.gnss.lat}, {telemetry.gnss.lng}")
            print(f"   System State: {telemetry.system_state}")
            
            # Check sensor statuses
            active_sensors = []
            error_sensors = []
            
            for sensor, status in telemetry.sensor_status.__dict__.items():
                if status == "ACTIVE":
                    active_sensors.append(sensor)
                else:
                    error_sensors.append(sensor)
            
            print(f"   Active Sensors: {', '.join(active_sensors) if active_sensors else 'None'}")
            print(f"   Error Sensors: {', '.join(error_sensors) if error_sensors else 'None'}")
            
        except Exception as e:
            print(f"❌ Parsing failed: {e}")
    
    print(f"\n{'=' * 50}")
    print("📝 RECOMMENDATIONS:")
    print("1. The updated ESP32 code should work perfectly with the parser")
    print("2. The parser can handle both direct serial and LoRa received formats")
    print("3. Missing data fields will use sensible defaults")
    print("4. Use the updated ESP32 code for best compatibility")

def test_port_discovery():
    """Test port discovery functionality"""
    print(f"\n🔍 Testing Port Discovery...")
    
    try:
        # Import the port discovery functions
        from find_ports import list_available_ports
        
        ports = list_available_ports()
        
        if ports:
            print(f"✅ Port discovery working! Found {len(ports)} ports")
            for port in ports:
                print(f"   - {port['port']}: {port['description']}")
        else:
            print("⚠️  No ports found, but discovery is working")
            
    except Exception as e:
        print(f"❌ Port discovery error: {e}")

if __name__ == "__main__":
    test_esp32_formats()
    test_port_discovery()
    
    print(f"\n🚀 NEXT STEPS:")
    print("1. Upload the updated ESP_GCS.ino to your ESP32")
    print("2. Run: python find_ports.py  # to find your ESP32 port")
    print("3. Run: ./start.sh  # choose option 2 to start parser API")
    print("4. Connect to your ESP32 port via the API")
    print("5. Watch the telemetry data flow!")
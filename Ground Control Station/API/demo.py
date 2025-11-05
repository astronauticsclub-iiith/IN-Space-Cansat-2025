#!/usr/bin/env python3
"""
API Demonstration Script
Shows the difference between main.py (random data) and parser.py (serial data)
"""

import requests
import json
import time

def test_api(port, name):
    """Test an API endpoint"""
    try:
        print(f"\n{'='*60}")
        print(f"🧪 Testing {name} API (Port {port})")
        print(f"{'='*60}")
        
        # Test root endpoint
        response = requests.get(f"http://localhost:{port}/")
        print(f"📋 API Info:")
        print(json.dumps(response.json(), indent=2))
        
        # Test telemetry endpoint
        response = requests.get(f"http://localhost:{port}/telemetry")
        if response.status_code == 200:
            data = response.json()
            print(f"\n📊 Sample Telemetry Data:")
            print(f"   Timestamp: {data['timestamp']}")
            print(f"   Altitude: {data['altitude']}")
            print(f"   Temperature: {data['temperature']}")
            print(f"   GNSS: {data['gnss']['lat']}, {data['gnss']['lng']}")
            print(f"   Battery: {data['battery']}%")
            print(f"   System State: {data['system_state']}")
        else:
            print(f"❌ Telemetry endpoint failed: {response.status_code}")
            
        # Test serial-specific endpoints for parser API
        if port == 8002:
            print(f"\n🔌 Serial-specific endpoints:")
            
            # List ports
            response = requests.get(f"http://localhost:{port}/serial/ports")
            if response.status_code == 200:
                ports = response.json()
                print(f"   Available ports: {len(ports)}")
                for port_info in ports[:3]:  # Show first 3 ports
                    print(f"     - {port_info['port']}: {port_info['description']}")
            
            # Check connection status
            response = requests.get(f"http://localhost:{port}/serial/status")
            if response.status_code == 200:
                status = response.json()
                print(f"   Connection status: {status['status']}")
        
        print(f"✅ {name} API is working!")
        
    except requests.exceptions.ConnectionError:
        print(f"❌ {name} API (Port {port}) is not running")
        print(f"   Start it with: python -m uvicorn {'main' if port == 8001 else 'parser'}:app --reload --host 0.0.0.0 --port {port}")
    except Exception as e:
        print(f"❌ Error testing {name} API: {e}")

def main():
    print("🚀 API Demonstration Script")
    print("This script shows the difference between the two telemetry APIs")
    
    # Test both APIs
    test_api(8001, "Main (Random Data)")
    test_api(8002, "Parser (Serial Data)")
    
    print(f"\n{'='*60}")
    print("📝 SUMMARY")
    print(f"{'='*60}")
    print("🎲 Main API (Port 8001):")
    print("   - Serves random/simulated telemetry data")
    print("   - Good for testing and development")
    print("   - Can also load CSV files for playback")
    print()
    print("📡 Parser API (Port 8002):")
    print("   - Reads real data from serial ports")
    print("   - Connect to hardware devices")
    print("   - Logs data to CSV files")
    print("   - Port discovery and management")
    print()
    print("🔧 To get started:")
    print("   1. Find your device port: python find_ports.py")
    print("   2. Start parser API: python -m uvicorn parser:app --reload --host 0.0.0.0 --port 8002")
    print("   3. Connect to device via API or web interface")

if __name__ == "__main__":
    main()
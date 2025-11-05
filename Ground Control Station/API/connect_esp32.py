#!/usr/bin/env python3
"""
Quick ESP32 Connection Helper
This script helps you quickly connect to your ESP32 via the API
"""

import requests
import json
import time

def get_available_ports():
    """Get available ports from the API"""
    try:
        response = requests.get("http://localhost:8002/serial/ports")
        if response.status_code == 200:
            return response.json()
        else:
            print(f"❌ Could not get ports: {response.status_code}")
            return []
    except requests.exceptions.ConnectionError:
        print("❌ Parser API not running. Start it with: ./start.sh (option 2)")
        return []
    except Exception as e:
        print(f"❌ Error: {e}")
        return []

def connect_to_port(port, baud_rate=115200):
    """Connect to a specific port via API"""
    try:
        data = {
            "port": port,
            "baud_rate": baud_rate
        }
        
        print(f"🔌 Attempting to connect to {port}...")
        response = requests.post("http://localhost:8002/serial/connect", 
                                json=data)
        
        if response.status_code == 200:
            result = response.json()
            print(f"✅ Connected successfully!")
            print(f"📍 Port: {result['port']}")
            print(f"⚡ Baud Rate: {result['baud_rate']}")
            return True
        else:
            print(f"❌ Connection failed: {response.status_code}")
            print(f"Error: {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ Connection error: {e}")
        return False

def check_connection_status():
    """Check current connection status"""
    try:
        response = requests.get("http://localhost:8002/serial/status")
        if response.status_code == 200:
            status = response.json()
            if status['connected']:
                print(f"✅ Currently connected to: {status['port']}")
                print(f"⚡ Baud rate: {status['baud_rate']}")
                return True
            else:
                print("📡 No active connection")
                return False
        return False
    except:
        return False

def test_telemetry():
    """Test getting telemetry data"""
    try:
        response = requests.get("http://localhost:8002/telemetry")
        if response.status_code == 200:
            data = response.json()
            print(f"\n📊 Latest Telemetry:")
            print(f"   Altitude: {data['altitude']} m")
            print(f"   Temperature: {data['temperature']} °C")
            print(f"   Battery: {data['battery']:.1f}%")
            print(f"   GPS: {data['gnss']['lat']}, {data['gnss']['lng']}")
            
            # Check if we're getting real data vs fallback
            if data['altitude'] == 0 and data['temperature'] == 0:
                print("⚠️  Receiving fallback data - check ESP32 connection")
            else:
                print("✅ Receiving live data from ESP32!")
        else:
            print(f"❌ Could not get telemetry: {response.status_code}")
    except Exception as e:
        print(f"❌ Telemetry error: {e}")

def main():
    print("🚀 ESP32 Quick Connection Helper")
    print("=" * 40)
    
    # Check if API is running
    print("🔍 Checking API status...")
    if not check_connection_status():
        print("📡 API is running but no connection active")
    
    # Get available ports
    print("\n🔍 Scanning for serial ports...")
    ports = get_available_ports()
    
    if not ports:
        print("❌ No ports found or API not running")
        return
    
    # Show ESP32-like ports
    esp32_ports = []
    print(f"\n📡 Available Serial Ports:")
    for i, port in enumerate(ports, 1):
        is_esp32 = False
        desc = port['description'].lower()
        
        if any(keyword in desc for keyword in ['cp210', 'cp2102', 'silicon labs', 'esp32']):
            is_esp32 = True
            esp32_ports.append(port)
            print(f"   {i}. {port['port']} ⭐ (ESP32-like)")
        else:
            print(f"   {i}. {port['port']}")
        
        print(f"      {port['description']}")
        if port['is_connected']:
            print(f"      🔗 Currently connected")
    
    # Auto-connect to likely ESP32 port
    if esp32_ports:
        target_port = esp32_ports[0]['port']
        print(f"\n🎯 Found likely ESP32 port: {target_port}")
        
        connect_choice = input("Connect to this port? (y/n): ").strip().lower()
        if connect_choice == 'y':
            if connect_to_port(target_port):
                print(f"\n⏳ Waiting for data from ESP32...")
                time.sleep(2)  # Give it time to receive data
                test_telemetry()
    else:
        print(f"\n⚠️  No obvious ESP32 ports found")
        print("Try connecting your ESP32 and running this script again")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n👋 Interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
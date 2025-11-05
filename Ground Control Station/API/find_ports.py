#!/usr/bin/env python3
"""
COM Port Discovery Tool
This script helps you find and identify the correct serial port for your device.
"""

import serial.tools.list_ports
import sys
import time

def list_available_ports():
    """List all available serial ports with detailed information"""
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("❌ No serial ports found!")
        return []
    
    print(f"\n{'='*80}")
    print(f"📋 AVAILABLE SERIAL PORTS")
    print(f"{'='*80}")
    
    port_info = []
    for i, port in enumerate(ports, 1):
        print(f"\n{i}. Port: {port.device}")
        print(f"   Description: {port.description}")
        print(f"   Manufacturer: {port.manufacturer or 'Unknown'}")
        print(f"   Hardware ID: {port.hwid}")
        
        # Try to identify common device types
        device_type = "Unknown"
        if "Arduino" in str(port.description) or "Arduino" in str(port.manufacturer):
            device_type = "🔧 Arduino"
        elif "ESP32" in str(port.description) or "Silicon Labs" in str(port.manufacturer):
            device_type = "📡 ESP32/ESP8266"
        elif "FTDI" in str(port.manufacturer):
            device_type = "🔌 FTDI Device"
        elif "USB" in str(port.description):
            device_type = "🔗 USB Device"
        
        print(f"   Likely Device: {device_type}")
        
        port_info.append({
            'number': i,
            'port': port.device,
            'description': port.description,
            'manufacturer': port.manufacturer,
            'type': device_type
        })
    
    print(f"\n{'='*80}")
    return port_info

def test_port_connection(port_name, baud_rate=115200):
    """Test connection to a specific port"""
    print(f"\n🔍 Testing connection to {port_name} at {baud_rate} baud...")
    
    try:
        # Try to open the port
        with serial.Serial(port_name, baud_rate, timeout=2) as ser:
            print(f"✅ Successfully opened {port_name}")
            print(f"📡 Listening for data for 5 seconds...")
            
            start_time = time.time()
            data_received = False
            
            while time.time() - start_time < 5:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8').strip()
                        if line:
                            print(f"📥 Received: {line}")
                            data_received = True
                    except UnicodeDecodeError:
                        print(f"📥 Received binary data (not text)")
                        data_received = True
                
                time.sleep(0.1)
            
            if data_received:
                print(f"✅ Port {port_name} is receiving data!")
                return True
            else:
                print(f"⚠️  Port {port_name} opened but no data received")
                return False
                
    except serial.SerialException as e:
        print(f"❌ Could not open {port_name}: {e}")
        return False
    except Exception as e:
        print(f"❌ Error testing {port_name}: {e}")
        return False

def interactive_port_selection():
    """Interactive port selection and testing"""
    while True:
        ports = list_available_ports()
        
        if not ports:
            input("\nPress Enter to refresh port list...")
            continue
        
        print(f"\n📌 OPTIONS:")
        print(f"   r - Refresh port list")
        print(f"   q - Quit")
        print(f"   1-{len(ports)} - Test specific port")
        print(f"   all - Test all ports")
        
        choice = input(f"\nSelect option: ").strip().lower()
        
        if choice == 'q':
            print("👋 Goodbye!")
            break
        elif choice == 'r':
            continue
        elif choice == 'all':
            print(f"\n🔍 Testing all ports...")
            for port in ports:
                test_port_connection(port['port'])
                print()
        else:
            try:
                port_num = int(choice)
                if 1 <= port_num <= len(ports):
                    selected_port = ports[port_num - 1]
                    print(f"\n📍 Selected: {selected_port['port']} ({selected_port['description']})")
                    
                    if test_port_connection(selected_port['port']):
                        print(f"\n🎯 RECOMMENDED PORT: {selected_port['port']}")
                        
                        # Ask if user wants to use this port
                        use_port = input(f"\nUse this port in parser.py? (y/n): ").strip().lower()
                        if use_port == 'y':
                            print(f"\n📝 To use this port:")
                            print(f"   1. Update parser.py or use the API endpoint:")
                            print(f"   2. POST to /serial/connect with port: '{selected_port['port']}'")
                            print(f"   3. Or run: python -m uvicorn parser:app --reload --host 0.0.0.0 --port 8002")
                            break
                else:
                    print(f"❌ Invalid selection. Please choose 1-{len(ports)}")
            except ValueError:
                print(f"❌ Invalid input. Please enter a number, 'r', 'all', or 'q'")

def main():
    print(f"🔍 COM Port Discovery Tool")
    print(f"This tool helps you find the correct serial port for your device.")
    
    try:
        interactive_port_selection()
    except KeyboardInterrupt:
        print(f"\n\n👋 Interrupted by user. Goodbye!")
    except Exception as e:
        print(f"\n❌ An error occurred: {e}")

if __name__ == "__main__":
    main()
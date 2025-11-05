#!/bin/bash

# Telemetry API Startup Script

echo "🚀 Telemetry API Startup Script"
echo "==============================="
echo
echo "Choose which API to start:"
echo "1. Main API (Random Data) - Port 8001"
echo "2. Parser API (Serial Data) - Port 8002"
echo "3. Find Serial Ports"
echo "4. Quick ESP32 Connection Helper"
echo "5. Test ESP32 Format Compatibility"
echo "6. Demo/Test APIs"
echo "7. Install Dependencies"
echo "q. Quit"
echo

read -p "Select option (1-7, q): " choice

case $choice in
    1)
        echo "🎲 Starting Main API with random data..."
        echo "Access at: http://localhost:8001"
        echo "Press Ctrl+C to stop"
        echo
        python -m uvicorn main:app --reload --host 0.0.0.0 --port 8001
        ;;
    2)
        echo "📡 Starting Parser API for serial data..."
        echo "Access at: http://localhost:8002"
        echo "Press Ctrl+C to stop"
        echo
        python -m uvicorn parser:app --reload --host 0.0.0.0 --port 8002
        ;;
    3)
        echo "🔍 Running port discovery tool..."
        echo
        python find_ports.py
        ;;
    4)
        echo "🚀 Quick ESP32 Connection Helper..."
        echo
        python connect_esp32.py
        ;;
    5)
        echo "🧪 Testing ESP32 format compatibility..."
        echo
        python simple_test.py
        ;;
    6)
        echo "🧪 Running API demo..."
        echo
        python demo.py
        ;;
    7)
        echo "📦 Installing dependencies..."
        echo
        pip install -r requirements.txt
        echo "✅ Dependencies installed!"
        ;;
    q|Q)
        echo "👋 Goodbye!"
        exit 0
        ;;
    *)
        echo "❌ Invalid option. Please choose 1-7 or q."
        exit 1
        ;;
esac
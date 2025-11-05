import serial
import threading
import csv
import time
import os

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/tty.usbmodem11301' # Change this to your ESP32's COM port
# For Windows, it will be like 'COM3' or 'COM4'
# For macOS or Linux, it will be like '/dev/tty.usbserial-...' or '/dev/tty.usbmodem...'
BAUD_RATE = 115200
CSV_FILENAME = 'cansat_telemetry.csv'
# --------------------

# The header for your CSV file as requested
CSV_HEADER = [
    "TeamID", "Timestamp", "Packet", "State", "Altitude", "Pressure", "Temp", 
    "Voltage", "Lat", "Lon", "Sats", "BNO_qi", "BNO_qj", "BNO_qk", "BNO_qr", 
    "SHT_Temp", "SHT_Hum", "BME_Press", "BME_Temp", "BME_Hum", "BME_Gas", 
    "BME_IAQ", "BMP_Press", "BMP_Temp", "ADS_V"
]

# --- Main Application ---

# Global variable for the serial connection
ser = None

def setup_csv():
    """Creates the CSV file and writes the header if it doesn't exist."""
    if not os.path.exists(CSV_FILENAME):
        with open(CSV_FILENAME, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(CSV_HEADER)
        print(f"Created CSV file: {CSV_FILENAME}")

def parse_and_save_data(data_string):
    """Parses telemetry data and saves it to the CSV file."""
    print(f"<<< Received Telemetry: {data_string}")
    
    # Create a dictionary with default values for all header fields
    data_row = {key: 'N/A' for key in CSV_HEADER}
    
    # Set default values for data we know we will always have
    data_row['TeamID'] = '2025' # Example Team ID
    data_row['Timestamp'] = time.strftime('%Y-%m-%d %H:%M:%S')

    try:
        # Expected format: "PING:Packet:123,TS:54321,Alt:300.5,..."
        # Remove the "PING:" prefix
        if data_string.startswith("PING:"):
            data_string = data_string[5:]
        
        # Split the data into key-value pairs
        pairs = data_string.split(',')
        for pair in pairs:
            if ':' in pair:
                key, value = pair.split(':', 1)
                # Map received keys to our CSV header keys
                # This makes the parser flexible. Add new mappings here.
                key_map = {
                    'Packet': 'Packet',
                    'TS': 'Timestamp', # Overwrites the formatted time with device time
                    'Altitude': 'Altitude',
                    'Temp': 'Temp',
                    'Lat': 'Lat',
                    'Lon': 'Lon'
                }
                if key in key_map:
                    csv_key = key_map[key]
                    data_row[csv_key] = value

        # Write the ordered data to the CSV file
        with open(CSV_FILENAME, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=CSV_HEADER)
            writer.writerow(data_row)
            
    except Exception as e:
        print(f"[ERROR] Could not parse or save data: {data_string}. Reason: {e}")


def read_from_port(ser):
    """Thread function to continuously read data from the serial port."""
    print("Receiver thread started. Listening for data...")
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("PING:"):
                    parse_and_save_data(line)
                elif line: # Print any other non-empty messages
                    print(f"<<< Received Msg: {line}")
        except serial.SerialException as e:
            print(f"Serial error: {e}. Disconnecting.")
            break
        except Exception as e:
            # Catch other potential errors, like decoding errors
            print(f"An error occurred in receiver thread: {e}")
            time.sleep(1)


if __name__ == "__main__":
    setup_csv()
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
        exit()

    # Start the receiver thread
    receiver_thread = threading.Thread(target=read_from_port, args=(ser,), daemon=True)
    receiver_thread.start()

    print("Ground Station is running. Type commands and press Enter to send.")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            # Main thread handles sending commands
            command = input(">>> ")
            if command:
                ser.write((command + '\n').encode('utf-8'))
                print(f">>> Sent Command: {command}")
    except KeyboardInterrupt:
        print("\nExiting program. Closing serial port.")
    finally:
        if ser and ser.is_open:
            ser.close()

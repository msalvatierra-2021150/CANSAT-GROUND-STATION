import serial
import csv
import time

PORT = '/dev/ttyUSB0' # or /dev/ttyACM0
BAUD = 115200
FILE_NAME = "telemetry_log.csv"

def run_telemetry():
    print(f"Opening {PORT}...")
    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser, \
             open(FILE_NAME, 'a', newline='') as f:

            writer = csv.writer(f)
            # Write header if file is empty
            if f.tell() == 0:
                writer.writerow(["Timestamp", "Sample Index", "RSSI", "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "Alt", "Temp"])

            # Decode line
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith("TELEMETRY"):
                    data = line.split(',')
                    # Remove TELEMETRY tag
                    # Skip index 0 and take everything else
                    numeric_data = data[1:]

                    # Add timestamp
                    timestamp = time.strftime("%H:%M:%S")
                    row = [timestamp] + numeric_data

                    # Save & print
                    writer.writerow(row)
                    #f.flush() # Force write to disk for crash safety
                    print(f"Sample index {numeric_data[0]} saved")

    except serial.SerialException as e:
        print(f"Error! Disconnected? {e}")
    except KeyboardInterrupt:
        print("\nStopping...")

if __name__ == "__main__":
    run_telemetry()

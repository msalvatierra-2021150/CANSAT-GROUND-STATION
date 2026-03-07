import serial
import csv
import time
from PyQt6.QtCore import Qt, QThread, pyqtSignal

class TelemetryReceiverThread(QThread):
    # Signals for main GUI
    status_update = pyqtSignal(str)  # For general logs
    data_received = pyqtSignal(list) # For the raw numeric data if needed later

    def run(self):
        PORT = '/dev/ttyUSB0' 
        BAUD = 115200
        FILE_NAME = "telemetry_log.csv"

        self.status_update.emit(f"Opening {PORT}...")
        
        try:
            with serial.Serial(PORT, BAUD, timeout=1) as ser, \
                 open(FILE_NAME, 'a', newline='') as f:

                writer = csv.writer(f)
                if f.tell() == 0:
                    writer.writerow(["time", "#", "rssi", "ax", "ay", "az", "gx", "gy", "gz", "alt", "temp"])

                while True:
                    # Check if thread was told to stop
                    if self.isInterruptionRequested():
                        break

                    line = ser.readline().decode('utf-8', errors='ignore').strip()

                    if line.startswith("TLM"):
                        data = line.split(',')
                        numeric_data = data[1:]
                        timestamp = time.strftime("%H:%M:%S")
                        row = [timestamp] + numeric_data

                        writer.writerow(row)
                        f.flush() 
                        
                        # Tell GUI to update message box
                        self.status_update.emit(f"Sample {numeric_data[0]} saved: RSSI {numeric_data[1]}")
                        self.data_received.emit(numeric_data)

        except serial.SerialException as e:
            self.status_update.emit(f"SERIAL ERROR: {e}")


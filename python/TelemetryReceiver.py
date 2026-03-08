import serial
import csv
import time
import random # Simulation data generation
from PyQt6.QtCore import Qt, QThread, pyqtSignal

class TelemetryReceiverThread(QThread):
    status_update = pyqtSignal(str)  
    data_received = pyqtSignal(list) 

    def __init__(self, use_simulation=False): # Flag to toggle simulation, off by default
        super().__init__()
        self.use_simulation = use_simulation
        self.sample_index = 0

    def run(self):
        PORT = '/dev/ttyUSB0' 
        BAUD = 115200

        if self.use_simulation:
            self.status_update.emit("Simulation mode! :)\n")
        else:
            self.status_update.emit(f"Opening {PORT} for serial communication!")
        
        try:
            # Only open the actual port if simulation is off
            ser = None
            if not self.use_simulation:
                ser = serial.Serial(PORT, BAUD, timeout=1)

            while True:
                if self.isInterruptionRequested():
                    break

                if self.use_simulation:
                    # SIMULATOR
                    time.sleep(2.5) # Simulated data rate = 1/sleeptime
                    self.sample_index += 1
                    
                    # Generate random fluctuating data
                    rssi = random.randint(-90, -30)
                    accel = [round(random.uniform(-1, 1), 2) for _ in range(3)]
                    gyro = [round(random.uniform(-10, 10), 2) for _ in range(3)]
                    alt = round(random.uniform(100, 150), 1)
                    temp = round(random.uniform(20, 25), 1)
                    
                    # Create a list in the same format as CSV
                    numeric_data = [self.sample_index, rssi] + accel + gyro + [alt, temp]
                    # Convert all to strings to match serial.readline() behavior
                    numeric_data = [str(x) for x in numeric_data]
                
                else:
                    # ACTUAL SERIAL RECEIVER LOGIC
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line.startswith("TLM"):
                        continue
                    numeric_data = line.split(',')[1:]

                # Construct the formatted string for new data
                # row[0] = time, row[1] = #, row[2] = rssi, row[3] = ax, etc.
                formatted_msg = (
                    f"#: {numeric_data[0]}    rssi: {numeric_data[1]}    "
                    f"ax: {numeric_data[2]}    ay: {numeric_data[3]}    az: {numeric_data[4]}    "
                    f"gx: {numeric_data[5]}    gy: {numeric_data[6]}    gz: {numeric_data[7]}    "
                    f"alt: {numeric_data[8]}    temp: {numeric_data[9]}\n"
                )
                self.status_update.emit(formatted_msg)
                self.data_received.emit(numeric_data)

            if ser: ser.close() # Close port if we were using the real thing

        except Exception as e:
            self.status_update.emit(f"ERROR: {e}")

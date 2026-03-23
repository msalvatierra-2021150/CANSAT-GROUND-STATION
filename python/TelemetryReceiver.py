import serial
import csv
import time
import random 
import math # For pressure calculations
from PyQt6.QtCore import Qt, QThread, pyqtSignal

class TelemetryReceiverThread(QThread):
    status_update = pyqtSignal(str)  
    data_received = pyqtSignal(list) 

    def __init__(self, use_simulation=False): 
        super().__init__()
        self.use_simulation = use_simulation
        self.sample_index = 0
        self.current_alt = 0.0
        self.prev_alt = 0.0

    def run(self):
        PORT = '/dev/ttyUSB0' 
        BAUD = 115200

        if self.use_simulation:
            self.status_update.emit("Simulation mode! :)\n")
        else:
            self.status_update.emit(f"Opening {PORT} for serial communication!")
        
        try:
            ser = None
            if not self.use_simulation:
                ser = serial.Serial(PORT, BAUD, timeout=1)

            while True:
                if self.isInterruptionRequested():
                    break

                if self.use_simulation:
                    # SIMULATOR SPEED
                    time.sleep(0.1) 
                    self.sample_index += 1
                    
                    # MISSION SEQUENCE
                    if self.sample_index <= 150:
                        self.current_alt += random.uniform(2.5, 3.5)
                    elif 150 < self.sample_index <= 200:
                        self.current_alt -= random.uniform(12.0, 15.0)
                    elif 200 < self.sample_index <= 225:
                        self.current_alt -= random.uniform(6.0, 8.0)
                    else:
                        self.current_alt -= random.uniform(3.0, 4.5)
                    
                    # Prevent going below ground
                    self.current_alt = max(0.0, self.current_alt)

                    # CALCULATE REALISTIC SIMULATOR DATA
                    velocity = (self.current_alt - self.prev_alt) / 0.5
                    self.prev_alt = self.current_alt

                    temp = round(25.0 - (0.0065 * self.current_alt) + random.uniform(-0.1, 0.1), 2)
                    press = round(1013.25 * math.pow((1 - (0.0065 * self.current_alt / 288.15)), 5.255), 2)
                    
                    rssi = random.randint(-70, -50)
                    accel = [0.0, 0.0, 9.8] 
                    gyro = [round(random.uniform(-2, 2), 2) for _ in range(3)]
                    
                    numeric_data = [
                        self.sample_index, rssi, 
                        accel[0], accel[1], accel[2], 
                        gyro[0], round(velocity, 2), gyro[1], 
                        round(self.current_alt, 2), temp, press
                    ]
                    
                    numeric_data = [str(x) for x in numeric_data]
                
                else:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line.startswith("TLM"):
                        continue
                    numeric_data = line.split(',')[1:]

                # Formatted message terminal
                formatted_msg = (
                    f"#: {numeric_data[0]}    rssi: {numeric_data[1]}    "
                    f"ax: {numeric_data[2]}    ay: {numeric_data[3]}    az: {numeric_data[4]}    "
                    f"vel: {numeric_data[6]}    alt: {numeric_data[8]}    temp: {numeric_data[9]}\n"
                )
                self.status_update.emit(formatted_msg)
                self.data_received.emit(numeric_data)

                # STOP
                # Break if simulated altitude hits 0 after the ascent phase is done
                if self.use_simulation and self.sample_index > 150 and self.current_alt <= 0.0:
                    self.status_update.emit("Simulation complete.\n")
                    break

            if ser: ser.close()

        except Exception as e:
            self.status_update.emit(f"ERROR: {e}")

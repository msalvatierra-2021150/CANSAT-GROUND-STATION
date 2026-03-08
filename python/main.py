import sys
import csv
import time
import re
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPlainTextEdit, QPushButton)
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from TelemetryReceiver import TelemetryReceiverThread
import pyqtgraph as pg 

class GUIThread(QMainWindow):
    def __init__(self):
        super().__init__()
        # Initialize empty telemetry log
        self.telemetry_log = []
        self.showFullScreen()

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        self.main_h_layout = QHBoxLayout(self.central_widget) # Horizontal layout splits left and right sides

        """
        MESSAGE TERMINAL (LEFT SIDE OF SCREEN)
        """
        self.message_box = QPlainTextEdit()
        self.message_box.setReadOnly(True)
        self.message_box.setStyleSheet("background-color: #121212; color: #00FF41; font-family: 'Consolas'; font-size: 13pt;")
        
        self.main_h_layout.addWidget(self.message_box, 1) # Add to left side with stretch factor of 1

        self.right_v_layout = QVBoxLayout() # Vertical layout to split right side into top and bottom

        """
        LINE GRAPH 1 (TOP RIGHT CORNER OF SCREEN)
        """
        self.line_graph1 = pg.PlotWidget()
        self.line_graph1.setBackground('#1a1a1a')
        self.line_graph1.setTitle("Live Telemetry Temp", color="w", size="15pt")
        self.line_graph1.showGrid(x=True, y=True)
        
        # Background & grid appearance for line_graph1
        self.line_graph1.setBackground('#FFFFFF') # White background
        self.line_graph1.showGrid(x=True, y=True, alpha=0.3) # Sets grid visibility & transparency
        
        # Axis labels for line_graph1
        self.line_graph1.setLabel('left', 'Temperature', units='°C', color='#0a0a0a', size='12pt') # y axis label
        self.line_graph1.setLabel('bottom', 'Sample Index', color='#0a0a0a', size='12pt') # x axis label
        
        self.data_x = list(range(100))  
        self.data_y = [0] * 100         
        self.line = self.line_graph1.plot(self.data_x, self.data_y, pen=pg.mkPen(color='#00FF41', width=2))
        
        self.right_v_layout.addWidget(self.line_graph1, 1) # Add to top right corner, stretch factor 1

        """
        START TELEMETRY RECEIVER BUTTON (BOTTOM RIGHT EDGE OF SCREEN)
        """
        self.start_btn = QPushButton("Start Telemetry Receiver")
        self.start_btn.setFixedHeight(50)
        self.start_btn.clicked.connect(self.start_telemetry)
        
        self.right_v_layout.addWidget(self.start_btn, 1) # Add to bottom right, stretch factor 1

        self.main_h_layout.addLayout(self.right_v_layout, 1) # Nest the right side layout into the main horizontal layout

        """
        SAVE TELEMETRY LOG BUTTON (BOTTOM RIGHT EDGE OF SCREEN)
        """
        self.save_log_btn = QPushButton("Save Telemetry Log")
        self.save_log_btn.setFixedHeight(50)
        self.save_log_btn.clicked.connect(self.save_log)
        
        self.right_v_layout.addWidget(self.save_log_btn, 1) # Add to bottom right, stretch factor 1

        self.main_h_layout.addLayout(self.right_v_layout, 1) # Nest the right side layout into the main horizontal layout
        
        """
        TELEMETRY RECEIVER SUBMODULE THREAD
        """
        # !!!!! Toggle simulation here !!!!!
        self.telemetry_thread = TelemetryReceiverThread(use_simulation=True) # !!!!! Turn of simulation here !!!!!
        # Connect status signal to log terminal message box
        self.telemetry_thread.status_update.connect(self.update_log)
        # Connect status signal to graph update function
        # Lambda to pick the specific column to plot. Temp is column 9, index = column - 1 = 8
        self.telemetry_thread.data_received.connect(lambda data: self.update_graph(data[8]))
    """
    GUI MAIN WINDOW METHODS
    """
    def start_telemetry(self):
        if not self.telemetry_thread.isRunning():
            self.telemetry_thread.start()
            self.start_btn.setEnabled(False) 

    def update_log(self, data):
        if data != "Simulation mode! :)\n":
            # Extract ONLY the numerical values after the colons for the CSV
            # This looks for a colon, optional spaces, and then grabs the numbers
            raw_values = re.findall(r':\s*([-\d.]+)', data)
            if raw_values:
                self.telemetry_log.append(raw_values)
        
        # Convert row to string and print as a msg to the GUI
        row_string = str(data)
        self.message_box.appendPlainText(f"{time.strftime('%H:%M:%S')}\n" + row_string)
        
        # Update scroll bar limits
        self.message_box.verticalScrollBar().setValue(
            self.message_box.verticalScrollBar().maximum()
        )

    def update_graph(self, new_value):
        self.data_y = self.data_y[1:]  
        self.data_y.append(float(new_value))  
        self.line.setData(self.data_x, self.data_y)

    def save_log(self):
        file_name = time.strftime('%H%M%S') + ".csv"
        with open(file_name, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["#", "rssi", "ax", "ay", "az", "gx", "gy", "gz", "alt", "temp"])
            
            # CRITICAL FIX: Use writerows (plural) to write the list of lists
            writer.writerows(self.telemetry_log) 
            
        self.message_box.appendPlainText("Telemetry log saved as " + file_name + " :-)\n")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.telemetry_thread.requestInterruption()
            self.telemetry_thread.wait() 
            self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GUIThread()
    window.show()
    sys.exit(app.exec())

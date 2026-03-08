import sys
import csv
import time
import re
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPlainTextEdit, QPushButton, QLabel,
                             QSizePolicy)
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
        
        """
        GLOBAL STYLESHEET (DARK MODE WITH BLUE ACCENTS)
        """
        self.setStyleSheet("""
            QWidget { 
                background-color: #080808; 
                color: #FFFFFF; 
            }
            QPushButton { 
                background-color: #121212; 
                border: 2px solid #1E90FF; 
                border-radius: 8px;
                color: #FFFFFF;
                font-size: 16pt;
                font-weight: bold;
            }
            QPushButton:hover { 
                background-color: #1a3b5c; 
            }
            QPushButton:pressed { 
                background-color: #1E90FF; 
                color: #000000; 
            }
        """)

        """
        MAIN VERTICAL LAYOUT (HOLDS TITLE BAR AND MAIN HORIZONTAL SPLIT)
        """
        self.main_v_layout = QVBoxLayout(self.central_widget)

        """
        TITLE BAR (TOP EDGE OF SCREEN)
        """
        self.title_label = QLabel("Telemetry Dashboard")
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("background-color: #0a0a0a; color: #FFFFFF; font-size: 24pt; font-weight: bold; padding: 15px; border-bottom: 2px solid #1E90FF;")
        
        self.main_v_layout.addWidget(self.title_label, 0)

        """
        MAIN HORIZONTAL LAYOUT (SPLITS LEFT AND RIGHT SIDES)
        """
        self.main_h_layout = QHBoxLayout() 
        # Sets the pixel gap between the left and right halves of the screen
        self.main_h_layout.setSpacing(40) 
        self.main_v_layout.addLayout(self.main_h_layout, 1)

        """
        LEFT SIDE LAYOUT (VERTICAL SPLIT)
        """
        self.left_v_layout = QVBoxLayout() 

        """
        LINE GRAPH 1 (TOP LEFT CORNER OF SCREEN)
        """
        self.line_graph1 = pg.PlotWidget()
        self.line_graph1.setTitle("Live Telemetry Temp", color="w", size="15pt")
        
        # Background & grid appearance for line_graph1
        self.line_graph1.setBackground('#0a0a0a') 
        self.line_graph1.showGrid(x=True, y=True, alpha=0.3) 
        
        # Axis labels for line_graph1
        self.line_graph1.setLabel('left', 'Temperature', units='°C', color='white', size='12pt') 
        self.line_graph1.setLabel('bottom', 'Sample Index', color='white', size='12pt') 
        
        self.data_x = list(range(100))  
        self.data_y = [0] * 100         
        self.line = self.line_graph1.plot(self.data_x, self.data_y, pen=pg.mkPen(color='#00FFFF', width=2.5))
        
        self.left_v_layout.addWidget(self.line_graph1, 1)

        """
        LINE GRAPH 2 (BOTTOM LEFT CORNER OF SCREEN)
        """
        self.line_graph2 = pg.PlotWidget()
        self.line_graph2.setTitle("Live Telemetry Altitude", color="w", size="15pt")
        
        # Background & grid appearance for line_graph2
        self.line_graph2.setBackground('#0a0a0a') 
        self.line_graph2.showGrid(x=True, y=True, alpha=0.3) 
        
        # Axis labels for line_graph2
        self.line_graph2.setLabel('left', 'Altitude', units='m', color='white', size='12pt') 
        self.line_graph2.setLabel('bottom', 'Sample Index', color='white', size='12pt') 
        
        self.data_y2 = [0] * 100         
        self.line2 = self.line_graph2.plot(self.data_x, self.data_y2, pen=pg.mkPen(color='#1E90FF', width=2.5))
        
        self.left_v_layout.addWidget(self.line_graph2, 1) 

        self.main_h_layout.addLayout(self.left_v_layout, 1)

        """
        RIGHT SIDE LAYOUT (VERTICAL SPLIT)
        """
        self.right_v_layout = QVBoxLayout() 

        """
        MESSAGE TERMINAL (TOP RIGHT OF SCREEN)
        """
        self.message_box = QPlainTextEdit()
        self.message_box.setReadOnly(True)
        self.message_box.setStyleSheet("background-color: #121212; color: #FFFFFF; font-family: 'Consolas'; font-size: 13pt; border: 1px solid #333333; border-radius: 5px;")
        
        self.right_v_layout.addWidget(self.message_box, 1) 

        """
        BUTTONS LAYOUT (BOTTOM RIGHT EDGE OF SCREEN)
        """
        self.button_h_layout = QHBoxLayout() 
        # Added spacing between the two buttons so they don't touch
        self.button_h_layout.setSpacing(20) 

        # Start button
        self.start_btn = QPushButton("Start Telemetry Receiver")
        self.start_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.start_btn.clicked.connect(self.start_telemetry)
        
        self.button_h_layout.addWidget(self.start_btn, 1) 

        # Save button
        self.save_log_btn = QPushButton("Save Telemetry Log")
        self.save_log_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.save_log_btn.clicked.connect(self.save_log)
        
        self.button_h_layout.addWidget(self.save_log_btn, 1) 

        self.right_v_layout.addLayout(self.button_h_layout, 1) 

        self.main_h_layout.addLayout(self.right_v_layout, 1) 
        
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
        # Connect status signal to graph 2 update function
        self.telemetry_thread.data_received.connect(lambda data: self.update_graph2(data[7]))

    """
    GUI MAIN WINDOW METHODS
    """
    def start_telemetry(self):
        if not self.telemetry_thread.isRunning():
            self.telemetry_thread.start()
            self.start_btn.setEnabled(False) 

    def update_log(self, data):
        if data != "Simulation mode! :)\n":
            raw_values = re.findall(r':\s*([-\d.]+)', data)
            if raw_values:
                self.telemetry_log.append(raw_values)
        
        # Convert row to string and print as a msg
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

    def update_graph2(self, new_value):
        self.data_y2 = self.data_y2[1:]  
        self.data_y2.append(float(new_value))  
        self.line2.setData(self.data_x, self.data_y2)

    def save_log(self):
        file_name = time.strftime('%H%M%S') + ".csv"
        with open(file_name, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["#", "rssi", "ax", "ay", "az", "gx", "gy", "gz", "alt", "temp"])
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
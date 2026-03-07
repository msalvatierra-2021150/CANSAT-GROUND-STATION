import sys
import serial
import csv
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPlainTextEdit, QPushButton) # Added QHBoxLayout
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from SubmoduleThreads import TelemetryReceiverThread
import pyqtgraph as pg 

class GUIThread(QMainWindow):
    def __init__(self):
        super().__init__()
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
        self.line_graph1.setLabel('left', 'Temperature', units='°C', color='#0a0a0a', size='12pt') # Configures Y-axis label
        self.line_graph1.setLabel('bottom', 'Sample Index', color='#0a0a0a', size='12pt') # Configures X-axis label
        
        self.data_x = list(range(100))  
        self.data_y = [0] * 100         
        self.line = self.line_graph1.plot(self.data_x, self.data_y, pen=pg.mkPen(color='#00FF41', width=2))
        
        self.right_v_layout.addWidget(self.line_graph1, 1) # Add to top-right with stretch factor 1

        """
        START TELEMETRY RECEIVER BUTTON (BOTTOM RIGHT EDGE OF SCREEN)
        """
        self.start_btn = QPushButton("Start Telemetry Receiver")
        self.start_btn.setFixedHeight(50)
        self.start_btn.clicked.connect(self.start_telemetry)
        
        self.right_v_layout.addWidget(self.start_btn, 1) # Add to bottom-right with stretch factor 1 to fill the quarter

        self.main_h_layout.addLayout(self.right_v_layout, 1) # Nest the right-side layout into the main horizontal layout

        self.telemetry_thread = TelemetryReceiverThread()
        self.telemetry_thread.status_update.connect(self.add_log)

    """
    GUI MAIN WINDOW METHODS
    """
    def start_telemetry(self):
        if not self.telemetry_thread.isRunning():
            self.add_log("Starting Background Thread...")
            self.telemetry_thread.start()
            self.start_btn.setEnabled(False) 

    def add_log(self, text):
        self.message_box.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {text}")
        self.message_box.verticalScrollBar().setValue(
            self.message_box.verticalScrollBar().maximum()
        )

    def update_graph(self, new_value):
        self.data_y = self.data_y[1:]  
        self.data_y.append(float(new_value))  
        self.line.setData(self.data_x, self.data_y)

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

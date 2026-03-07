import sys
import serial
import csv
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPlainTextEdit, QPushButton, )
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from SubmoduleThreads import TelemetryReceiverThread
import pyqtgraph as pg # This is for all live plots

# Main GUI
class GUIThread(QMainWindow):
    def __init__(self):
        super().__init__()
        self.showFullScreen()

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Message box (logging terminal)
        self.message_box = QPlainTextEdit()
        self.message_box.setReadOnly(True)
        self.message_box.setStyleSheet("background-color: #121212; color: #00FF41; font-family: 'Consolas'; font-size: 13pt;")
        self.layout.addWidget(self.message_box)

        # Live line graph 1
        self.line_graph1 = pg.PlotWidget()
        self.line_graph1.setBackground('#1a1a1a')
        self.line_graph1.setTitle("Live Telemetry Temp", color="w", size="15pt")
        self.line_graph1.showGrid(x=True, y=True)
        
        # Create a line reference
        self.data_x = list(range(100))  # 100 time points
        self.data_y = [0] * 100         # Start with 100 zeros
        self.line = self.line_graph1.plot(self.data_x, self.data_y, pen=pg.mkPen(color='#00FF41', width=2))
        
        self.layout.addWidget(self.line_graph1, 2) # Weight of 2 (Graph is wider)

        # Start button
        self.start_btn = QPushButton("Start Telemetry Receiver")
        self.start_btn.setFixedHeight(50)
        self.start_btn.clicked.connect(self.start_telemetry)
        self.layout.addWidget(self.start_btn)

        # Initialize thread but don't start
        self.telemetry_thread = TelemetryReceiverThread()
        self.telemetry_thread.status_update.connect(self.add_log)

    def start_telemetry(self):
        if not self.telemetry_thread.isRunning():
            self.add_log("Starting Background Thread...")
            self.telemetry_thread.start()
            self.start_btn.setEnabled(False) # Prevent double start

    def add_log(self, text):
        self.message_box.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {text}")
        self.message_box.verticalScrollBar().setValue(
            self.message_box.verticalScrollBar().maximum()
        )

    def update_graph(self, new_value):
        """
        Call every time a new TLM packet arrives (or when we want to plot one).
        """
        self.data_y = self.data_y[1:]  # Remove oldest data point
        self.data_y.append(float(new_value))  # Add new telemetry point
        self.line.setData(self.data_x, self.data_y)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.telemetry_thread.requestInterruption()
            self.telemetry_thread.wait() # Close thread
            self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GUIThread()
    window.show()
    sys.exit(app.exec())

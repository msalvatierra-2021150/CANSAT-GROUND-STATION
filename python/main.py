import sys
import serial
import csv
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPlainTextEdit, QPushButton)
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from SubmoduleThreads import TelemetryReceiverThread

# Main GUI
class GUIThread(QMainWindow):
    def __init__(self):
        super().__init__()
        self.showFullScreen()

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Message Box
        self.message_box = QPlainTextEdit()
        self.message_box.setReadOnly(True)
        self.message_box.setStyleSheet("background-color: #121212; color: #00FF41; font-family: 'Consolas'; font-size: 13pt;")
        self.layout.addWidget(self.message_box)

        # Start Button
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

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.telemetry_thread.requestInterruption()
            self.telemetry_thread.wait() # Cleanly close thread
            self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GUIThread()
    window.show()
    sys.exit(app.exec())

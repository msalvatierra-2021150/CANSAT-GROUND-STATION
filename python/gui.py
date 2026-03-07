import sys
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPlainTextEdit, QPushButton, QLabel)
from PyQt6.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Init window
        self.setWindowTitle("Ground Station GUI")

        # Go full screen
        self.showFullScreen()

        # Central widget & layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout(self.central_widget)
        self.layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Placeholder sections
        self.label = QLabel("This is the GUI :)")
        self.label.setStyleSheet("font-size: 16px; font-weight: bold; color: #333;")

        self.layout.addWidget(self.label)

        # Scrolling message box (live logging display)
        self.message_box = QPlainTextEdit()
        self.message_box.setReadOnly(True)
        self.message_box.setStyleSheet("""
            QPlainTextEdit {
                background-color: #1e1e1e;
                color: #00ff00;
                font-family: 'Courier New';
                font-size: 14pt;
                border: 2px solid #333;
                padding: 10px;
            }
        """)
        self.layout.addWidget(self.message_box)


    def add_message(self, message):
            """Appends a timestamped message and auto-scrolls to the bottom."""
            # timestamp = datetime.now().strftime("%H:%M:%S")
            #new_text = f"[{timestamp}] System status: All systems operational."
            # appendPlainText automatically adds a new line
            self.message_box.appendPlainText(message)
            
            # Force scroll to the bottom so the newest message is always visible
            self.message_box.verticalScrollBar().setValue(
                self.message_box.verticalScrollBar().maximum()
            )

    def keyPressEvent(self, event):
        """Allow user to exit with esc or add test message with p."""
        if event.key() == Qt.Key.Key_Escape:
            self.close()
        elif event.key() == Qt.Key.Key_P:
            self.add_message("hahaahahahahaha")
        elif event.key() == Qt.Key.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()

if __name__ == "__main__":
    # Initialize application
    app = QApplication(sys.argv)

    # Create and show main window
    window = MainWindow()
    window.show()

    # Start main loop
    sys.exit(app.exec())

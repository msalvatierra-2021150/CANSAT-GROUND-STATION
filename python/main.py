import sys
import csv
import time
import re
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPlainTextEdit, QPushButton, QLabel,
                             QSizePolicy, QDialog)
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from TelemetryReceiver import TelemetryReceiverThread
from ImageProcessor import ImageProcessorThread
import pyqtgraph as pg 

class GUIThread(QMainWindow):
    def __init__(self):
        super().__init__()
        self.telemetry_log = []
        self.showFullScreen()

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        """
        GLOBAL STYLESHEET (LIGHT MODE: BLUE PRIMARY, ORANGE SECONDARY)
        """
        self.setStyleSheet("""
            QWidget { 
                background-color: #F5F5F5; 
                color: #1A1A1A; 
            }
            QPushButton { 
                background-color: #E0E0E0; 
                border: 2px solid #0056B3; /* Primary Blue */
                border-radius: 8px;
                color: #1A1A1A;
                font-size: 16pt;
                font-weight: bold;
            }
            QPushButton:hover { 
                background-color: #FFF3E0; /* Light Orange Tint */
                border: 2px solid #FF8C00; /* Secondary Orange */
            }
            QPushButton:pressed { 
                background-color: #FF8C00; /* Secondary Orange */
                color: #FFFFFF; 
            }
        """)

        self.main_v_layout = QVBoxLayout(self.central_widget)

        """
        TITLE BAR (PRIMARY BLUE ACCENT)
        """
        self.title_label = QLabel("Telemetry Dashboard")
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("""
            background-color: #FFFFFF; 
            color: #000000; 
            font-size: 24pt; 
            font-weight: bold; 
            padding: 15px; 
            border-bottom: 3px solid #0056B3; /* Primary Blue */
        """)
        
        self.main_v_layout.addWidget(self.title_label, 0)

        self.main_h_layout = QHBoxLayout() 
        self.main_h_layout.setSpacing(40) 
        self.main_v_layout.addLayout(self.main_h_layout, 1)

        """
        LEFT SIDE (GRAPHS)
        """
        self.left_v_layout = QVBoxLayout() 

        # --- Graph 1 (Environmental Conditions) ---
        self.line_graph1 = pg.PlotWidget()
        self.line_graph1.setTitle("Environmental Conditions", color="k", size="15pt")
        self.line_graph1.setBackground('#FFFFFF') 
        self.line_graph1.showGrid(x=True, y=True, alpha=0.3) 
        self.line_graph1.setXRange(0, 50, padding=0)
        
        # Y-AXIS: Temperature (20 to 40°C)
        self.line_graph1.setYRange(20, 40, padding=0)
        
        self.line_graph1.setLabel('left', 'Temperature', units='°C', color='black', size='12pt') 
        self.line_graph1.setLabel('bottom', 'Sample Index', color='black', size='12pt') 
        self.line_graph1.setLabel('right', 'Pressure', units='hPa', color='black', size='12pt')
        self.line_graph1.getAxis('left').setTextPen('k')
        self.line_graph1.getAxis('bottom').setTextPen('k')
        self.line_graph1.getAxis('right').setTextPen('k')
        
        self.legend = self.line_graph1.addLegend(offset=(30, 30))
        self.line_graph1.showAxis('right')
        
        self.p1 = self.line_graph1.plotItem
        self.p2 = pg.ViewBox()
        self.p1.scene().addItem(self.p2)
        self.p1.getAxis('right').linkToView(self.p2)
        self.p2.setXLink(self.p1)
        
        # Y-AXIS: Pressure (900 to 1100 hPa)
        self.p2.setYRange(900, 1100, padding=0)

        self.data_x = []  
        self.data_y = []         
        self.line = self.line_graph1.plot(self.data_x, self.data_y, pen=pg.mkPen(color='#0056B3', width=2.5), name="Temperature")
        
        self.data_press = []
        self.line_press = pg.PlotCurveItem(self.data_x, self.data_press, pen=pg.mkPen(color='#FF8C00', width=2.5))
        self.p2.addItem(self.line_press)
        self.legend.addItem(self.line_press, "Pressure")

        def updateViews1():
            self.p2.setGeometry(self.p1.vb.sceneBoundingRect())
            self.p2.linkedViewChanged(self.p1.vb, self.p2.XAxis)
        self.p1.vb.sigResized.connect(updateViews1)

        self.left_v_layout.addWidget(self.line_graph1, 1)

        # --- Graph 2 (Altitude & Velocity) ---
        self.line_graph2 = pg.PlotWidget()
        self.line_graph2.setTitle("CanSat Altitude & Descent Velocity", color="k", size="15pt")
        self.line_graph2.setBackground('#FFFFFF') 
        self.line_graph2.showGrid(x=True, y=True, alpha=0.3) 
        self.line_graph2.setXRange(0, 50, padding=0)
        
        # Y-AXIS: Altitude (0 to 600m)
        self.line_graph2.setYRange(0, 600, padding=0)
        
        self.legend2 = self.line_graph2.addLegend(offset=(30, 30))
        self.line_graph2.setLabel('left', 'Altitude', units='m', color='black', size='12pt') 
        self.line_graph2.setLabel('bottom', 'Sample Index', color='black', size='12pt') 
        self.line_graph2.getAxis('left').setTextPen('k')
        self.line_graph2.getAxis('bottom').setTextPen('k')

        self.line_graph2.showAxis('right')
        self.line_graph2.setLabel('right', 'Descent Velocity', units='m/s', color='black', size='12pt')
        self.line_graph2.getAxis('right').setTextPen('k')
        
        self.p3 = self.line_graph2.plotItem
        self.p4 = pg.ViewBox()
        self.p3.scene().addItem(self.p4)
        self.p3.getAxis('right').linkToView(self.p4)
        self.p4.setXLink(self.p3)
        
        # Y-AXIS: Descent Velocity (-60 to 10 m/s)
        self.p4.setYRange(-60, 10, padding=0)
        
        self.data_y2 = []  # Altitude
        self.data_vel = [] # Velocity
        
        self.line2 = self.p3.plot(self.data_x, self.data_y2, pen=pg.mkPen(color='#0056B3', width=2.5), name="Altitude")
        self.line_vel = pg.PlotCurveItem(self.data_x, self.data_vel, pen=pg.mkPen(color='#FF8C00', width=2.5))
        self.p4.addItem(self.line_vel)
        self.legend2.addItem(self.line_vel, "Descent Velocity")

        def updateViews2():
            self.p4.setGeometry(self.p3.vb.sceneBoundingRect())
            self.p4.linkedViewChanged(self.p3.vb, self.p4.XAxis)
        self.p3.vb.sigResized.connect(updateViews2)

        self.left_v_layout.addWidget(self.line_graph2, 1) 

        self.main_h_layout.addLayout(self.left_v_layout, 1)

        """
        RIGHT SIDE (TERMINAL & BUTTONS)
        """
        self.right_v_layout = QVBoxLayout() 
        self.message_box = QPlainTextEdit()
        self.message_box.setReadOnly(True)
        self.message_box.setStyleSheet("background-color: #FAFAFA; color: #000000; font-family: 'Consolas'; font-size: 13pt; border: 1px solid #CCCCCC; border-radius: 5px;")
        self.right_v_layout.addWidget(self.message_box, 1) 
        
        # BUTTONS START HERE
        self.button_h_layout = QHBoxLayout() 
        self.button_h_layout.setSpacing(20) 

        # --- MISSION STATUS INDICATOR (Left Side) ---
        self.phase_v_layout = QVBoxLayout()
        self.phase_v_layout.setSpacing(5)
        
        # --- MISSION STATE VARIABLES ---
        self.max_altitude = 0.0
        self.previous_altitude = 0.0
        self.mission_phase = 1

        # --- RX TIMER SETUP ---
        self.last_rx_seconds = 0
        self.rx_timer = QTimer(self)
        self.rx_timer.timeout.connect(self.tick_rx_timer)

        self.phase1_label = QLabel("Mission Phase: Drone Ascent")
        self.phase2_label = QLabel("Last Rx: 0s")
        self.phase3_label = QLabel("Camera waiting")
        self.phase4_label = QLabel("Mission in Progress")

        # Styling to make them look like indicator bars
        indicator_style = """
            background-color: #E0E0E0; 
            color: #1A1A1A; 
            font-size: 14pt; 
            font-weight: bold; 
            border: 1px solid #CCCCCC;
            border-radius: 5px;
            padding: 5px;
        """
        
        for phase_label in [self.phase1_label, self.phase2_label, self.phase3_label, self.phase4_label]:
            phase_label.setStyleSheet(indicator_style)
            phase_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.phase_v_layout.addWidget(phase_label)

        # Add the phase indicator layout to the left side
        self.button_h_layout.addLayout(self.phase_v_layout, 1)

        self.stacked_button_v_layout = QVBoxLayout()
        self.stacked_button_v_layout.setSpacing(10)

        self.start_btn = QPushButton("Start Mission Systems")
        self.start_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.start_btn.clicked.connect(self.start_systems)
        self.stacked_button_v_layout.addWidget(self.start_btn, 1) 
        
        self.save_log_btn = QPushButton("Save Telemetry Log")
        self.save_log_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.save_log_btn.clicked.connect(self.save_log)
        self.stacked_button_v_layout.addWidget(self.save_log_btn, 1) 

        self.button_h_layout.addLayout(self.stacked_button_v_layout, 1)

        self.right_v_layout.addLayout(self.button_h_layout, 1)

        self.main_h_layout.addLayout(self.right_v_layout, 1) 
        
        # --- THREAD INITIALIZATION ---
        self.telemetry_thread = TelemetryReceiverThread(use_simulation=False) 
        self.telemetry_thread.status_update.connect(self.update_log)
        
        self.telemetry_thread.image_received.connect(self.show_image_popup)
        
        # --- INDICES HERE ---
        self.telemetry_thread.data_received.connect(lambda data: self.update_graph(data[9]))
        self.telemetry_thread.data_received.connect(lambda data: self.update_graph2(data[8]))
        self.telemetry_thread.data_received.connect(lambda data: self.update_graph_pressure(data[10]))
        self.telemetry_thread.data_received.connect(lambda data: self.update_graph_velocity(data[6]))

    def start_systems(self):
        # Start both threads when the button is clicked
        if not self.telemetry_thread.isRunning():
            self.telemetry_thread.start()
        if not self.image_thread.isRunning():
            self.image_thread.start()
            
        # Start the Rx tracking timer
        if not self.rx_timer.isActive():
            self.rx_timer.start(1000) # 1000 milliseconds = 1 second
            
        self.start_btn.setEnabled(False) 

    def tick_rx_timer(self):
        """Called every second to increment the Last Rx timer"""
        if self.last_rx_seconds < 30:
            self.last_rx_seconds += 1
        
        self.phase2_label.setText(f"Last Rx: {self.last_rx_seconds}s")
        
    def reset_rx_timer(self):
        """Called whenever new telemetry data is detected"""
        self.last_rx_seconds = 0
        self.phase2_label.setText("Last Rx: 0s")

    def show_image_popup(self, image_path):
        """Displays a pop-up dialog with the image sent from the processor thread"""
        
        # --- UPDATE PHASE 3 INDICATOR ---
        self.phase3_label.setText("Camera Done")
        self.phase3_label.setStyleSheet("""
            background-color: #28A745; /* Success Green */
            color: #FFFFFF; 
            font-size: 14pt; 
            font-weight: bold; 
            border: 1px solid #1E7E34;
            border-radius: 5px;
            padding: 5px;
        """)

        # Create a new dialog window
        self.image_dialog = QDialog(self)
        self.image_dialog.setWindowTitle("Stereoscopic Image Received")
        
        layout = QVBoxLayout()
        image_label = QLabel()
        
        # Load the image
        pixmap = QPixmap(image_path)
        
        if pixmap.isNull():
            self.message_box.appendPlainText(f"Error: Could not load {image_path}. Check if file exists.\n")
        else:
            # Scale the image slightly if it's too huge, keeping aspect ratio
            scaled_pixmap = pixmap.scaled(800, 600, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            image_label.setPixmap(scaled_pixmap)
            
        layout.addWidget(image_label)
        self.image_dialog.setLayout(layout)
        self.image_dialog.exec()

    def update_log(self, data):
        # We process incoming string data
        if data != "Simulation mode! :)\n":
            raw_values = re.findall(r':\s*([-\d.]+)', data)
            if raw_values:
                # We successfully found telemetry numbers, meaning data is valid!
                self.telemetry_log.append(raw_values)
                self.reset_rx_timer() # Reset the Phase 2 Timer
        
        row_string = str(data)
        self.message_box.appendPlainText(f"{time.strftime('%H:%M:%S')}\n" + row_string)
        self.message_box.verticalScrollBar().setValue(self.message_box.verticalScrollBar().maximum())

    def update_graph(self, new_value):
        self.data_y.append(float(new_value))
        self.data_x.append(len(self.data_y) - 1)
        self.line.setData(self.data_x, self.data_y)
        if len(self.data_x) > 50:
            self.line_graph1.setXRange(0, len(self.data_x), padding=0)

    def update_graph2(self, new_value):
        altitude = float(new_value)
        self.data_y2.append(altitude)
        self.line2.setData(self.data_x, self.data_y2)
        if len(self.data_x) > 50:
            self.line_graph2.setXRange(0, len(self.data_x), padding=0)
            
        # Altitude -> state machine
        self.check_mission_phase(altitude)

    def update_graph_pressure(self, new_value):
        self.data_press.append(float(new_value))
        self.line_press.setData(self.data_x, self.data_press)

    def update_graph_velocity(self, new_value):
        self.data_vel.append(float(new_value))
        self.line_vel.setData(self.data_x, self.data_vel)

    def check_mission_phase(self, current_altitude):
        # Update maximum altitude reached
        if current_altitude > self.max_altitude:
            self.max_altitude = current_altitude

        # Check Phase 1 -> Phase 2 (Ascent to Free Fall) transition
        if self.mission_phase == 1:
            # If we hit 400m AND current altitude is lower than last reading
            if self.max_altitude >= 400.0 and current_altitude < self.previous_altitude:
                self.mission_phase = 2
                self.phase1_label.setText("Mission Phase: Free Fall")

        # Check Phase 2 -> Phase 3 (Free Fall to Descent) transition
        elif self.mission_phase == 2:
            if current_altitude <= 100.0:
                self.mission_phase = 3
                self.phase1_label.setText("Mission Phase: Descent")
                
        # Check Phase 3 -> Phase 4 (Descent to Landed) transition
        elif self.mission_phase == 3:
            if current_altitude <= 0.0:
                self.mission_phase = 4
                self.phase1_label.setText("Mission Phase: Landed")
                self.phase4_label.setText("Mission Complete")
                # Make the Mission Complete indicator glow green
                self.phase4_label.setStyleSheet("""
                    background-color: #28A745; /* Success Green */
                    color: #FFFFFF; 
                    font-size: 14pt; 
                    font-weight: bold; 
                    border: 1px solid #1E7E34;
                    border-radius: 5px;
                    padding: 5px;
                """)

        # Save current altitude for the next comparison
        self.previous_altitude = current_altitude

    def save_log(self):
        file_name = time.strftime('%H%M%S') + ".csv"
        with open(file_name, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["#", "rssi", "ax", "ay", "az", "gx", "gy", "gz", "alt", "temp", "press"])
            writer.writerows(self.telemetry_log)
        self.message_box.appendPlainText("Telemetry log saved as " + file_name + " :-)\n")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            # Stop both threads and the timer cleanly
            self.telemetry_thread.requestInterruption()
            self.image_thread.requestInterruption()
            self.rx_timer.stop()
            
            self.telemetry_thread.wait() 
            self.image_thread.wait()
            
            self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GUIThread()
    window.show()
    sys.exit(app.exec())

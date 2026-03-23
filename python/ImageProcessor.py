import time
from PyQt6.QtCore import QThread, pyqtSignal

class ImageProcessorThread(QThread):
    status_update = pyqtSignal(str)  
    data_received = pyqtSignal(list) 
    image_ready = pyqtSignal(str)  # NEW SIGNAL FOR THE IMAGE

    def __init__(self, use_simulation=False): 
        super().__init__()
        self.use_simulation = use_simulation
        
    def run(self):
        try:
            self.status_update.emit("Image Processor Thread Initialized.\n")
            sim_counter = 0  # Track how many loops we've done
            
            while True:
                # 1. Always check if the main GUI wants us to stop
                if self.isInterruptionRequested():
                    self.status_update.emit("Image Processor Thread Stopped.\n")
                    break

                # 2. Simulation Mode Placeholder
                if self.use_simulation:
                    time.sleep(2.0) # Simulate time taken to process an image
                    sim_counter += 1
                    
                    if sim_counter == 3:
                        # After ~6 seconds, trigger the image!
                        self.status_update.emit("Stereoscopic Image Ready\n")
                        self.image_ready.emit("test_image.png")
                    elif sim_counter < 3:
                        self.status_update.emit("Simulating image processing...\n")
                
                # 3. Hardware Mode Placeholder
                else:
                    time.sleep(2.0) 
                    self.status_update.emit("Waiting for real camera hardware...\n")

        except Exception as e:
            self.status_update.emit(f"IMAGE PROCESSOR ERROR: {e}\n")

import cv2
import threading
import time

class CameraStream:
    def __init__(self, width=640, height=480, fps=20, gstreamer_ip="192.168.1.100"):
        self.width = width
        self.height = height
        self.fps = fps
        self.gstreamer_ip = gstreamer_ip
        self.frame = None
        self.running = False
        self.lock = threading.Lock()
        self.camera = None
        self.writer = None
        
    def start_capture(self):
        try:
            # For Raspberry Pi with CSI camera
            self.camera = cv2.VideoCapture(0)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.camera.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Setup GStreamer pipeline
            gst_out = f'appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay config-interval=1 pt=96 ! udpsink host={self.gstreamer_ip} port=5600'
            self.writer = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, self.fps, (self.width, self.height))
            
            if not self.camera.isOpened() or not self.writer.isOpened():
                print("Failed to open camera or gstreamer pipeline")
                return False
                
            self.running = True
            self.thread = threading.Thread(target=self._update_frame)
            self.thread.daemon = True
            self.thread.start()
            return True
        except Exception as e:
            print(f"Error starting camera: {e}")
            return False
    
    def _update_frame(self):
        while self.running:
            ret, frame = self.camera.read()
            if ret:
                with self.lock:
                    self.frame = frame
            else:
                print("Failed to grab frame")
                time.sleep(0.1)
    
    def get_frame(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()
    
    def send_frame(self, frame):
        if self.writer and self.writer.isOpened():
            self.writer.write(frame)
    
    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        
        if self.camera:
            self.camera.release()
        
        if self.writer:
            self.writer.release()

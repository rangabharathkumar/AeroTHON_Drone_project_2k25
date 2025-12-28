import time
import threading
import RPi.GPIO as GPIO

class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin, name="sensor"):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.name = name
        self.distance = 999
        self.running = False
        self.lock = threading.Lock()
        
        # Initialize GPIO
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            self.initialized = True
            print(f"Ultrasonic sensor {self.name} initialized on pins {trigger_pin}, {echo_pin}")
        except Exception as e:
            print(f"Error initializing ultrasonic sensor: {e}")
            self.initialized = False
    
    def measure_distance(self):
        if not self.initialized:
            return 999
            
        try:
            # Send trigger pulse
            GPIO.output(self.trigger_pin, False)
            time.sleep(0.01)
            GPIO.output(self.trigger_pin, True)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, False)
            
            # Get echo pulse
            pulse_start = time.time()
            timeout = pulse_start + 0.1  # 100ms timeout
            
            # Wait for echo to go HIGH
            while GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return 999
            
            # Wait for echo to go LOW
            pulse_end = time.time()
            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return 999
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound = 343 m/s = 34300 cm/s
            distance = round(distance, 2)
            
            return distance
        except Exception as e:
            print(f"Error measuring distance: {e}")
            return 999
    
    def start_monitoring(self):
        if not self.initialized:
            print(f"Cannot start monitoring with uninitialized sensor {self.name}")
            return False
            
        self.running = True
        self.thread = threading.Thread(target=self._monitoring_loop)
        self.thread.daemon = True
        self.thread.start()
        print(f"Started monitoring with sensor {self.name}")
        return True
    
    def _monitoring_loop(self):
        while self.running:
            dist = self.measure_distance()
            with self.lock:
                self.distance = dist
            time.sleep(0.1)  # 10 Hz measurement rate
    
    def get_distance(self):
        with self.lock:
            return self.distance
    
    def stop(self):
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        print(f"Stopped monitoring with sensor {self.name}")

class ObstacleDetector:
    def __init__(self):
        self.sensors = {}
        self.obstacle_threshold = 50  # cm
        self.critical_threshold = 30  # cm
        self.callbacks = []
        
    def add_sensor(self, trigger_pin, echo_pin, name, direction):
        sensor = UltrasonicSensor(trigger_pin, echo_pin, name)
        self.sensors[direction] = {
            "sensor": sensor,
            "distance": 999,
            "obstacle_detected": False
        }
        return sensor.initialized
    
    def start_all_sensors(self):
        for direction, sensor_data in self.sensors.items():
            sensor_data["sensor"].start_monitoring()
    
    def stop_all_sensors(self):
        for direction, sensor_data in self.sensors.items():
            sensor_data["sensor"].stop()
    
    def register_callback(self, callback):
        self.callbacks.append(callback)
    
    def set_thresholds(self, obstacle_threshold, critical_threshold):
        self.obstacle_threshold = obstacle_threshold
        self.critical_threshold = critical_threshold
    
    def update(self):
        obstacles = {}
        critical_obstacles = {}
        
        for direction, sensor_data in self.sensors.items():
            distance = sensor_data["sensor"].get_distance()
            sensor_data["distance"] = distance
            
            # Check for obstacles
            if distance < self.critical_threshold:
                sensor_data["obstacle_detected"] = True
                critical_obstacles[direction] = distance
                obstacles[direction] = distance
            elif distance < self.obstacle_threshold:
                sensor_data["obstacle_detected"] = True
                obstacles[direction] = distance
            else:
                sensor_data["obstacle_detected"] = False
        
        # Call callbacks if obstacles detected
        if obstacles:
            for callback in self.callbacks:
                callback(obstacles, critical_obstacles)
        
        return obstacles, critical_obstacles
    
    def is_path_clear(self, direction):
        if direction not in self.sensors:
            return True  # No sensor in this direction
            
        return not self.sensors[direction]["obstacle_detected"]
    
    def get_distances(self):
        distances = {}
        for direction, sensor_data in self.sensors.items():
            distances[direction] = sensor_data["distance"]
        return distances

import time
import threading

# This will only work on Raspberry Pi
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("RPi.GPIO not available, running in simulation mode")
    GPIO_AVAILABLE = False

class ServoController:
    def __init__(self, pin=17):
        self.pin = pin
        self.lock = threading.Lock()
        self.initialized = False
        
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.pin, GPIO.OUT)
                self.pwm = GPIO.PWM(self.pin, 50)  # 50Hz frequency
                self.pwm.start(0)
                self.initialized = True
                print(f"Servo initialized on pin {self.pin}")
            except Exception as e:
                print(f"Error initializing servo: {e}")
        
    def set_angle(self, angle):
        if not self.initialized:
            print("Servo not initialized, simulating angle set")
            return
            
        with self.lock:
            try:
                # Convert angle to duty cycle (0-180 degrees maps to 2-12% duty cycle)
                duty = angle / 18 + 2
                self.pwm.ChangeDutyCycle(duty)
                time.sleep(0.3)  # Allow time for servo to move
                self.pwm.ChangeDutyCycle(0)  # Stop sending pulses
            except Exception as e:
                print(f"Error setting servo angle: {e}")
    
    def release_payload(self):
        print("Releasing payload...")
        # Move to drop position
        self.set_angle(90)
        time.sleep(0.5)
        # Return to closed position
        self.set_angle(0)
        print("Payload released")
        
    def cleanup(self):
        if self.initialized:
            try:
                self.pwm.stop()
                GPIO.cleanup(self.pin)
                print("Servo cleanup completed")
            except Exception as e:
                print(f"Error during servo cleanup: {e}")

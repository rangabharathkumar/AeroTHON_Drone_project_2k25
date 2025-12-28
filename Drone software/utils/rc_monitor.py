import threading
import time

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("RPi.GPIO not available, running in simulation mode")
    GPIO_AVAILABLE = False

class RCChannelMonitor:
    def __init__(self, config_loader=None):
        self.config_loader = config_loader
        self.channels = {}
        self.callbacks = {}
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        
        # Try to initialize from config
        if self.config_loader:
            self.init_from_config()
        
    def init_from_config(self):
        if not self.config_loader:
            return
            
        rc_functions = self.config_loader.get_rc_channel_functions()
        if not rc_functions:
            return
            
        # Mode switch
        if "mode_switch" in rc_functions:
            mode_switch = rc_functions["mode_switch"]
            self.add_channel(
                name="mode_switch",
                channel=mode_switch["channel"],
                positions=mode_switch["positions"]
            )
            
        # Area select
        if "area_select" in rc_functions:
            area_select = rc_functions["area_select"]
            self.add_channel(
                name="area_select",
                channel=area_select["channel"],
                positions=area_select["positions"]
            )
            
        # Payload drop
        if "payload_drop" in rc_functions:
            payload_drop = rc_functions["payload_drop"]
            self.add_channel(
                name="payload_drop",
                channel=payload_drop["channel"],
                position=payload_drop.get("position", 1)
            )
    
    def add_channel(self, name, channel, positions=None, position=None):
        with self.lock:
            self.channels[name] = {
                "channel": channel,
                "positions": positions,
                "position": position,
                "value": None,
                "last_value": None,
                "pin": self._get_gpio_pin(channel)
            }
            
    def _get_gpio_pin(self, channel):
        # Map RC receiver channel to GPIO pin
        # This mapping depends on your specific wiring
        channel_to_pin = {
            5: 17,  # Mode switch on GPIO17
            6: 27,  # Area select on GPIO27
            7: 22   # Payload drop on GPIO22
        }
        return channel_to_pin.get(channel, None)
    
    def register_callback(self, channel_name, callback):
        with self.lock:
            self.callbacks[channel_name] = callback
    
    def start_monitoring(self):
        if not GPIO_AVAILABLE:
            print("GPIO not available, cannot monitor RC channels")
            return False
            
        if self.running:
            return True
            
        try:
            GPIO.setmode(GPIO.BCM)
            
            # Setup GPIO pins for each channel
            for name, channel in self.channels.items():
                pin = channel["pin"]
                if pin is not None:
                    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                    
                    # Add event detection for payload drop (simple trigger)
                    if name == "payload_drop":
                        GPIO.add_event_detect(
                            pin, 
                            GPIO.RISING, 
                            callback=self._payload_drop_callback,
                            bouncetime=300
                        )
            
            # Start monitoring thread for other channels
            self.running = True
            self.thread = threading.Thread(target=self._monitoring_loop)
            self.thread.daemon = True
            self.thread.start()
            
            print("RC channel monitoring started")
            return True
            
        except Exception as e:
            print(f"Error starting RC monitoring: {e}")
            return False
    
    def _monitoring_loop(self):
        while self.running:
            with self.lock:
                for name, channel in self.channels.items():
                    # Skip payload drop channel (handled by event detection)
                    if name == "payload_drop":
                        continue
                        
                    pin = channel["pin"]
                    if pin is not None:
                        # Read the current value
                        value = GPIO.input(pin)
                        channel["last_value"] = channel["value"]
                        channel["value"] = value
                        
                        # Check if value changed
                        if channel["value"] != channel["last_value"] and channel["last_value"] is not None:
                            self._handle_channel_change(name, channel)
            
            time.sleep(0.1)  # Check every 100ms
    
    def _handle_channel_change(self, name, channel):
        # Get position label based on value
        position_label = None
        if channel["positions"] and str(channel["value"]) in channel["positions"]:
            position_label = channel["positions"][str(channel["value"])]
        
        # Call the registered callback
        if name in self.callbacks and callable(self.callbacks[name]):
            self.callbacks[name](channel["value"], position_label)
    
    def _payload_drop_callback(self, channel):
        with self.lock:
            if "payload_drop" in self.channels and "payload_drop" in self.callbacks:
                callback = self.callbacks["payload_drop"]
                if callable(callback):
                    callback(1, "drop")
    
    def get_channel_value(self, name):
        with self.lock:
            if name in self.channels:
                return self.channels[name]["value"]
        return None
    
    def get_channel_position(self, name):
        with self.lock:
            if name in self.channels:
                channel = self.channels[name]
                if channel["positions"] and channel["value"] is not None:
                    return channel["positions"].get(str(channel["value"]), None)
        return None
    
    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
            
        if GPIO_AVAILABLE:
            # Clean up GPIO
            for name, channel in self.channels.items():
                pin = channel["pin"]
                if pin is not None:
                    try:
                        GPIO.cleanup(pin)
                    except:
                        pass
                        
        print("RC channel monitoring stopped")

import json
import os
import time
import threading

class MissionLogger:
    def __init__(self, log_dir="./logs"):
        self.log_dir = log_dir
        self.detections = []
        self.disaster_locations = []
        self.helipad_locations = []
        self.mission_data = {
            "start_time": None,
            "end_time": None,
            "mission_type": None,
            "status": "initialized",
            "events": []
        }
        self.lock = threading.Lock()
        
        # Create log directory if it doesn't exist
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
    def start_mission(self, mission_type):
        with self.lock:
            self.mission_data["start_time"] = time.time()
            self.mission_data["mission_type"] = mission_type
            self.mission_data["status"] = "in_progress"
            self.log_event(f"Mission {mission_type} started")
            
    def end_mission(self, status="completed"):
        with self.lock:
            self.mission_data["end_time"] = time.time()
            self.mission_data["status"] = status
            self.log_event(f"Mission ended with status: {status}")
            self.save_mission_log()
            
    def log_event(self, event_description):
        with self.lock:
            event = {
                "timestamp": time.time(),
                "description": event_description
            }
            self.mission_data["events"].append(event)
            print(f"Event: {event_description}")
            
    def log_detection(self, detection_type, class_name, confidence, bbox=None, gps=None):
        with self.lock:
            detection = {
                "timestamp": time.time(),
                "type": detection_type,
                "class": class_name,
                "confidence": confidence,
                "bbox": bbox,
                "gps": gps
            }
            self.detections.append(detection)
            self.log_event(f"Detected {class_name} with confidence {confidence:.2f}")
            
            # Save disaster locations
            if detection_type == "disaster" and class_name != "normal" and gps:
                self.disaster_locations.append({
                    "class": class_name,
                    "gps": gps,
                    "timestamp": time.time(),
                    "confidence": confidence
                })
                self.log_event(f"Saved disaster location: {class_name} at {gps}")
                
            # Save helipad locations
            if detection_type == "helipad" and class_name == "target" and gps:
                self.helipad_locations.append({
                    "gps": gps,
                    "timestamp": time.time(),
                    "confidence": confidence
                })
                self.log_event(f"Saved helipad location at {gps}")
                
    def get_latest_disaster_location(self):
        with self.lock:
            if not self.disaster_locations:
                return None
            return max(self.disaster_locations, key=lambda x: x["confidence"])
            
    def get_latest_helipad_location(self):
        with self.lock:
            if not self.helipad_locations:
                return None
            return max(self.helipad_locations, key=lambda x: x["confidence"])
            
    def save_detection_log(self):
        with self.lock:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(self.log_dir, f"detections_{timestamp}.json")
            
            with open(filename, "w") as f:
                json.dump(self.detections, f, indent=2)
                
            print(f"Detection log saved to {filename}")
            
    def save_mission_log(self):
        with self.lock:
            if not self.mission_data["start_time"]:
                print("No mission data to save")
                return
                
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            mission_type = self.mission_data["mission_type"]
            filename = os.path.join(self.log_dir, f"mission_{mission_type}_{timestamp}.json")
            
            # Add detection summary to mission data
            self.mission_data["detections"] = {
                "total": len(self.detections),
                "disaster_locations": self.disaster_locations,
                "helipad_locations": self.helipad_locations
            }
            
            with open(filename, "w") as f:
                json.dump(self.mission_data, f, indent=2)
                
            print(f"Mission log saved to {filename}")
            
            # Also save detection log
            self.save_detection_log()
            
class MissionController:
    def __init__(self, mission_logger):
        self.logger = mission_logger
        self.search_area = {
            "min_lat": None,
            "max_lat": None,
            "min_lon": None,
            "max_lon": None
        }
        self.search_timeout = 300  # 5 minutes default
        self.search_start_time = None
        self.mission_altitude = 15.0  # meters
        self.drop_altitude = 10.0  # meters
        
    def set_search_area(self, min_lat, max_lat, min_lon, max_lon):
        self.search_area = {
            "min_lat": min_lat,
            "max_lat": max_lat,
            "min_lon": min_lon,
            "max_lon": max_lon
        }
        self.logger.log_event(f"Search area set: {self.search_area}")
        
    def set_search_timeout(self, timeout_seconds):
        self.search_timeout = timeout_seconds
        self.logger.log_event(f"Search timeout set to {timeout_seconds} seconds")
        
    def start_search(self):
        self.search_start_time = time.time()
        self.logger.log_event("Search started")
        
    def is_search_timeout(self):
        if not self.search_start_time:
            return False
            
        elapsed = time.time() - self.search_start_time
        return elapsed > self.search_timeout
        
    def is_in_search_area(self, lat, lon):
        if None in self.search_area.values():
            return True  # No search area defined
            
        return (
            self.search_area["min_lat"] <= lat <= self.search_area["max_lat"] and
            self.search_area["min_lon"] <= lon <= self.search_area["max_lon"]
        )
        
    def get_next_search_waypoint(self, current_lat, current_lon):
        # Simple grid search pattern
        # This is a placeholder - a real implementation would use a more sophisticated search pattern
        
        if None in self.search_area.values():
            # No search area defined, just return current position
            return current_lat, current_lon
            
        # Calculate center of search area
        center_lat = (self.search_area["min_lat"] + self.search_area["max_lat"]) / 2
        center_lon = (self.search_area["min_lon"] + self.search_area["max_lon"]) / 2
        
        # Calculate distance from current position to center
        from math import sqrt
        dist_to_center = sqrt(
            (current_lat - center_lat)**2 + 
            (current_lon - center_lon)**2
        )
        
        if dist_to_center < 0.0001:  # Very close to center
            # Move to a corner of the search area
            return self.search_area["min_lat"], self.search_area["min_lon"]
            
        # Otherwise, move toward center
        return center_lat, center_lon

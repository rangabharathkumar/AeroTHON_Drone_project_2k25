import math
import time
import threading
import numpy as np

class PathPlanner:
    def __init__(self, obstacle_detector=None):
        self.obstacle_detector = obstacle_detector
        self.waypoints = []
        self.current_waypoint_index = 0
        self.home_location = None
        self.lock = threading.Lock()
        self.path_modified = False
        self.avoidance_radius = 5  # meters
        self.safety_distance = 2   # meters
        
    def set_home_location(self, lat, lon, alt):
        self.home_location = {
            "latitude": lat,
            "longitude": lon,
            "altitude": alt
        }
        print(f"Home location set: {lat}, {lon}, {alt}")
        
    def add_waypoint(self, lat, lon, alt, waypoint_type="navigate"):
        waypoint = {
            "latitude": lat,
            "longitude": lon,
            "altitude": alt,
            "type": waypoint_type,  # navigate, land, takeoff, etc.
            "original": True,       # Flag to identify original vs. avoidance waypoints
            "visited": False
        }
        with self.lock:
            self.waypoints.append(waypoint)
        
    def clear_waypoints(self):
        with self.lock:
            self.waypoints = []
            self.current_waypoint_index = 0
            
    def get_current_waypoint(self):
        with self.lock:
            if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
                return None
            return self.waypoints[self.current_waypoint_index]
            
    def get_next_waypoint(self):
        with self.lock:
            if not self.waypoints:
                return None
                
            # Mark current waypoint as visited
            if self.current_waypoint_index < len(self.waypoints):
                self.waypoints[self.current_waypoint_index]["visited"] = True
                
            # Move to next waypoint
            self.current_waypoint_index += 1
            
            # Check if we've reached the end
            if self.current_waypoint_index >= len(self.waypoints):
                return None
                
            return self.waypoints[self.current_waypoint_index]
            
    def has_more_waypoints(self):
        with self.lock:
            return self.current_waypoint_index < len(self.waypoints)
            
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # Calculate distance between two GPS coordinates using Haversine formula
        R = 6371000  # Earth radius in meters
        
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Differences
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        # Haversine formula
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        return distance
        
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        # Calculate bearing between two GPS coordinates
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon2_rad - lon1_rad)
        
        bearing = math.atan2(y, x)
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360  # Normalize to 0-360
        
        return bearing
        
    def get_avoidance_waypoint(self, current_lat, current_lon, current_alt, obstacle_direction, obstacle_distance):
        # Calculate avoidance waypoint based on obstacle direction and distance
        
        # Convert direction to bearing
        direction_to_bearing = {
            "front": 0,
            "right": 90,
            "back": 180,
            "left": 270,
            "front_right": 45,
            "front_left": 315,
            "back_right": 135,
            "back_left": 225
        }
        
        if obstacle_direction not in direction_to_bearing:
            print(f"Unknown obstacle direction: {obstacle_direction}")
            return None
            
        # Get bearing of obstacle
        obstacle_bearing = direction_to_bearing[obstacle_direction]
        
        # Calculate avoidance bearing (perpendicular to obstacle)
        avoidance_bearing = (obstacle_bearing + 90) % 360
        
        # Calculate avoidance distance based on obstacle distance
        avoidance_distance = max(self.avoidance_radius, 
                                self.safety_distance + (self.obstacle_detector.obstacle_threshold - obstacle_distance) / 100)
        
        # Convert bearing and distance to lat/lon offset
        avoidance_lat, avoidance_lon = self.get_location_from_bearing(
            current_lat, current_lon, avoidance_bearing, avoidance_distance
        )
        
        return {
            "latitude": avoidance_lat,
            "longitude": avoidance_lon,
            "altitude": current_alt,
            "type": "avoidance",
            "original": False,
            "visited": False
        }
        
    def get_location_from_bearing(self, lat, lon, bearing, distance):
        # Calculate new lat/lon given a bearing and distance
        R = 6371000  # Earth radius in meters
        
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        bearing_rad = math.radians(bearing)
        
        # Calculate new latitude
        new_lat_rad = math.asin(
            math.sin(lat_rad) * math.cos(distance/R) + 
            math.cos(lat_rad) * math.sin(distance/R) * math.cos(bearing_rad)
        )
        
        # Calculate new longitude
        new_lon_rad = lon_rad + math.atan2(
            math.sin(bearing_rad) * math.sin(distance/R) * math.cos(lat_rad),
            math.cos(distance/R) - math.sin(lat_rad) * math.sin(new_lat_rad)
        )
        
        # Convert back to degrees
        new_lat = math.degrees(new_lat_rad)
        new_lon = math.degrees(new_lon_rad)
        
        return new_lat, new_lon
        
    def handle_obstacle(self, current_position, obstacles):
        if not obstacles or not self.obstacle_detector:
            return False
            
        current_lat = current_position["latitude"]
        current_lon = current_position["longitude"]
        current_alt = current_position["altitude"]
        
        # Get the closest obstacle
        closest_direction = min(obstacles, key=obstacles.get)
        closest_distance = obstacles[closest_direction]
        
        # Check if we need to avoid
        if closest_distance < self.obstacle_detector.obstacle_threshold:
            # Calculate avoidance waypoint
            avoidance_waypoint = self.get_avoidance_waypoint(
                current_lat, current_lon, current_alt,
                closest_direction, closest_distance
            )
            
            if avoidance_waypoint:
                with self.lock:
                    # Insert avoidance waypoint after current waypoint
                    self.waypoints.insert(self.current_waypoint_index + 1, avoidance_waypoint)
                    self.path_modified = True
                    print(f"Added avoidance waypoint: {avoidance_waypoint}")
                return True
                
        return False
        
    def optimize_path(self):
        with self.lock:
            # Remove unnecessary avoidance waypoints
            optimized = []
            for wp in self.waypoints:
                if wp["original"] or not wp["visited"]:
                    optimized.append(wp)
            
            # Update waypoints list
            self.waypoints = optimized
            
            # Reset current index if needed
            if self.current_waypoint_index >= len(self.waypoints):
                self.current_waypoint_index = max(0, len(self.waypoints) - 1)
                
    def get_return_to_home_waypoints(self, current_lat, current_lon, current_alt):
        if not self.home_location:
            print("Home location not set, cannot return to home")
            return []
            
        # Create waypoints to return home
        return_waypoints = []
        
        # First, ascend to safe altitude if needed
        safe_alt = max(current_alt, self.home_location["altitude"] + 5)
        
        if current_alt < safe_alt:
            return_waypoints.append({
                "latitude": current_lat,
                "longitude": current_lon,
                "altitude": safe_alt,
                "type": "navigate",
                "original": False,
                "visited": False
            })
        
        # Then, navigate to home location at safe altitude
        return_waypoints.append({
            "latitude": self.home_location["latitude"],
            "longitude": self.home_location["longitude"],
            "altitude": safe_alt,
            "type": "navigate",
            "original": False,
            "visited": False
        })
        
        # Finally, descend to home altitude
        return_waypoints.append({
            "latitude": self.home_location["latitude"],
            "longitude": self.home_location["longitude"],
            "altitude": self.home_location["altitude"],
            "type": "land",
            "original": False,
            "visited": False
        })
        
        return return_waypoints
        
    def set_return_to_home_path(self, current_lat, current_lon, current_alt):
        return_waypoints = self.get_return_to_home_waypoints(current_lat, current_lon, current_alt)
        
        with self.lock:
            # Clear any remaining waypoints
            self.waypoints = self.waypoints[:self.current_waypoint_index+1]
            
            # Add return waypoints
            self.waypoints.extend(return_waypoints)
            
            self.path_modified = True
            print("Return to home path set")

import math

class GPSConverter:
    def __init__(self, camera_fov_h=62.2, camera_fov_v=48.8, img_width=640, img_height=480):
        self.camera_fov_h = camera_fov_h  # Horizontal field of view in degrees
        self.camera_fov_v = camera_fov_v  # Vertical field of view in degrees
        self.img_width = img_width
        self.img_height = img_height
        
    def pixel_to_angle(self, pixel_x, pixel_y):
        # Convert pixel coordinates to angle offsets from center
        center_x = self.img_width / 2
        center_y = self.img_height / 2
        
        # Calculate angle offsets
        angle_h = ((pixel_x - center_x) / self.img_width) * self.camera_fov_h
        angle_v = ((pixel_y - center_y) / self.img_height) * self.camera_fov_v
        
        return angle_h, angle_v
        
    def angle_to_distance(self, angle_h, angle_v, altitude):
        # Convert angle offsets to ground distances
        # altitude should be in meters
        
        # Convert angles to radians
        angle_h_rad = math.radians(angle_h)
        angle_v_rad = math.radians(angle_v)
        
        # Calculate ground distances using trigonometry
        distance_x = altitude * math.tan(angle_h_rad)
        distance_y = altitude * math.tan(angle_v_rad)
        
        return distance_x, distance_y
        
    def pixel_to_gps_offset(self, pixel_x, pixel_y, altitude, current_lat, current_lon):
        # Convert pixel coordinates to GPS offset
        
        # Get angle offsets
        angle_h, angle_v = self.pixel_to_angle(pixel_x, pixel_y)
        
        # Get ground distances
        distance_x, distance_y = self.angle_to_distance(angle_h, angle_v, altitude)
        
        # Convert distances to lat/lon offsets
        # Approximate conversion: 1 degree latitude = 111,111 meters
        # 1 degree longitude = 111,111 * cos(latitude) meters
        lat_offset = distance_y / 111111
        lon_offset = distance_x / (111111 * math.cos(math.radians(current_lat)))
        
        # Calculate new lat/lon
        new_lat = current_lat + lat_offset
        new_lon = current_lon + lon_offset
        
        return new_lat, new_lon
        
    def bbox_center_to_gps(self, bbox_center, altitude, current_lat, current_lon):
        # Convert bounding box center to GPS coordinates
        pixel_x, pixel_y = bbox_center
        return self.pixel_to_gps_offset(pixel_x, pixel_y, altitude, current_lat, current_lon)
        
    def estimate_distance(self, lat1, lon1, lat2, lon2):
        # Estimate distance between two GPS coordinates using Haversine formula
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

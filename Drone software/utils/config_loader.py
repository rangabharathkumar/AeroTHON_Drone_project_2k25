import json
import os
import time

class ConfigLoader:
    def __init__(self, config_path="./config.json"):
        self.config_path = config_path
        self.config = None
        self.last_modified = 0
        self.load_config()
        
    def load_config(self):
        try:
            if os.path.exists(self.config_path):
                current_modified = os.path.getmtime(self.config_path)
                
                # Only reload if file has been modified
                if current_modified > self.last_modified:
                    with open(self.config_path, 'r') as f:
                        self.config = json.load(f)
                    self.last_modified = current_modified
                    print(f"Loaded configuration from {self.config_path}")
                    return True
            else:
                print(f"Configuration file not found: {self.config_path}")
                return False
        except Exception as e:
            print(f"Error loading configuration: {e}")
            return False
            
    def get_search_area(self, area_name=None):
        if not self.config or "search_areas" not in self.config:
            return None
            
        # If no area name specified, use default
        if area_name is None and "mission_parameters" in self.config:
            area_name = self.config["mission_parameters"].get("default_search_area")
            
        # Find the specified area
        for area in self.config["search_areas"]:
            if area["name"] == area_name:
                return area
                
        # If not found, return the first area
        if self.config["search_areas"]:
            return self.config["search_areas"][0]
            
        return None
        
    def get_target(self, target_name=None):
        if not self.config or "targets" not in self.config:
            return None
            
        # If no target name specified, use default
        if target_name is None and "mission_parameters" in self.config:
            target_name = self.config["mission_parameters"].get("default_target")
            
        # Find the specified target
        for target in self.config["targets"]:
            if target["name"] == target_name:
                return target
                
        # If not found, return the first target
        if self.config["targets"]:
            return self.config["targets"][0]
            
        return None
        
    def get_mission_parameters(self):
        if not self.config or "mission_parameters" not in self.config:
            # Return default values
            return {
                "mission_altitude": 15.0,
                "drop_altitude": 10.0,
                "search_timeout": 300
            }
            
        return self.config["mission_parameters"]
        
    def get_rc_channel_functions(self):
        if not self.config or "rc_channel_functions" not in self.config:
            return {}
            
        return self.config["rc_channel_functions"]
        
    def get_search_areas(self):
        if not self.config or "search_areas" not in self.config:
            return []
            
        return self.config["search_areas"]
        
    def get_targets(self):
        if not self.config or "targets" not in self.config:
            return []
            
        return self.config["targets"]

import asyncio
import sys
import time
import threading
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import OffboardError, PositionNedYaw

class DroneCommander:
    def __init__(self, connection_string="udp://:14540"):
        self.connection_string = connection_string
        self.drone = System()
        self.connected = False
        self.lock = threading.Lock()
        self.mission_in_progress = False
        
    async def connect(self):
        print(f"Connecting to drone on {self.connection_string}...")
        await self.drone.connect(system_address=self.connection_string)
        
        status_text_task = asyncio.ensure_future(self.print_status_text())
        
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                self.connected = True
                break
        
        if not self.connected:
            print("Failed to connect to drone")
            return False
            
        return True
        
    async def print_status_text(self):
        async for status_text in self.drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
            
    async def arm_and_takeoff(self, altitude=15.0):
        if not self.connected:
            print("Not connected to drone")
            return False
            
        print("Arming drone...")
        await self.drone.action.arm()
        
        print(f"Taking off to {altitude} meters...")
        await self.drone.action.takeoff()
        
        # Wait for drone to reach target altitude
        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m >= altitude * 0.95:  # 95% of target
                print(f"Reached target altitude of {altitude} meters")
                break
                
        return True
        
    async def goto_location(self, latitude, longitude, altitude=15.0, speed=5.0):
        if not self.connected:
            print("Not connected to drone")
            return False
            
        print(f"Going to location: Lat={latitude}, Lon={longitude}, Alt={altitude}m")
        
        await self.drone.action.goto_location(latitude, longitude, altitude, speed)
        
        # Wait until we're close to the target
        async for position in self.drone.telemetry.position():
            curr_lat = position.latitude_deg
            curr_lon = position.longitude_deg
            curr_alt = position.relative_altitude_m
            
            # Calculate rough distance (not accounting for Earth's curvature)
            dist_lat = abs(curr_lat - latitude) * 111000  # 1 deg lat = ~111km
            dist_lon = abs(curr_lon - longitude) * 111000 * \
                       abs(abs(curr_lat) - 90) / 90  # Adjust for latitude
            
            horizontal_dist = (dist_lat**2 + dist_lon**2)**0.5
            
            if horizontal_dist < 2.0 and abs(curr_alt - altitude) < 1.0:
                print("Reached target location")
                break
                
        return True
        
    async def descend_and_hold(self, target_altitude=10.0, speed=1.0):
        if not self.connected:
            print("Not connected to drone")
            return False
            
        print(f"Descending to {target_altitude} meters...")
        
        # Get current position
        position = await self.drone.telemetry.position().__anext__()
        latitude = position.latitude_deg
        longitude = position.longitude_deg
        
        # Go to same lat/lon but lower altitude
        await self.goto_location(latitude, longitude, target_altitude, speed)
        
        # Hold position for a few seconds
        print("Holding position...")
        await asyncio.sleep(5)
        
        return True
        
    async def return_to_launch(self):
        if not self.connected:
            print("Not connected to drone")
            return False
            
        print("Returning to launch point...")
        await self.drone.action.return_to_launch()
        
        # Wait until we're close to home
        home_pos = await self.drone.telemetry.home().__anext__()
        home_lat = home_pos.latitude_deg
        home_lon = home_pos.longitude_deg
        
        async for position in self.drone.telemetry.position():
            curr_lat = position.latitude_deg
            curr_lon = position.longitude_deg
            
            # Calculate rough distance
            dist_lat = abs(curr_lat - home_lat) * 111000
            dist_lon = abs(curr_lon - home_lon) * 111000 * \
                       abs(abs(curr_lat) - 90) / 90
            
            horizontal_dist = (dist_lat**2 + dist_lon**2)**0.5
            
            if horizontal_dist < 3.0:
                print("Reached home location")
                break
                
        return True
        
    async def get_current_gps(self):
        if not self.connected:
            print("Not connected to drone")
            return None
            
        position = await self.drone.telemetry.position().__anext__()
        return {
            "latitude": position.latitude_deg,
            "longitude": position.longitude_deg,
            "altitude": position.relative_altitude_m
        }
        
    async def land(self):
        if not self.connected:
            print("Not connected to drone")
            return False
            
        print("Landing...")
        await self.drone.action.land()
        
        # Wait for drone to land
        async for landed_state in self.drone.telemetry.landed_state():
            if landed_state.name == "ON_GROUND":
                print("Landed successfully")
                break
                
        return True
        
    async def disconnect(self):
        if self.connected:
            print("Disconnecting from drone...")
            # No explicit disconnect in MAVSDK, just cleanup tasks
            self.connected = False

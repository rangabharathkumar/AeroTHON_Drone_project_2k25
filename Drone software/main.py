import asyncio
import threading
import time
import os
import cv2
import numpy as np
import argparse
from flask import Flask, render_template, Response, jsonify
import json

# Import utility modules
from utils.camera import CameraStream
from utils.model_loader import ModelLoader
from utils.inference import InferenceEngine
from utils.overlay import OverlayManager
from utils.servo_control import ServoController
from utils.mavsdk_commander import DroneCommander
from utils.gps_utils import GPSConverter
from utils.mission_utils import MissionLogger, MissionController
from utils.obstacle_detection import ObstacleDetector
from utils.path_planning import PathPlanner
from utils.config_loader import ConfigLoader
from utils.rc_monitor import RCChannelMonitor

# Global variables
camera = None
model_loader = None
inference_engine = None
overlay_manager = None
servo = None
drone = None
gps_converter = None
mission_logger = None
mission_controller = None
obstacle_detector = None
path_planner = None
config_loader = None
rc_monitor = None
app = Flask(__name__)

# Status variables
running = False
current_mission = None
current_frame = None
detection_results = {}
target_info = None
disaster_info = None

def init_hardware():
    global camera, model_loader, inference_engine, overlay_manager, servo
    global drone, gps_converter, mission_logger, mission_controller
    global obstacle_detector, path_planner, config_loader, rc_monitor
    
    # Create log directory
    if not os.path.exists("./logs"):
        os.makedirs("./logs")
    
    # Initialize camera
    camera = CameraStream(width=640, height=480, fps=20)
    
    # Initialize model loader
    model_loader = ModelLoader(models_dir="./models")
    
    # Initialize inference engine
    inference_engine = InferenceEngine(model_loader)
    
    # Initialize overlay manager
    overlay_manager = OverlayManager()
    
    # Initialize servo controller
    servo = ServoController(pin=17)
    
    # Initialize obstacle detector
    obstacle_detector = ObstacleDetector()
    # Add ultrasonic sensors (trigger_pin, echo_pin, name, direction)
    obstacle_detector.add_sensor(23, 24, "front_sensor", "front")
    obstacle_detector.add_sensor(25, 26, "right_sensor", "right")
    obstacle_detector.add_sensor(27, 28, "left_sensor", "left")
    obstacle_detector.add_sensor(29, 30, "back_sensor", "back")
    obstacle_detector.start_all_sensors()
    
    # Initialize path planner
    path_planner = PathPlanner(obstacle_detector)
    
    # Initialize config loader
    config_loader = ConfigLoader(config_path="./config.json")
    
    # Initialize RC monitor
    rc_monitor = RCChannelMonitor(config_loader)
    
    # Initialize drone commander
    drone = DroneCommander(connection_string="udp://:14540")
    
    # Initialize GPS converter
    gps_converter = GPSConverter(camera_fov_h=62.2, camera_fov_v=48.8, 
                                img_width=640, img_height=480)
    
    # Initialize mission logger
    mission_logger = MissionLogger(log_dir="./logs")
    
    # Initialize mission controller
    mission_controller = MissionController(mission_logger)
    
    # Setup RC callbacks
    rc_monitor.register_callback("mode_switch", mode_switch_callback)
    rc_monitor.register_callback("area_select", area_select_callback)
    rc_monitor.register_callback("payload_drop", payload_drop_callback)
    rc_monitor.start_monitoring()
    
    print("Hardware initialized")
    return True

def mode_switch_callback(value, mode):
    global current_mission
    
    if mode and mode != current_mission:
        print(f"Switching mission mode to: {mode}")
        mission_logger.log_event(f"RC switch changed mission mode to: {mode}")
        
        # Change mission mode
        if mode == "autonomous" and current_mission != "autonomous":
            # Start autonomous mission in a new thread
            current_mission = "autonomous"
            threading.Thread(target=lambda: asyncio.run(autonomous_mission())).start()
        elif mode == "manual" and current_mission != "manual":
            # Switch to manual mode
            current_mission = "manual"
            threading.Thread(target=lambda: asyncio.run(manual_mission())).start()

def area_select_callback(value, area_name):
    global path_planner, mission_controller, config_loader
    
    if area_name:
        print(f"Selected search area: {area_name}")
        mission_logger.log_event(f"RC switch selected search area: {area_name}")
        
        # Get the selected search area from config
        area = config_loader.get_search_area(area_name)
        if area:
            # Set search area boundaries
            mission_controller.set_search_area(
                area["min_lat"], area["max_lat"],
                area["min_lon"], area["max_lon"]
            )
            
            # Clear existing waypoints
            path_planner.clear_waypoints()
            
            # Add waypoints from config
            if "waypoints" in area:
                for wp in area["waypoints"]:
                    path_planner.add_waypoint(
                        wp["lat"], wp["lon"], wp["alt"]
                    )

def payload_drop_callback(value, label):
    global servo, mission_logger
    
    if value == 1:  # Switch is in the "drop" position
        print("RC signal received to drop payload")
        mission_logger.log_event("RC switch triggered payload drop")
        
        if servo:
            servo.release_payload()

def obstacle_callback(obstacles, critical_obstacles):
    global current_mission
    
    # Log obstacle detection
    for direction, distance in obstacles.items():
        mission_logger.log_event(f"Obstacle detected: {direction} at {distance}cm")
    
    # For critical obstacles, take immediate action
    if critical_obstacles:
        mission_logger.log_event(f"CRITICAL OBSTACLE DETECTED: {critical_obstacles}")
        
        if current_mission == "manual":
            # Alert pilot
            print("ALERT: CRITICAL OBSTACLE DETECTED!")
            # You could add a sound alert or other notification here
        else:
            # In autonomous mode, the path planner will handle avoidance
            pass

def process_frame(frame):
    global detection_results, target_info, disaster_info
    
    # Set frame for inference
    inference_engine.set_frame(frame)
    
    # Get results from inference engine
    helipad_results = inference_engine.get_results("helipad_model")
    disaster_results = inference_engine.get_results("disaster_model")
    object_results = inference_engine.get_results("object_detection_model")
    
    # Store results
    detection_results = {
        "helipad": helipad_results,
        "disaster": disaster_results,
        "object": object_results
    }
    
    # Process frame with overlays
    processed_frame = frame.copy()
    
    # Draw object detections
    if object_results:
        processed_frame = overlay_manager.draw_predictions(processed_frame, object_results, "object")
    
    # Draw disaster detections
    if disaster_results:
        processed_frame = overlay_manager.draw_predictions(processed_frame, disaster_results, "disaster")
        
        # Check for disasters
        for result in disaster_results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0].item())
                cls_name = result.names[cls_id]
                conf = box.conf[0].item()
                
                # If we detect a disaster (not normal)
                if cls_name != "normal" and conf > 0.6:
                    disaster_info = {
                        "class": cls_name,
                        "confidence": conf,
                        "bbox": box.xyxy[0].cpu().numpy().astype(int).tolist()
                    }
                    
                    # Log disaster detection
                    if current_mission == "autonomous" and mission_logger:
                        # Get current GPS
                        asyncio.run_coroutine_threadsafe(log_disaster_detection(cls_name, conf), asyncio.get_event_loop())
    
    # Draw helipad detections and get target info
    if helipad_results:
        processed_frame = overlay_manager.draw_predictions(processed_frame, helipad_results, "helipad")
        target_info = overlay_manager.get_target_center(helipad_results, "target")
        
        if target_info:
            processed_frame = overlay_manager.highlight_target(processed_frame, target_info)
            
            # Log helipad detection
            if current_mission == "autonomous" and mission_logger:
                # Get current GPS
                asyncio.run_coroutine_threadsafe(log_helipad_detection(target_info), asyncio.get_event_loop())
    
    return processed_frame

async def log_disaster_detection(cls_name, confidence):
    # Get current GPS
    gps_data = await drone.get_current_gps()
    if gps_data:
        mission_logger.log_detection(
            "disaster", 
            cls_name, 
            confidence,
            gps=gps_data
        )

async def log_helipad_detection(target_info):
    # Get current GPS
    gps_data = await drone.get_current_gps()
    if gps_data and target_info:
        # Convert pixel coordinates to GPS offset
        pixel_x, pixel_y = target_info["center"]
        altitude = gps_data["altitude"]
        current_lat = gps_data["latitude"]
        current_lon = gps_data["longitude"]
        
        target_lat, target_lon = gps_converter.pixel_to_gps_offset(
            pixel_x, pixel_y, altitude, current_lat, current_lon
        )
        
        target_gps = {
            "latitude": target_lat,
            "longitude": target_lon,
            "altitude": altitude
        }
        
        mission_logger.log_detection(
            "helipad", 
            "target", 
            target_info["confidence"],
            bbox=target_info["bbox"],
            gps=target_gps
        )

def camera_thread():
    global current_frame, running
    
    print("Starting camera thread")
    if not camera.start_capture():
        print("Failed to start camera")
        running = False
        return
    
    while running:
        frame = camera.get_frame()
        if frame is not None:
            # Process frame
            processed_frame = process_frame(frame)
            
            # Update current frame
            current_frame = processed_frame
            
            # Stream processed frame
            camera.send_frame(processed_frame)
        
        time.sleep(0.01)
    
    camera.stop()
    print("Camera thread stopped")

async def manual_mission():
    global current_mission, running, target_info
    
    try:
        current_mission = "manual"
        mission_logger.start_mission("manual")
        mission_logger.log_event("Starting manual mission")
        
        # Connect to drone
        if not await drone.connect():
            mission_logger.log_event("Failed to connect to drone")
            mission_logger.end_mission("failed")
            return False
        
        # Register obstacle callback
        obstacle_detector.register_callback(obstacle_callback)
        
        # Get initial position and set as home
        gps_data = await drone.get_current_gps()
        if gps_data:
            path_planner.set_home_location(
                gps_data["latitude"], 
                gps_data["longitude"], 
                gps_data["altitude"]
            )
            mission_logger.log_event(f"Home location set: {gps_data}")
        
        mission_logger.log_event("Connected to drone, ready for manual control")
        
        # Main mission loop
        while running and current_mission == "manual":
            # Check for obstacles
            obstacles, critical = obstacle_detector.update()
            
            # Check if target is detected
            if target_info and target_info["confidence"] > 0.7:
                mission_logger.log_event(f"Target detected with confidence {target_info['confidence']}")
                
                # Notify ground station (in a real system, this would send a message)
                mission_logger.log_event("Notifying ground station of target detection")
                
                # Wait for a while before checking again
                await asyncio.sleep(5)
            
            await asyncio.sleep(0.1)
        
        mission_logger.end_mission("completed")
        return True
        
    except Exception as e:
        mission_logger.log_event(f"Error in manual mission: {e}")
        mission_logger.end_mission("failed")
        return False

async def autonomous_mission():
    global current_mission, running, target_info, disaster_info
    
    try:
        current_mission = "autonomous"
        mission_logger.start_mission("autonomous")
        mission_logger.log_event("Starting autonomous mission")
        
        # Connect to drone
        if not await drone.connect():
            mission_logger.log_event("Failed to connect to drone")
            mission_logger.end_mission("failed")
            return False
        
        # Register obstacle callback
        obstacle_detector.register_callback(obstacle_callback)
        
        # Get initial position and set as home
        gps_data = await drone.get_current_gps()
        if gps_data:
            path_planner.set_home_location(
                gps_data["latitude"], 
                gps_data["longitude"], 
                gps_data["altitude"]
            )
            mission_logger.log_event(f"Home location set: {gps_data}")
        
        # Arm and takeoff
        mission_logger.log_event("Arming and taking off")
        if not await drone.arm_and_takeoff(altitude=mission_controller.mission_altitude):
            mission_logger.log_event("Failed to takeoff")
            mission_logger.end_mission("failed")
            return False
        
        mission_logger.log_event(f"Reached mission altitude of {mission_controller.mission_altitude}m")
        
        # Start search
        mission_controller.start_search()
        mission_logger.log_event("Starting search pattern")
        
        # Define search area waypoints
        current_pos = await drone.get_current_gps()
        if current_pos:
            # Create a search pattern (example: simple square pattern)
            lat_offset = 0.0001  # ~10m
            lon_offset = 0.0001  # ~10m
            
            path_planner.add_waypoint(
                current_pos["latitude"] + lat_offset, 
                current_pos["longitude"], 
                mission_controller.mission_altitude
            )
            
            path_planner.add_waypoint(
                current_pos["latitude"] + lat_offset, 
                current_pos["longitude"] + lon_offset, 
                mission_controller.mission_altitude
            )
            
            path_planner.add_waypoint(
                current_pos["latitude"], 
                current_pos["longitude"] + lon_offset, 
                mission_controller.mission_altitude
            )
            
            path_planner.add_waypoint(
                current_pos["latitude"], 
                current_pos["longitude"], 
                mission_controller.mission_altitude
            )
        
        helipad_found = False
        disaster_found = False
        
        # Main mission loop
        while running and current_mission == "autonomous":
            # Check for obstacles and update path if needed
            obstacles, critical = obstacle_detector.update()
            if obstacles:
                current_pos = await drone.get_current_gps()
                if current_pos:
                    path_modified = path_planner.handle_obstacle(current_pos, obstacles)
                    if path_modified:
                        mission_logger.log_event(f"Path modified to avoid obstacles: {obstacles}")
            
            # Navigate to next waypoint if available
            current_waypoint = path_planner.get_current_waypoint()
            if current_waypoint and not path_planner.path_modified:
                await drone.goto_location(
                    current_waypoint["latitude"],
                    current_waypoint["longitude"],
                    current_waypoint["altitude"]
                )
                
                # Check if we've reached the waypoint
                current_pos = await drone.get_current_gps()
                if current_pos:
                    dist = path_planner.calculate_distance(
                        current_pos["latitude"], current_pos["longitude"],
                        current_waypoint["latitude"], current_waypoint["longitude"]
                    )
                    
                    if dist < 3.0:  # Within 3 meters
                        next_waypoint = path_planner.get_next_waypoint()
                        if next_waypoint:
                            mission_logger.log_event(f"Moving to next waypoint: {next_waypoint}")
            
            # Check if helipad is found
            if target_info and target_info["confidence"] > 0.7:
                mission_logger.log_event(f"Helipad found with confidence {target_info['confidence']}")
                helipad_found = True
                break
            
            # Check if disaster is found
            if disaster_info and disaster_info["confidence"] > 0.6:
                mission_logger.log_event(f"Disaster found: {disaster_info['class']} with confidence {disaster_info['confidence']}")
                disaster_found = True
                # Continue searching for helipad
            
            # Check if search timeout or out of search area
            if mission_controller.is_search_timeout():
                mission_logger.log_event("Search timeout reached")
                break
            
            # Check if we've completed all waypoints
            if not path_planner.has_more_waypoints():
                mission_logger.log_event("Completed all search waypoints")
                break
            
            await asyncio.sleep(1)
        
        # If helipad found, navigate to it and drop payload
        if helipad_found:
            mission_logger.log_event("Navigating to helipad")
            
            # Get latest helipad location
            helipad_location = mission_logger.get_latest_helipad_location()
            if helipad_location:
                helipad_lat = helipad_location["gps"]["latitude"]
                helipad_lon = helipad_location["gps"]["longitude"]
                
                # Clear current path and set direct path to helipad
                path_planner.clear_waypoints()
                path_planner.add_waypoint(
                    helipad_lat, helipad_lon, mission_controller.mission_altitude
                )
                
                # Navigate to helipad
                await drone.goto_location(helipad_lat, helipad_lon, mission_controller.mission_altitude)
                
                # Descend to drop altitude
                mission_logger.log_event(f"Descending to {mission_controller.drop_altitude}m")
                await drone.descend_and_hold(mission_controller.drop_altitude)
                
                # Drop payload
                mission_logger.log_event("Dropping payload")
                servo.release_payload()
                
                # Ascend back to mission altitude
                mission_logger.log_event(f"Ascending to {mission_controller.mission_altitude}m")
                await drone.goto_location(helipad_lat, helipad_lon, mission_controller.mission_altitude)
            
        # If no helipad found but disaster found, drop at disaster location
        elif disaster_found:
            mission_logger.log_event("No helipad found, navigating to disaster location")
            
            # Get latest disaster location
            disaster_location = mission_logger.get_latest_disaster_location()
            if disaster_location:
                disaster_lat = disaster_location["gps"]["latitude"]
                disaster_lon = disaster_location["gps"]["longitude"]
                
                # Clear current path and set direct path to disaster
                path_planner.clear_waypoints()
                path_planner.add_waypoint(
                    disaster_lat, disaster_lon, mission_controller.mission_altitude
                )
                
                # Navigate to disaster
                await drone.goto_location(disaster_lat, disaster_lon, mission_controller.mission_altitude)
                
                # Descend to drop altitude
                mission_logger.log_event(f"Descending to {mission_controller.drop_altitude}m")
                await drone.descend_and_hold(mission_controller.drop_altitude)
                
                # Drop payload
                mission_logger.log_event("Dropping payload")
                servo.release_payload()
                
                # Ascend back to mission altitude
                mission_logger.log_event(f"Ascending to {mission_controller.mission_altitude}m")
                await drone.goto_location(disaster_lat, disaster_lon, mission_controller.mission_altitude)
        
        # Return to launch using path planner
        mission_logger.log_event("Returning to launch")
        current_pos = await drone.get_current_gps()
        if current_pos:
            path_planner.set_return_to_home_path(
                current_pos["latitude"], 
                current_pos["longitude"], 
                current_pos["altitude"]
            )
            
            # Follow return path
            while path_planner.has_more_waypoints() and running:
                current_waypoint = path_planner.get_current_waypoint()
                if current_waypoint:
                    if current_waypoint["type"] == "land":
                        await drone.land()
                        break
                    else:
                        await drone.goto_location(
                            current_waypoint["latitude"],
                            current_waypoint["longitude"],
                            current_waypoint["altitude"]
                        )
                        
                        # Check if we've reached the waypoint
                        current_pos = await drone.get_current_gps()
                        if current_pos:
                            dist = path_planner.calculate_distance(
                                current_pos["latitude"], current_pos["longitude"],
                                current_waypoint["latitude"], current_waypoint["longitude"]
                            )
                            
                            if dist < 3.0:  # Within 3 meters
                                path_planner.get_next_waypoint()
                
                # Check for obstacles
                obstacles, critical = obstacle_detector.update()
                if obstacles:
                    current_pos = await drone.get_current_gps()
                    if current_pos:
                        path_planner.handle_obstacle(current_pos, obstacles)
                
                await asyncio.sleep(1)
        else:
            # Fallback to simple RTL if path planning fails
            await drone.return_to_launch()
            await drone.land()
        
        mission_logger.end_mission("completed")
        return True
        
    except Exception as e:
        mission_logger.log_event(f"Error in autonomous mission: {e}")
        mission_logger.end_mission("failed")
        return False

# Flask routes for web interface
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        global current_frame
        while running:
            if current_frame is not None:
                ret, jpeg = cv2.imencode('.jpg', current_frame)
                if ret:
                    yield (b'--frame\r\n'
                          b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')
            time.sleep(0.1)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    global detection_results, target_info, disaster_info, current_mission
    
    status_data = {
        "mission": current_mission,
        "target_detected": target_info is not None,
        "disaster_detected": disaster_info is not None,
        "disaster_type": disaster_info["class"] if disaster_info else None,
        "timestamp": time.time()
    }
    
    return jsonify(status_data)

@app.route('/drop_payload', methods=['POST'])
def drop_payload():
    global servo, mission_logger
    
    if servo:
        mission_logger.log_event("Manual payload drop triggered from web interface")
        servo.release_payload()
        return jsonify({"status": "success", "message": "Payload released"})
    else:
        return jsonify({"status": "error", "message": "Servo not initialized"})

def flask_thread():
    app.run(host='0.0.0.0', port=5000, threaded=True)

async def main():
    global running, current_mission
    
    parser = argparse.ArgumentParser(description='Drone Control Software')
    parser.add_argument('--mission', type=str, choices=['manual', 'autonomous'], 
                       default=None, help='Mission type (optional, can be set by RC)')
    parser.add_argument('--ip', type=str, default='192.168.1.100',
                       help='IP address for GStreamer stream')
    parser.add_argument('--config', type=str, default='./config.json',
                       help='Path to configuration file')
    args = parser.parse_args()
    
    # Initialize hardware
    if not init_hardware():
        print("Failed to initialize hardware")
        return
    
    # Set GStreamer IP
    camera.gstreamer_ip = args.ip
    
    # Load mission parameters from config
    mission_params = config_loader.get_mission_parameters()
    mission_controller.mission_altitude = mission_params.get("mission_altitude", 15.0)
    mission_controller.drop_altitude = mission_params.get("drop_altitude", 10.0)
    mission_controller.search_timeout = mission_params.get("search_timeout", 300)
    
    # Load default search area
    default_area = config_loader.get_search_area()
    if default_area:
        mission_controller.set_search_area(
            default_area["min_lat"], default_area["max_lat"],
            default_area["min_lon"], default_area["max_lon"]
        )
        
        # Add waypoints from config
        if "waypoints" in default_area:
            for wp in default_area["waypoints"]:
                path_planner.add_waypoint(
                    wp["lat"], wp["lon"], wp["alt"]
                )
    
    # Load models
    model_loader.load_model("helipad_model")
    model_loader.load_model("disaster_model")
    model_loader.load_model("object_detection_model")
    
    # Start inference engine
    inference_engine.start_inference([
        "helipad_model", 
        "disaster_model", 
        "object_detection_model"
    ])
    
    # Start camera thread
    running = True
    cam_thread = threading.Thread(target=camera_thread)
    cam_thread.daemon = True
    cam_thread.start()
    
    # Start Flask thread
    flask_thread = threading.Thread(target=flask_thread)
    flask_thread.daemon = True
    flask_thread.start()
    
    # Determine initial mission mode
    if args.mission:
        # Use command line argument if provided
        current_mission = args.mission
    else:
        # Check RC switch position
        mode = rc_monitor.get_channel_position("mode_switch")
        if mode:
            current_mission = mode
        else:
            # Default to manual if no mode is set
            current_mission = "manual"
    
    # Run initial mission
    print(f"Starting in {current_mission} mode")
    if current_mission == "manual":
        await manual_mission()
    else:
        await autonomous_mission()
    
    # Cleanup
    running = False
    cam_thread.join(timeout=1.0)
    servo.cleanup()
    if obstacle_detector:
        obstacle_detector.stop_all_sensors()
    if rc_monitor:
        rc_monitor.stop()
    await drone.disconnect()
    
    print("Mission completed")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Ensure cleanup
        if servo:
            servo.cleanup()
        if camera:
            camera.stop()

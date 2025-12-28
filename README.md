# Autonomous Drone Disaster Response System

> **WARNING: CONCEPTUAL PROJECT - HARDWARE TESTING REQUIRED**
> 
> This project is a proof-of-concept implementation for an autonomous drone disaster response system. **It can only be tested with actual hardware** including a drone, Raspberry Pi, camera, GPS module, and servo mechanisms. The code is provided for educational and demonstration purposes.

## Overview

An intelligent autonomous drone system designed for disaster response and emergency payload delivery. The system integrates computer vision, GPS navigation, obstacle avoidance, and autonomous flight control to detect disasters, locate landing zones (helipads), and deliver emergency supplies.

### Key Capabilities

- **Real-time Disaster Detection** - Identifies various disaster scenarios using deep learning
- **Helipad/Target Detection** - Locates precise landing zones for payload delivery
- **Autonomous Navigation** - GPS-based waypoint navigation with dynamic path planning
- **Obstacle Avoidance** - Multi-sensor ultrasonic obstacle detection system
- **Payload Delivery** - Automated servo-controlled payload release mechanism
- **Dual Operation Modes** - Manual and autonomous flight modes with RC control
- **Live Video Streaming** - Real-time video feed with detection overlays via Flask web interface
- **Mission Logging** - Comprehensive logging of detections, GPS coordinates, and mission events

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Drone Hardware Layer                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │  Camera  │  │GPS Module│  │Ultrasonic│  │  Servo   │   │
│  │ (CSI/USB)│  │(MAVLink) │  │ Sensors  │  │ Control  │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│              Raspberry Pi Processing Layer                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              ML Inference Engine                      │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌──────────────┐   │   │
│  │  │  Disaster   │ │   Helipad   │ │   Object     │   │   │
│  │  │  Detection  │ │  Detection  │ │  Detection   │   │   │
│  │  │   (YOLOv8)  │ │  (YOLOv8)   │ │   (YOLOv8)   │   │   │
│  │  └─────────────┘ └─────────────┘ └──────────────┘   │   │
│  └──────────────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │         Mission Control & Navigation                  │   │
│  │  • Path Planning  • GPS Conversion  • RC Monitoring  │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                   Ground Control Station                     │
│              Flask Web Interface (Port 5000)                 │
│         Live Video Feed | Mission Status | Controls          │
└─────────────────────────────────────────────────────────────┘
```

## Machine Learning Models

This repository contains the **drone control software only**. The ML models are hosted separately due to their size and training requirements:

### Model Repositories (Coming Soon)

> **Note:** The following model repositories will be published separately. Each contains:
> - Trained model weights (.pt files)
> - Training code and notebooks
> - Dataset preparation scripts
> - Model evaluation metrics

1. **Disaster Detection Model**
   - **Link:** https://github.com/rangabharathkumar/Disaster_detection_model.git
   - **Description:** Detects various disaster scenarios (fire, flood, earthquake damage, etc.)
   - **Architecture:** YOLOv8-based classification/detection
   - **Dataset:** AIDER dataset with custom augmentation
   - **Performance:** Trained for 15 epochs with enhanced data augmentation

2. **Helipad/Target Detection Model**
   - **Link:** https://github.com/rangabharathkumar/Object_Detection_model.git
   - **Description:** Identifies landing zones and target areas for payload delivery
   - **Architecture:** YOLOv8 object detection
   - **Use Case:** Precision landing and payload drop coordination

3. **Payload Drop Detection Model**
   - **Link:** https://github.com/rangabharathkumar/Payload_Drop_Point_Detection_Model.git
   - **Description:** Detects payload drop zones and validates delivery
   - **Architecture:** YOLOv8 trained on custom dataset
   - **Training:** 30 epochs, 416x416 input size, exported to TFLite

### Using the Models

To use this system, you must:

1. Clone this repository (drone control software)
2. Download the trained models from the separate repositories above
3. Place the model files in the `Drone software/models/` directory:
   ```
   Drone software/
   └── models/
       ├── disaster_model.pt
       ├── helipad_model.pt
       └── object_detection_model.pt
   ```
4. Ensure you have the required hardware setup (see Hardware Requirements below)

## Hardware Requirements

> **WARNING: This project CANNOT be tested without the following hardware:**

### Essential Components

| Component | Specification | Purpose |
|-----------|--------------|---------|
| **Drone Platform** | MAVLink-compatible (PX4/ArduPilot) | Flight control |
| **Flight Controller** | Pixhawk or compatible | Autonomous navigation |
| **Companion Computer** | Raspberry Pi 4 (4GB+ RAM) | ML inference & control |
| **Camera** | CSI Camera or USB Camera | Real-time video capture |
| **GPS Module** | MAVLink GPS | Position tracking |
| **Ultrasonic Sensors** | HC-SR04 (×4) | Obstacle detection |
| **Servo Motor** | Standard 9g servo | Payload release |
| **RC Transmitter** | 7+ channel RC | Manual control & mode switching |
| **Power Supply** | LiPo battery (appropriate for drone) | System power |

### Optional Components

- **GStreamer-compatible Ground Station** - For video streaming
- **Telemetry Radio** - Extended range communication
- **Additional Sensors** - Lidar, depth cameras for enhanced obstacle avoidance

## Software Dependencies

### Python Requirements

```bash
pip install -r requirements.txt
```

**Core Dependencies:**
- `numpy` - Numerical computations
- `opencv-python` - Computer vision and image processing
- `torch` & `torchvision` - PyTorch deep learning framework
- `ultralytics` - YOLOv8 implementation
- `mavsdk` - Drone communication and control
- `flask` - Web interface for ground control
- `RPi.GPIO` - Raspberry Pi GPIO control
- `dronekit` & `pymavlink` - MAVLink protocol support
- `gstreamer-python` - Video streaming

### System Requirements

- **Operating System:** Raspberry Pi OS (64-bit recommended)
- **Python Version:** 3.8+
- **RAM:** Minimum 4GB (8GB recommended for smooth ML inference)
- **Storage:** 16GB+ SD card

## Installation & Setup

### 1. Clone the Repository

```bash
git clone https://github.com/YOUR_USERNAME/drone-disaster-response.git
cd drone-disaster-response
```

### 2. Install Dependencies

```bash
cd "Drone software"
pip install -r requirements.txt
```

### 3. Download ML Models

Download the trained models from the separate repositories (links above) and place them in the `models/` directory.

### 4. Configure System

Edit `config.json` to set your mission parameters:

```json
{
  "search_areas": [...],
  "mission_parameters": {
    "mission_altitude": 15.0,
    "drop_altitude": 10.0,
    "search_timeout": 300
  },
  "rc_channel_functions": {...}
}
```

### 5. Hardware Connections

- Connect camera to Raspberry Pi (CSI or USB)
- Wire ultrasonic sensors to GPIO pins (as defined in `main.py`)
- Connect servo to GPIO pin 17
- Establish MAVLink connection to flight controller (default: `udp://:14540`)

### 6. Run the System

```bash
python main.py --mission autonomous --ip 192.168.1.100
```

**Command Line Arguments:**
- `--mission` - Mission type: `manual` or `autonomous` (optional, can be set via RC)
- `--ip` - IP address for GStreamer video stream (default: `192.168.1.100`)
- `--config` - Path to configuration file (default: `./config.json`)

## Operation Modes

### Manual Mode
- Pilot controls drone via RC transmitter
- System provides real-time detection overlays
- Alerts pilot of obstacles and detected targets
- Manual payload release via RC switch or web interface

### Autonomous Mode
- Drone follows pre-programmed waypoints
- Automatic disaster and helipad detection
- Dynamic obstacle avoidance and path replanning
- Autonomous navigation to target and payload delivery
- Automatic return-to-launch (RTL) after mission completion

### RC Channel Mapping

| Channel | Function | Values |
|---------|----------|--------|
| 5 | Mode Switch | 0=Manual, 1=Autonomous |
| 6 | Area Select | 0=Area 1, 1=Area 2 |
| 7 | Payload Drop | 1=Release |

## Web Interface

Access the ground control station at `http://<raspberry-pi-ip>:5000`

**Features:**
- Live video feed with detection overlays
- Real-time mission status
- Detection confidence scores
- GPS coordinates of detections
- Manual payload drop control

## Project Structure

```
drone-disaster-response/
├── Drone software/
│   ├── main.py                    # Main application entry point
│   ├── config.json                # Mission configuration
│   ├── requirements.txt           # Python dependencies
│   ├── utils/                     # Utility modules
│   │   ├── camera.py              # Camera capture and streaming
│   │   ├── model_loader.py        # ML model management
│   │   ├── inference.py           # Inference engine
│   │   ├── overlay.py             # Detection visualization
│   │   ├── servo_control.py       # Payload release mechanism
│   │   ├── mavsdk_commander.py    # Drone flight control
│   │   ├── gps_utils.py           # GPS coordinate conversion
│   │   ├── mission_utils.py       # Mission logging and control
│   │   ├── obstacle_detection.py  # Ultrasonic sensor management
│   │   ├── path_planning.py       # Waypoint navigation
│   │   ├── config_loader.py       # Configuration management
│   │   └── rc_monitor.py          # RC channel monitoring
│   ├── templates/
│   │   └── index.html             # Web interface
│   ├── models/                    # ML model files (download separately)
│   └── logs/                      # Mission logs (auto-generated)
└── ML models/                     # Training code (reference only)
    ├── disaster/                  # Disaster detection training
    ├── payload_drop/              # Payload detection training
    └── finalmodels/               # Trained model weights
```

## Configuration

### Mission Parameters

- **mission_altitude** - Flight altitude during search (meters)
- **drop_altitude** - Altitude for payload release (meters)
- **search_timeout** - Maximum search duration (seconds)

### Search Areas

Define custom search areas with GPS boundaries and waypoints in `config.json`.

### Camera Settings

Adjust camera resolution and FPS in `main.py`:
```python
camera = CameraStream(width=640, height=480, fps=20)
```

## Mission Logging

All missions are logged in the `logs/` directory with:
- Mission start/end timestamps
- GPS coordinates of all detections
- Obstacle encounters and avoidance actions
- Payload drop events
- Flight path and waypoint navigation

## Safety & Legal Considerations

> **IMPORTANT:** This is experimental software for educational purposes.

- Always test in a safe, controlled environment
- Comply with local drone regulations and airspace restrictions
- Maintain visual line of sight (VLOS) during testing
- Have a safety pilot ready to take manual control
- Test obstacle avoidance thoroughly before autonomous flights
- Ensure proper insurance and permissions for drone operations
- Never fly over people or restricted areas
- This software is provided AS-IS with no warranty

## Contributing

This is a conceptual project. Contributions, suggestions, and improvements are welcome!



## Author

Rudra Team

## Acknowledgments

- **YOLOv8** by Ultralytics for object detection framework
- **MAVSDK** for drone communication protocols
- **AIDER Dataset** for disaster detection training data
- **Roboflow** for dataset management and augmentation

---

### Related Repositories

- [Disaster Detection Model Repository](https://github.com/rangabharathkumar/Disaster_detection_model.git)
- [Helipad/Object Detection Model Repository](https://github.com/rangabharathkumar/Object_Detection_model.git)
- [Payload Drop Detection Model Repository](https://github.com/rangabharathkumar/Payload_Drop_Point_Detection_Model.git)

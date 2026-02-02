# TEKNOFEST UAV Competition Projects

This repository contains autonomous UAV control and computer vision projects developed for TEKNOFEST competitions. All implementations utilize Gazebo simulation environment, ArduPilot open-source flight control software, and ROS2 infrastructure.

## Project Overview

The projects focus on autonomous mission execution combining image processing, remote sensing, target tracking, and precision navigation for fixed-wing UAVs.

## Missions

### 1. QR Code Detection and Kamikaze Maneuver

![Kamikaze Demo](Kamikaze-Demo/kamikaze.mp4)

The fixed-wing UAV executes the following sequence:

- Autonomous flight to designated area
- Ground-based QR code detection and reading
- Controlled kamikaze dive maneuver toward the QR code
- Automatic pull-up to prevent collision after QR code acquisition

**Technical Implementation:**

- Computer vision processing for QR code detection
- Precision altitude control during dive maneuver
- Automated recovery trajectory calculation

### 2. Precision Payload Delivery and Retrieval

![Delivery and Retrieval Demo](Delivery-and-Retrieval-Demo/Gorev-Videosu.mp4)

This mission demonstrates autonomous payload management with geometric target recognition:

**Delivery Phase:**

- Detection and localization of blue hexagon marker
- Precision payload drop on blue hexagon
- Navigation to red triangle marker
- Second payload drop on red triangle

**Retrieval Phase:**

- Return navigation to target locations
- Payload recovery operations
- Reverse sequence execution

**Technical Implementation:**

- Shape and color-based target recognition
- Geometric feature extraction and classification
- Precision positioning control for payload operations
- Multi-waypoint mission planning

### 3. Dynamic Target Tracking

![Fixed-wing-uav-tracking-Demo Demo](Fixed-wing-uav-tracking-Demo/Fixed-wing-uav-tracking-Demo.mp4)

Real-time tracking of a moving target UAV:

- Visual detection and lock-on to lead aircraft
- Continuous position estimation and tracking
- Adaptive flight path adjustment
- Maintaining optimal tracking distance and orientation

**Technical Implementation:**

- Real-time object detection and tracking algorithms
- Predictive trajectory estimation
- Dynamic waypoint generation
- Sensor fusion for robust tracking

## Core Capabilities

### Waypoint Navigation System

![Waypoint-Navigation-System-Demo Demo](Waypoint-Navigation-System-Demo/Waypoint-Navigation-System-Demo.mp4)

A robust waypoint navigation framework that enables:

- Loading predefined waypoints and coordinates
- Autonomous sequential waypoint traversal
- Dynamic mission replanning
- Position hold and loiter patterns

### Image Processing Pipeline

- Real-time video stream processing
- Object detection and classification
- QR code decoding
- Geometric shape recognition (hexagons, triangles)
- Color-based filtering and segmentation

### Flight Control Integration

- ArduPilot SITL (Software In The Loop) integration
- MAVLink communication protocol implementation
- Custom flight mode transitions
- Altitude and attitude control
- Autonomous takeoff and landing sequences

## Technology Stack

- **Simulation Environment:** Gazebo
- **Flight Control Software:** ArduPilot
- **Framework:** ROS2
- **Programming Language:** Python / C++
- **Computer Vision:** OpenCV
- **Communication Protocol:** MAVLink

## Project Structure

```
.
├── Kamikaze-Demo/          # QR detection and kamikaze dive mission
├── Delivery-and-Retrieval-Demo/     # Geometric target detection and payload operations
├── Fixed-wing-uav-tracking-Demo/      # Dynamic target tracking system
├── Waypoint-Navigation-System-Demo/  # Core waypoint navigation module
```

## Video Demonstrations

Visual demonstrations of all missions are available showing:

- Complete mission execution from takeoff to landing
- Real-time computer vision processing
- Autonomous decision-making and maneuver execution
- Simulation environment and UAV behavior

## Setup and Installation

### Prerequisites

- Ubuntu 20.04 / 22.04
- ROS2 (Foxy / Humble)
- Gazebo 11
- ArduPilot SITL
- Python 3.8+
- OpenCV 4.x

### Dependencies

```bash
# ROS2 packages
sudo apt install ros-<distro>-cv-bridge ros-<distro>-vision-msgs

# Python dependencies
pip install pymavlink opencv-python numpy
```

### Building

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone <repository-url>

# Build
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```

## Running the Missions

### QR Code Kamikaze Mission

```bash
# Launch Gazebo simulation
ros2 launch qr_kamikaze gazebo_sim.launch.py

# Start ArduPilot SITL
sim_vehicle.py -v ArduPlane

# Execute mission
ros2 run qr_kamikaze qr_kamikaze_mission
```

### Payload Delivery Mission

```bash
ros2 launch payload_delivery payload_mission.launch.py
ros2 run payload_delivery target_detection
ros2 run payload_delivery payload_controller
```

### Target Tracking Mission

```bash
ros2 launch target_tracking tracking_sim.launch.py
ros2 run target_tracking tracker_node
```

## Configuration

Mission parameters can be adjusted in configuration files:

- Flight altitude and speed
- Detection thresholds
- Waypoint coordinates
- Camera parameters
- PID controller gains

## Development Notes

All missions were developed and tested in simulation environment before any real-world deployment consideration. The modular architecture allows for:

- Independent testing of subsystems
- Easy integration of new sensors
- Scalable mission complexity
- Reusable navigation and control modules

## Future Improvements

- Integration with hardware-in-the-loop testing
- Enhanced computer vision algorithms
- Multi-UAV coordination
- Obstacle avoidance systems
- Real-time path planning optimization

## Acknowledgments

Developed for TEKNOFEST UAV competitions, demonstrating autonomous flight capabilities, computer vision integration, and precision control systems.

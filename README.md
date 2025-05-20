# Auto Docking System using ArUco

> A precision visual docking system for autonomous robots using ArUco marker detection and ROS2

## System Overview

The ARUco Visual Docking System enables robots to autonomously detect, approach, and precisely dock with targets equipped with ArUco markers. This system provides a robust, camera-based solution for applications such as automated charging, payload transfer, or precision positioning.

## Features

- Real-time ArUco marker detection with OpenCV
- 6DoF pose estimation for precise docking alignment
- PID-controlled approach for smooth, stable docking
- Automatic marker search behavior when target is lost
- Complete visual feedback for monitoring and debugging

## Architecture

The system consists of three main components:

1. Video Capture Node: Interfaces with the camera and publishes raw image frames
2. Pose Estimation Node: Detects ArUco markers and calculates their 3D poses
3. Docking Controller: Processes pose data to guide the robot to its target position

## Installation

Prerequisites

- ROS2 (Foxy or later)
- Python 3.8+
- OpenCV 4.5+ with ArUco module
- cv_bridge

Setup

1. Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/b00tiful/Auto-ArUco-Docking.git
```

2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Build the ROS2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select aruco_docking_system
```

4. Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

```bash
# Start the camera node
ros2 run aruco_docking_system video_capture.py

# Start the pose estimation node
ros2 run aruco_docking_system aruco_pose_estimation.py

# Start the docking controller
ros2 run aruco_docking_system docking_controller.py
```

### Configuration

The system can be configured by modifying the following parameters:

#Camera Calibration

Update the camera intrinsic parameters in `aruco_pose_estimation.py`:

```python
# Camera intrinsic parameters
self.intrinsic_camera = np.array([
    [933.15867, 0, 657.59],
    [0, 933.1586, 400.36993],
    [0, 0, 1]
])

# Camera distortion coefficients
self.distortion = np.array([-0.43948, 0.18514, 0, 0])
```

#Marker Size

Modify the marker size (in meters) in `aruco_pose_estimation.py`:

```python
# Marker size (in meters)
self.marker_size = 0.02
```

#Docking Parameters

Adjust the docking behavior in `docking_controller.py`:

```python
# Target pose parameters (in meters)
self.target_x = 0.0        # Target x position (centered laterally)
self.target_y = 0.0        # Target y position (centered vertically)
self.target_z = 0.5        # Target distance from camera

# PID controller parameters
self.x_pid = PIDController(kp=1.0, ki=0.05, kd=0.1)
self.y_pid = PIDController(kp=0.5, ki=0.02, kd=0.05)
self.z_pid = PIDController(kp=0.8, ki=0.05, kd=0.1)
```

## System Operation

ROS2 Topic Structure

The system communicates using the following ROS2 topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera frames |
| `/pose_estimation/output_image` | `sensor_msgs/Image` | Annotated video feed with detected markers |
| `/marker_poses` | `interfaces/MarkerPose` | 3D pose data of detected markers |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot movement commands |

### Behavior States

The docking controller operates in three primary states:

1. *Waiting*: Initialized, awaiting marker detection
2. *Tracking*: Actively following and approaching a detected marker
3. *Searching*: Rotating to locate a marker when lost

## PID Control System

The docking approach uses three separate PID controllers:

- **X-axis controller**: Handles lateral positioning (controls rotation)
- **Y-axis controller**: Manages vertical alignment (if supported by robot)
- **Z-axis controller**: Controls approach distance (forward/backward movement)

### Control Parameters

Each controller can be fine-tuned with these parameters:

- Kp: Proportional gain (immediate response to error)
- Ki: Integral gain (addresses accumulated error)
- Kd: Derivative gain (dampens oscillations)

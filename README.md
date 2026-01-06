# Don's UGV Rover

A ROS2 Jazzy-based autonomous robot built on the Waveshare UGV Rover platform with custom modular packages.

![UGV Rover](images/rover.jpg)

## Hardware

- **Platform:** Waveshare UGV Rover
- **Computer:** Raspberry Pi 5 (Ubuntu 24.04)
- **Microcontroller:** ESP32 (motor control bridge)
- **LIDAR:** LD19 360° Laser Scanner
- **Sensors:** 
  - INA219 Battery Monitor
  - 128x64 OLED Display
- **Control:** Logitech F710 Wireless Gamepad

## Software Stack

- **OS:** Ubuntu 24.04
- **Framework:** ROS2 Jazzy
- **Navigation:** SLAM Toolbox

## Package Overview

- **rover_bringup** - Launch files, ESP32 bridge, joystick teleoperation
- **rover_description** - URDF model and robot visualization
- **rover_motion** - Odometry publishing
- **rover_msgs** - Custom messages and services (Patrol action, LED control)
- **rover_navigation** - SLAM configuration and mapping
- **rover_utils** - Battery monitoring, OLED display, safety stop
- **rover_example** - Example patrol behavior client/server

## Installation

### Prerequisites
```bash
sudo apt install ros-jazzy-desktop ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-slam-toolbox
```

### Clone and Build
```bash
# Create workspace
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src

# Clone this repository
git clone https://github.com/Dwilliestyle/Dons_Rover.git .

# Clone LIDAR driver dependency
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

# Build
cd ~/rover_ws
colcon build
source install/setup.bash
```

## Usage

### Launch Robot with All Sensors
```bash
ros2 launch rover_bringup robot.launch.py
```

This starts:
- ESP32 bridge for motor control
- LD19 LIDAR
- Battery monitoring
- OLED display
- Safety stop monitoring

### Teleoperation
```bash
ros2 launch rover_bringup teleop.launch.py
```

Control with Logitech F710 gamepad (Mode: X, LED off).

### SLAM Mapping
```bash
ros2 launch rover_navigation slam.launch.py
```

### Visualize Robot Model
```bash
ros2 launch rover_description display.launch.py
```

## Configuration

Robot parameters are in `rover_bringup/config/rover_params.yaml`:
- Wheel separation and radius
- Motor control limits
- Battery thresholds
- Safety parameters

## Development

Built following modular ROS2 package architecture learned through Antonio Brandi's robotics courses. Each package handles a specific aspect of robot functionality for maintainability and reusability.

## License

MIT

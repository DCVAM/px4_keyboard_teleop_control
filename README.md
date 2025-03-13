# Keyboard Teleop: Twist Keyboard Control of PX4 Drones

## Overview
This package provides a simple and efficient way to control PX4-based drones using a keyboard. Built on ROS 2, it enables seamless communication with the PX4 Autopilot via `geometry_msgs/Twist` commands, making it ideal for testing and manual control scenarios.

This repository is inspired by [PX4/px4_ros_com](https://github.com/PX4/px4_ros_com/tree/main). Before using this package, it is highly recommended to read the official PX4 documentation: [PX4 User Guide](https://docs.px4.io/main/en/).

## Features
- Direct keyboard-based control of PX4 drones.
- Utilizes `cmd_vel` messages for velocity-based control.
- Built with ROS 2 Humble and tested in Gazebo Harmonic.

## Dependencies
Ensure the following dependencies are installed before proceeding:
- **ROS 2 Humble** (https://docs.ros.org/en/humble/index.html)
- **PX4 Autopilot** (Stable version: **v1.15**) (https://github.com/PX4/PX4-Autopilot)
- **px4_msgs package** (https://github.com/PX4/px4_msgs)
- **QGroundControl** (for PX4 configuration and monitoring) (https://qgroundcontrol.com/)
- **Micro XRCE-DDS Agent** (for PX4-ROS 2 communication) (https://github.com/eProsima/Micro-XRCE-DDS-Agent)

## Installation
### 1. Clone the Repository and `px4_msgs` package
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/DCVAM/px4_keyboard_teleop_control.git
```

### 2. Build the Package 
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage
### Run QGroundControl
```bash
cd /path/to/QGroundControl.AppImage
./QGroundControl.AppImage
```

### Run the Micro XRCE-DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

### Launch the PX4 SITL Environment
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

### Start the Keyboard Teleoperation Node
```bash
cd ~/ros2_ws
ros2 launch px4_keyboard_teleop_control keyboard_control.launch.py
```

### Controls
| Key | Action |
|-----|--------|
| `w` | Increase Altitude |
| `s` | Decrease Altitude |
| `a` | Yaw Left |
| `d` | Yaw Right |
| `↑` | Move Forward |
| `↓` | Move Backward |
| `←` | Move Left |
| `→` | Move Right |
| `q` | Increase Linear Speed |
| `x` | Decrease Linear Speed |
| `e` | Increase Angular Speed |
| `c` | Decrease Angular Speed |

## Testing & Validation
This package has been tested on:
- **PX4 Stable v1.15**
- **ROS 2 Humble**
- **Gazebo Harmonic**

## Future Improvements
- Integration with joystick/gamepad controllers.
- Customizable key mappings.
- Additional safety features.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Contributions
Contributions are welcome! Please follow the standard GitHub workflow:
1. Fork the repository.
2. Create a feature branch.
3. Submit a pull request.



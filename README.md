# PX4 ROS 2 Offboard Control with Keyboard Teleoperation

This project is a fork of the [PX4/px4_ros_com](https://github.com/PX4/px4_ros_com) repository, modified to enable offboard control of a PX4-based drone using keyboard inputs via teleoperation. The original offboard control functionality has been extended to allow manual control through keyboard commands, providing an intuitive interface for drone operation.

## Features

- **Keyboard Teleoperation**: Control your drone in offboard mode using simple keyboard inputs.
- **ROS 2 Integration**: Built upon ROS 2, leveraging its robust communication infrastructure.
- **PX4 Compatibility**: Fully compatible with the PX4 autopilot system, utilizing the `px4_ros_com` interface.

## Prerequisites

Before setting up this project, ensure you have the following installed:

- **Ubuntu 20.04** (or later)
- **ROS 2 Foxy** (or later)
- **PX4 Autopilot** (version 1.14 or later)

For detailed instructions on setting up the PX4-ROS 2 bridge and offboard control, refer to the [PX4 ROS 2 Offboard Control Example](https://docs.px4.io/main/en/ros2/offboard_control.html).

## Build and Run

1. **Open a new terminal.**

2. **Create and navigate into a new colcon workspace directory using:**
   ```bash
   mkdir -p ~/ws_keyboard_control/src/
   cd ~/ws_keyboard_control/src/
   ```

3. **Clone the px4_msgs repo to the `/src` directory (this repo is needed in every ROS 2 PX4 workspace!):**
   ```bash
   git clone https://github.com/PX4/px4_msgs.git
   ```

4. **Clone the example repository `px4_keyboard_teleop_control` in the `/src` directory:**
   ```bash
   git clone https://github.com/Vezz-084/px4_keyboard_teleop_control.git
   ```

5. **Source the ROS 2 development environment into the current terminal and compile the workspace using colcon:**
   ```bash
   cd ..
   source /opt/ros/humble/setup.bash
   colcon build
   ```

6. **After building, source the setup script to overlay this workspace into your environment:**
   ```bash
   source install/setup.bash
   ```

## Running the Code

You will have to open 4 different terminals for this:

1. **Run QGroundControl.**
2. **Run MicroXRCEAgent.**
3. **Run a `px4_sitl_gz_####` instance.**
4. **Run the keyboard control node:**
   ```bash
   ros2 run px4_ros_com offboard_control.py
   ```
5. **Run the teleop node:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

## Acknowledgements

This project builds upon the excellent work of the PX4 and ROS 2 communities. Special thanks to the maintainers of the [PX4/px4_ros_com](https://github.com/PX4/px4_ros_com) repository for providing a solid foundation.

---

For more detailed information on offboard control and ROS 2 integration with PX4, please refer to the [PX4 ROS 2 Offboard Control Example](https://docs.px4.io/main/en/ros2/offboard_control.html).


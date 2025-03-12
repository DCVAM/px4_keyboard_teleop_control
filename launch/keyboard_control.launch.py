import launch
import launch_ros.actions
import launch.actions
import os

def generate_launch_description():
    # Check if we are inside a tmux session
    is_tmux = "TMUX" in os.environ

    if is_tmux:
        # Split the current tmux window horizontally and run teleop_twist_keyboard
        cmd = ['tmux', 'split-window', '-h', 'ros2', 'run', 'px4_keyboard_teleop_control', 'teleop_node.py']
    else:
        # Run normally in a new terminal
        cmd = ['gnome-terminal', '--', 'ros2', 'run', 'px4_keyboard_teleop_control', 'teleop_node.py']

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=cmd,
            output='screen'
        ),
        launch_ros.actions.Node(
            package='px4_keyboard_teleop_control',
            executable='offboard_control',
            output='screen',
            shell=True,
        )
    ])

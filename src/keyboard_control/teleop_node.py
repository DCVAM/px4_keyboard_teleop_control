#!/usr/bin/env python3
import sys

import geometry_msgs.msg
import rclpy
import std_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. 
Using the arrow keys and WASD you have Mode 2 RC controls.
w: Up
s: Down
a: Yaw Left
d: Yaw Right

Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

q/x : increase/decrease linear speed
e/c : increase/decrease angular speed
"""

moveBindings = {
    'w': (0, 0, 1, 0), #Z+
    's': (0, 0, -1, 0),#Z-
    'a': (0, 0, 0, -1), #Yaw+
    'd': (0, 0, 0, 1),#Yaw-
    '\x1b[A' : (0, 1, 0, 0),  #Up Arrow
    '\x1b[B' : (0, -1, 0, 0), #Down Arrow
    '\x1b[C' : (-1, 0, 0, 0), #Right Arrow
    '\x1b[D' : (1, 0, 0, 0),  #Left Arrow
}


speedBindings = {
    'q': (1.0, 0),
    'x': (-1.0, 0),
    'e': (0, 1.0),
    'c': (0, -1.0),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        if key == '\x1b':  # if the first character is \x1b, we might be dealing with an arrow key
            additional_chars = sys.stdin.read(2)  # read the next two characters
            key += additional_chars  # append these characters to the key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (round(speed,3), round(turn,3))


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )


    pub = node.create_publisher(geometry_msgs.msg.Twist, '/keyboard_control_cmd', qos_profile)

    arm_toggle = False
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)


    speed = 0.5
    turn = .2
    x = 0.0
    y = 0.0
    z = 0.0
    vel = 0.0
    ang_vel = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 2.5
    yaw_val = 0.0
    valid_key = False


    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                valid_key = True

            elif key in speedBindings.keys():
                vel = speedBindings[key][0]
                ang_vel = speedBindings[key][1]
                speed = (vel * 0.05) + speed
                turn = (ang_vel * 0.05) + turn
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                print(vels(speed, turn))
            
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                vel = 0.0
                ang_vel = 0.0
                if (key == '\x03'):
                    break
            
            if valid_key:
                twist = geometry_msgs.msg.Twist()
                x_val = (x * speed) + x_val
                y_val = (y * speed) + y_val
                z_val = (z * speed) + z_val
                yaw_val = (th * turn) + yaw_val
                twist.linear.x = x_val
                twist.linear.y = y_val
                twist.linear.z = z_val
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = yaw_val
                pub.publish(twist)
                print("X:",round(twist.linear.x,3), "   Y:",round(twist.linear.y,3), "   Z:",round(twist.linear.z,3), "   Yaw:",round(twist.angular.z,3))
            

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
# #!/usr/bin/env python3
# ############################################################################
# #
# #   Copyright (C) 2022 PX4 Development Team. All rights reserved.
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions
# # are met:
# #
# # 1. Redistributions of source code must retain the above copyright
# #    notice, this list of conditions and the following disclaimer.
# # 2. Redistributions in binary form must reproduce the above copyright
# #    notice, this list of conditions and the following disclaimer in
# #    the documentation and/or other materials provided with the
# #    distribution.
# # 3. Neither the name PX4 nor the names of its contributors may be
# #    used to endorse or promote products derived from this software
# #    without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# # OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.
# #
# ############################################################################

# __author__ = "Jaeyoung Lim"
# __contact__ = "jalim@ethz.ch"

# from re import M
# import numpy as np

# import rclpy
# from rclpy.node import Node
# from rclpy.clock import Clock
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# from px4_msgs.msg import VehicleAttitude
# from px4_msgs.msg import VehicleLocalPosition
# from px4_msgs.msg import TrajectorySetpoint
# from geometry_msgs.msg import PoseStamped, Point
# from nav_msgs.msg import Path
# from visualization_msgs.msg import Marker


# def vector2PoseMsg(frame_id, position, attitude):
#     pose_msg = PoseStamped()
#     # msg.header.stamp = Clock().now().nanoseconds / 1000
#     pose_msg.header.frame_id = frame_id
#     pose_msg.pose.orientation.w = attitude[0]
#     pose_msg.pose.orientation.x = attitude[1]
#     pose_msg.pose.orientation.y = attitude[2]
#     pose_msg.pose.orientation.z = attitude[3]
#     pose_msg.pose.position.x = position[0]
#     pose_msg.pose.position.y = position[1]
#     pose_msg.pose.position.z = position[2]
#     return pose_msg


# class PX4Visualizer(Node):
#     def __init__(self):
#         super().__init__("visualizer")

#         # Configure subscritpions
#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
#             history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
#             depth=1,
#         )

#         self.attitude_sub = self.create_subscription(
#             VehicleAttitude,
#             "/fmu/out/vehicle_attitude",
#             self.vehicle_attitude_callback,
#             qos_profile,
#         )
#         self.local_position_sub = self.create_subscription(
#             VehicleLocalPosition,
#             "/fmu/out/vehicle_local_position",
#             self.vehicle_local_position_callback,
#             qos_profile,
#         )
#         self.setpoint_sub = self.create_subscription(
#             TrajectorySetpoint,
#             "/fmu/in/trajectory_setpoint",
#             self.trajectory_setpoint_callback,
#             qos_profile,
#         )

#         self.vehicle_pose_pub = self.create_publisher(
#             PoseStamped, "/px4_visualizer/vehicle_pose", 10
#         )
#         self.vehicle_vel_pub = self.create_publisher(
#             Marker, "/px4_visualizer/vehicle_velocity", 10
#         )
#         self.vehicle_path_pub = self.create_publisher(
#             Path, "/px4_visualizer/vehicle_path", 10
#         )
#         self.setpoint_path_pub = self.create_publisher(
#             Path, "/px4_visualizer/setpoint_path", 10
#         )

#         self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
#         self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
#         self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
#         self.setpoint_position = np.array([0.0, 0.0, 0.0])
#         self.vehicle_path_msg = Path()
#         self.setpoint_path_msg = Path()

#         # trail size
#         self.trail_size = 1000

#         # time stamp for the last local position update received on ROS2 topic
#         self.last_local_pos_update = 0.0
#         # time after which existing path is cleared upon receiving new
#         # local position ROS2 message
#         self.declare_parameter("path_clearing_timeout", -1.0)

#         timer_period = 0.05  # seconds
#         self.timer = self.create_timer(timer_period, self.cmdloop_callback)

#     def vehicle_attitude_callback(self, msg):
#         # TODO: handle NED->ENU transformation
#         self.vehicle_attitude[0] = msg.q[0]
#         self.vehicle_attitude[1] = msg.q[1]
#         self.vehicle_attitude[2] = -msg.q[2]
#         self.vehicle_attitude[3] = -msg.q[3]

#     def vehicle_local_position_callback(self, msg):
#         path_clearing_timeout = (
#             self.get_parameter("path_clearing_timeout")
#             .get_parameter_value()
#             .double_value
#         )
#         if path_clearing_timeout >= 0 and (
#             (Clock().now().nanoseconds / 1e9 - self.last_local_pos_update)
#             > path_clearing_timeout
#         ):
#             self.vehicle_path_msg.poses.clear()
#         self.last_local_pos_update = Clock().now().nanoseconds / 1e9

#         # TODO: handle NED->ENU transformation
#         self.vehicle_local_position[0] = msg.x
#         self.vehicle_local_position[1] = -msg.y
#         self.vehicle_local_position[2] = -msg.z
#         self.vehicle_local_velocity[0] = msg.vx
#         self.vehicle_local_velocity[1] = -msg.vy
#         self.vehicle_local_velocity[2] = -msg.vz

#     def trajectory_setpoint_callback(self, msg):
#         self.setpoint_position[0] = msg.position[0]
#         self.setpoint_position[1] = -msg.position[1]
#         self.setpoint_position[2] = -msg.position[2]

#     def create_arrow_marker(self, id, tail, vector):
#         msg = Marker()
#         msg.action = Marker.ADD
#         msg.header.frame_id = "map"
#         # msg.header.stamp = Clock().now().nanoseconds / 1000
#         msg.ns = "arrow"
#         msg.id = id
#         msg.type = Marker.ARROW
#         msg.scale.x = 0.1
#         msg.scale.y = 0.2
#         msg.scale.z = 0.0
#         msg.color.r = 0.5
#         msg.color.g = 0.5
#         msg.color.b = 0.0
#         msg.color.a = 1.0
#         dt = 0.3
#         tail_point = Point()
#         tail_point.x = tail[0]
#         tail_point.y = tail[1]
#         tail_point.z = tail[2]
#         head_point = Point()
#         head_point.x = tail[0] + dt * vector[0]
#         head_point.y = tail[1] + dt * vector[1]
#         head_point.z = tail[2] + dt * vector[2]
#         msg.points = [tail_point, head_point]
#         return msg

#     def append_vehicle_path(self, msg):
#         self.vehicle_path_msg.poses.append(msg)
#         if len(self.vehicle_path_msg.poses) > self.trail_size:
#             del self.vehicle_path_msg.poses[0]

#     def append_setpoint_path(self, msg):
#         self.setpoint_path_msg.poses.append(msg)
#         if len(self.setpoint_path_msg.poses) > self.trail_size:
#             del self.setpoint_path_msg.poses[0]

#     def cmdloop_callback(self):
#         vehicle_pose_msg = vector2PoseMsg(
#             "map", self.vehicle_local_position, self.vehicle_attitude
#         )
#         self.vehicle_pose_pub.publish(vehicle_pose_msg)

#         # Publish time history of the vehicle path
#         self.vehicle_path_msg.header = vehicle_pose_msg.header
#         self.append_vehicle_path(vehicle_pose_msg)
#         self.vehicle_path_pub.publish(self.vehicle_path_msg)

#         # Publish time history of the vehicle path
#         setpoint_pose_msg = vector2PoseMsg("map", self.setpoint_position, self.vehicle_attitude)
#         self.setpoint_path_msg.header = setpoint_pose_msg.header
#         self.append_setpoint_path(setpoint_pose_msg)
#         self.setpoint_path_pub.publish(self.setpoint_path_msg)

#         # Publish arrow markers for velocity
#         velocity_msg = self.create_arrow_marker(1, self.vehicle_local_position, self.vehicle_local_velocity)
#         self.vehicle_vel_pub.publish(velocity_msg)


# def main(args=None):
#     rclpy.init(args=args)

#     px4_visualizer = PX4Visualizer()

#     rclpy.spin(px4_visualizer)

#     px4_visualizer.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python

"""
PX4 Off-board velocity control in ROS2. The controller publishes attitude
set points (attitude and Z-body thrust) to PX4 attitude controller. 
Tested on PX4 main (~v1.15) and Ros2 Foxy (ubuntu 20.04)
Author: Tareq Alqutami (tareqaziz2010@gmail.com)
Dec 2023
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, Vector3Stamped
from rcl_interfaces.msg import SetParametersResult
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleCommand
from px4_msgs.msg import VehicleStatus, VehicleAttitude, VehicleLocalPosition, HoverThrustEstimate

from px4_offboard_ros2.PIDn import PIDn
from px4_offboard_ros2.px4_transforms import px4_to_ros_local_frame, px4_to_ros_orientation, ros_to_px4_orientation, ros_to_px4_body_vector
from px4_offboard_ros2.common import thrust_to_attitude, TrapezoidalProfile

class OffboardVelControl(Node):
    """
    PX4 off-board velocity control that runs on top of PX4 attitude control
    """
    def __init__(self):
        super().__init__('vel_control_py')

        self.offboard_control_mode = ['attitude']
        self.engage_px4_offboard_mode = False
        self.hover_thrust = np.array([0.0, 0.0, 0.5]) # in inertial frame
        self.tol = 0.003 # consider vel below this value as zero
        
        # tracking variables
        self.att = np.array([1.0, 0.0, 0.0, 0.0]) # w,x,y,z
        self.att_sp = np.array([1.0, 0.0, 0.0, 0.0]) # w,x,y,z
        self.att_rpy = np.zeros(3) # in degrees
        self.att_sp_rpy = np.zeros(3) # in degrees
        self.position = np.zeros(3)
        self.vel_sp = np.zeros(3)
        self.vel_sp_t = np.zeros(3) # smoothed setpoint following trapezoidal profile
        self.vel = np.zeros(3)
        self.acc = np.zeros(3)
        self.max_acc = 2.0 # max allowed acc using trapezoidal velocity profile
        self.use_trapz_profile = True
        self.px4_vehicle_state = VehicleStatus()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.hover_thrust_estimate = 0.5 # using PX4 estimator
        self.control_effort = np.zeros(3)
        self.p_effort = np.zeros(3)
        self.i_effort = np.zeros(3)
        self.d_effort = np.zeros(3)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # px4 publishers
        self.px4_offboard_mode_pub = self.create_publisher(
            OffboardControlMode,'/fmu/in/offboard_control_mode', qos_profile)
        self.px4_att_sp_pub = self.create_publisher(
            VehicleAttitudeSetpoint,'/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.px4_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # px4 subscribers
        self.px4_status_sub = self.create_subscription(
            VehicleStatus,'/fmu/out/vehicle_status', self.px4_status_callback, qos_profile)
        self.px4_att_sub = self.create_subscription(
            VehicleAttitude,'/fmu/out/vehicle_attitude', self.px4_attitude_callback, qos_profile)
        self.px4_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.px4_local_position_callback, qos_profile)
        self.px4_local_position_sub = self.create_subscription(
            HoverThrustEstimate, '/fmu/out/hover_thrust_estimate',
            self.px4_hover_thrust_callback, qos_profile)

        # setpoint subscribers
        self.target_twist_sub = self.create_subscription(
            Twist,'/target_twist', self.target_twist_callback,10)
        self.target_att_sub = self.create_subscription(
            Vector3Stamped,'/target_att_rpy', self.target_att_callback,10)

        # setpoints and state publishers
        self.att_sp_rpy_pub = self.create_publisher(
            Vector3Stamped,'/att_sp_rpy', qos_profile)
        self.att_rpy_pub = self.create_publisher(
            Vector3Stamped,'/att_rpy', qos_profile)
        self.vel_pub = self.create_publisher(
            Vector3Stamped,'/linear_vel', qos_profile)
        self.vel_sp_pub = self.create_publisher(
            Vector3Stamped,'/linear_vel_sp', qos_profile)
        self.vel_sp_t_pub = self.create_publisher(
            Vector3Stamped,'/linear_vel_trapz_sp', qos_profile)
        self.position_pub = self.create_publisher(
            Vector3Stamped,'/position', qos_profile)
        self.control_effort_pub = self.create_publisher(
            Vector3Stamped,'/control_effort', qos_profile)
        self.p_effort_pub = self.create_publisher(
            Vector3Stamped,'/p_effort', qos_profile)
        self.i_effort_pub = self.create_publisher(
            Vector3Stamped,'/i_effort', qos_profile)
        self.d_effort_pub = self.create_publisher(
            Vector3Stamped,'/d_effort', qos_profile)

        # controller params
        # parameters tuning for tilted_hex
        self.kp = np.array([0.2,0.2,0.3])
        self.ki = np.array([0.001,0.001,0.25])
        self.kd = np.array([0.01,0.01,0.004])
        
        self.out_max = np.array([1.0, 1.0, 1.0]) # PX4 normalized acceleration limits
        self.out_min = np.array([-1.0, -1.0, -1.0]) # PX4 normalized acceleration limits
        self.derivative_on_state = True
        self.use_measured_dot = True

        # declare ros parameters and callback for controller gains and settings
        # dynamic reconfigure can be used to perform online tuning
        self.declare_ros_params()
        self.add_on_set_parameters_callback(self.ros_param_callback)

        # controller
        self.vel_control = PIDn(3, self.kp, self.ki, self.kd,
                                    self.out_max, self.out_min,True,
                                    self.derivative_on_state, self.use_measured_dot)
        # trapezoidal velocity profile
        self.trapz_profile = TrapezoidalProfile(n=3,max_acc=self.max_acc)

        control_period = 0.02  # 0.02 seconds = 50hz
        self.prev_time = self.get_clock().now().nanoseconds
        self.control_loop_timer = self.create_timer(control_period,self.control_loop)


    def px4_status_callback(self, msg: VehicleStatus) -> None:
        """ Callback for PX4 vehicle status"""
        if self.nav_state != msg.nav_state:
            if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info("changed to offboard flight mode")
            else:
                self.get_logger().info(f"changed mode to {msg.nav_state}",throttle_duration_sec=3)

        self.nav_state = msg.nav_state
        self.px4_vehicle_state = msg

    def px4_attitude_callback(self,msg:VehicleAttitude) -> None:
        """ Callback for PX4 attitude. 
        PX4 attitude is in body FRD (left-handed) while ROS is is ENU reference frame.
        Transform to ROS ENU by converting the sign of y and z values"""
        q_enu = px4_to_ros_orientation(msg.q)
        self.att[0] = q_enu[0] # w
        self.att[1] = q_enu[1] # x
        self.att[2] = q_enu[2] # y
        self.att[3] = q_enu[3] # z        
        ypr_rad_enu = Rotation.from_quat([q_enu[1], q_enu[2], q_enu[3], q_enu[0]]).as_euler('zyx') #scalar last
        rpy =  np.rad2deg(np.array([ypr_rad_enu[2], ypr_rad_enu[1], ypr_rad_enu[0]]))
        self.att_rpy = rpy
       
    def px4_local_position_callback(self, msg:VehicleLocalPosition) -> None:
        """ Callback for PX4 local_position. 
        PX4 attitude is in body FRD (left-handed) while ROS is is ENU reference frame.
        Transform to ROS ENU by converting the sign of y and z values"""
        self.position = px4_to_ros_local_frame(np.array([msg.x, msg.y, msg.z]))
        self.vel = px4_to_ros_local_frame(np.array([msg.vx, msg.vy, msg.vz]))
        self.acc = px4_to_ros_local_frame(np.array([msg.ax, msg.ay, msg.az]))


    def px4_hover_thrust_callback(self,msg:HoverThrustEstimate) -> None:
        """ Callback for PX4 hover thrust estimate. This is only published after takeoff 
        and requires adding the topic in PX4 firmware to dds_topics.yaml"""
        # TODO check variance before accepting new estimates
        self.hover_thrust_estimate = msg.hover_thrust

    def publish_px4_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.px4_command_pub.publish(msg)

    def arm(self) -> None:
        """Send an arm command to the vehicle."""
        self.publish_px4_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self) -> None:
        """Send a disarm command to the vehicle."""
        self.publish_px4_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self) -> None:
        """Switch to offboard mode."""
        # set velocity set-point to zero to avoid jumps
        self.vel_sp = np.zeros(3)
        self.publish_px4_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self) -> None:
        """Switch to land mode."""
        self.publish_px4_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def target_att_callback(self, msg:Vector3Stamped) -> None:
        """ callback to update the target attitude (rpy in degrees) used in attitude control.
        The data received is assumed to be in ENU frame"""

        self.att_sp_rpy[0] = msg.vector.x
        self.att_sp_rpy[1] = msg.vector.y
        self.att_sp_rpy[2] = msg.vector.z
        
        att_sp = Rotation.from_euler(
            'XYZ',[self.att_sp_rpy[0], self.att_sp_rpy[1], self.att_sp_rpy[2]],degrees=True).as_quat()
        self.att_sp[0] = att_sp[3]
        self.att_sp[1] = att_sp[0]
        self.att_sp[2] = att_sp[1]
        self.att_sp[3] = att_sp[2]

    def target_twist_callback(self,msg:Twist) -> None:
        """ callback to update the target twist used in velocity control.
        The data received is assumed to be in ENU frame"""
        self.vel_sp[0] = msg.linear.x
        self.vel_sp[1] = msg.linear.y
        self.vel_sp[2] = msg.linear.z

    def declare_ros_params(self):
        """ Declare ros parameters"""
        self.get_logger().info("Declaring parameters")
        self.declare_parameter("x_kp", self.kp[0])
        self.declare_parameter("y_kp", self.kp[1])
        self.declare_parameter("z_kp", self.kp[2])
        self.declare_parameter("x_ki", self.ki[0])
        self.declare_parameter("y_ki", self.ki[1])
        self.declare_parameter("z_ki", self.ki[2])
        self.declare_parameter("x_kd", self.kd[0])
        self.declare_parameter("y_kd", self.kd[1])
        self.declare_parameter("z_kd", self.kd[2])
        self.declare_parameter("derivative_on_measurement",self.derivative_on_state)
        self.declare_parameter("use_measured_derivative",self.use_measured_dot)
        self.declare_parameter("engage_px4_offboard_mode",False)
        self.declare_parameter('use_trapezoidal_vel_profile',True)
        self.declare_parameter('max_acc',self.max_acc)

    def get_ros_params(self):
        """Update local variables from ros parameters"""
        self.kp = np.array([self.get_parameter("x_kp").value,
                                self.get_parameter("y_kp").value,
                                self.get_parameter("z_kp").value
                                ])
        self.ki = np.array([self.get_parameter("x_ki").value,
                                self.get_parameter("y_ki").value,
                                self.get_parameter("z_ki").value
                                ])
        self.kd = np.array([self.get_parameter("x_kd").value,
                                self.get_parameter("y_kd").value,
                                self.get_parameter("z_kd").value
                                ])
        self.derivative_on_state = self.get_parameter("derivative_on_measurement").value
        self.use_measured_dot = self.get_parameter("use_measured_derivative").value
        self.use_trapz_profile = self.get_parameter('use_trapezoidal_vel_profile').value
        self.max_acc = self.get_parameter('max_acc').value

    def update_control_params(self):
        """Update controllers parameters and reset them"""
        self.vel_control.set_gains(self.kp,self.ki, self.kd)
        self.vel_control.set_derivative_method(self.derivative_on_state, self.use_measured_dot)
        self.vel_control.reset()

    def ros_param_callback(self,params:list):
        """Callback when ros parameters change"""
        for param in params:
            if param.name == 'x_kp':
                self.kp[0] = param.value
                self.get_logger().info(f"updated x_kp to {self.kp[0]}")
            elif param.name == 'x_ki':
                self.ki[0] = param.value
                self.get_logger().info(f"updated x_ki to {self.ki[0]}")
            elif param.name == 'x_kd':
                self.kd[0] = param.value
                self.get_logger().info(f"updated x_kd to {self.kd[0]}")
            elif param.name == 'y_kp':
                self.kp[1] = param.value
                self.get_logger().info(f"updated y_kp to {self.kp[1]}")
            elif param.name == 'y_ki':
                self.ki[1] = param.value
                self.get_logger().info(f"updated y_ki to {self.ki[1]}")
            elif param.name == 'y_kd':
                self.kd[1] = param.value
                self.get_logger().info(f"updated y_kd to {self.kd[1]}")
            elif param.name == 'z_kp':
                self.kp[2] = param.value
                self.get_logger().info(f"updated z_kp to {self.kp[2]}")
            elif param.name == 'z_ki':
                self.ki[2] = param.value
                self.get_logger().info(f"updated z_ki to {self.ki[2]}")
            elif param.name == 'z_kd':
                self.kd[2] = param.value
                self.get_logger().info(f"updated z_kd to {self.kd[2]}")
            elif param.name == 'derivative_on_measurement':
                self.derivative_on_state = param.value
                self.get_logger().info(f"updated derv_on_state to {self.derivative_on_state}")
            elif param.name == 'use_measured_derivative':
                self.use_measured_dot = param.value
                self.get_logger().info(f"updated use_measured_derivative to {self.use_measured_dot}")

            elif param.name == 'engage_px4_offboard_mode':
                self.engage_px4_offboard_mode = param.value
                self.get_logger().info(f"updated engage_px4_offboard_mode to {self.engage_px4_offboard_mode}")
                self.engage_offboard_mode()

            elif param.name == 'use_trapezoidal_vel_profile':
                self.get_logger().info(f"updated use_trapezoidal_vel_profile to {self.use_trapz_profile}")
                if param.value != self.use_trapz_profile:
                    self.use_trapz_profile = param.value
                if self.use_trapz_profile:
                    self.trapz_profile.reset()

            elif param.name == 'max_acc':
                self.get_logger().info(f"updated max_acc to {self.max_acc}")
                self.max_acc = param.value
                self.trapz_profile.set_max_acc(self.max_acc)

        self.update_control_params()
        return SetParametersResult(successful=True)

    def publish_px4_offboard_control_mode(self, modes:list) -> None:
        """publish px4 offboard control mode. 

        Args:
            modes (list): list of modes to turn on
        """
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True if 'position' in modes else False
        offboard_msg.velocity = True if 'velocity' in modes else False
        offboard_msg.acceleration = True if 'acceleration' in modes else False
        offboard_msg.attitude = True if 'attitude' in modes else False
        offboard_msg.body_rate = True if 'body_rate' in modes else False
        self.offboard_control_mode = modes
        self.px4_offboard_mode_pub.publish(offboard_msg)

    def publish_states(self) -> None:
        """Publish states for monitoring, plotting and troubleshooting.
        """
        # timestamp = int(self.get_clock().now().nanoseconds / 1000)

        rpy_msg = Vector3Stamped()
        rpy_msg.vector.x = self.att_sp_rpy[0]
        rpy_msg.vector.y = self.att_sp_rpy[1]
        rpy_msg.vector.z = self.att_sp_rpy[2]
        self.att_sp_rpy_pub.publish(rpy_msg)

        rpy_msg = Vector3Stamped()
        rpy_msg.vector.x = self.att_rpy[0]
        rpy_msg.vector.y = self.att_rpy[1]
        rpy_msg.vector.z = self.att_rpy[2]
        self.att_rpy_pub.publish(rpy_msg)

        vel_msg = Vector3Stamped()
        vel_msg.vector.x = self.vel[0]
        vel_msg.vector.y = self.vel[1]
        vel_msg.vector.z = self.vel[2]
        self.vel_pub.publish(vel_msg)

        pos_msg = Vector3Stamped()
        pos_msg.vector.x = self.position[0]
        pos_msg.vector.y = self.position[1]
        pos_msg.vector.z = self.position[2]
        self.position_pub.publish(pos_msg)

        vel_msg = Vector3Stamped()
        vel_msg.vector.x = self.vel_sp_t[0]
        vel_msg.vector.y = self.vel_sp_t[1]
        vel_msg.vector.z = self.vel_sp_t[2]
        self.vel_sp_pub.publish(vel_msg)

        vel_msg = Vector3Stamped()
        vel_msg.vector.x = self.vel_sp[0]
        vel_msg.vector.y = self.vel_sp[1]
        vel_msg.vector.z = self.vel_sp[2]
        self.vel_sp_t_pub.publish(vel_msg)

        effort_msg = Vector3Stamped()
        effort_msg.vector.x = self.control_effort[0]
        effort_msg.vector.y = self.control_effort[1]
        effort_msg.vector.z = self.control_effort[2]
        self.control_effort_pub.publish(effort_msg)

        effort_msg = Vector3Stamped()
        effort_msg.vector.x = self.p_effort[0]
        effort_msg.vector.y = self.p_effort[1]
        effort_msg.vector.z = self.p_effort[2]
        self.p_effort_pub.publish(effort_msg)

        effort_msg = Vector3Stamped()
        effort_msg.vector.x = self.i_effort[0]
        effort_msg.vector.y = self.i_effort[1]
        effort_msg.vector.z = self.i_effort[2]
        self.i_effort_pub.publish(effort_msg)

        effort_msg = Vector3Stamped()
        effort_msg.vector.x = self.d_effort[0]
        effort_msg.vector.y = self.d_effort[1]
        effort_msg.vector.z = self.d_effort[2]
        self.d_effort_pub.publish(effort_msg)

    def control_loop(self) -> None:
        """Callback function for the timer that runs the control loop at desired rate"""
        # Publish offboard control modes
        self.publish_px4_offboard_control_mode(self.offboard_control_mode)
        # publish states
        self.publish_states()

        # in attitude offboard mode, use our custom controller
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            time_now = self.get_clock().now().nanoseconds
            dt = (time_now - self.prev_time)/1000000000.0
            self.prev_time = time_now

            # PID velocity (vx,vy,vz) control
            # floor low velocity fluctuations
            filtered_vel, filtered_acc = self.vel, self.acc
            filtered_vel[abs(filtered_vel)<self.tol] = 0.0
            filtered_acc[abs(filtered_acc)<self.tol] = 0.0

            # smooth velocity setpoint to follow trapezoidal profile
            if self.use_trapz_profile:
                self.vel_sp_t = self.trapz_profile.update_vel_sp(dt, self.vel_sp, filtered_vel)
                acc_out = self.vel_control.update(dt, filtered_vel, self.vel_sp_t, filtered_acc)
            else:
                acc_out = self.vel_control.update(dt, filtered_vel, self.vel_sp, filtered_acc)

            acc_out = np.clip(acc_out,self.out_min,self.out_max)            
            self.control_effort = acc_out
            p_err, _, _ = self.vel_control.get_current_errors()
            self.p_effort, self.i_effort, self.d_effort = self.vel_control.get_current_efforts()
            self.get_logger().info(f"err={p_err.round(2)}, acc={acc_out.round(2)}",
                                    throttle_duration_sec=2)
        
            # Compute and saturate thrust values
            self.hover_thrust[2] = self.hover_thrust_estimate
            thrust_inertial =  acc_out + self.hover_thrust
            thrust_inertial = thrust_inertial.clip(-1.0,1.0)

            # ========== under-actuated control acc+yaw>>att+thrust ======
            rpy_sp = Rotation.from_quat(
                [self.att_sp[1], self.att_sp[2], self.att_sp[3], self.att_sp[0]]).as_euler('zyx')
            yaw_sp = np.deg2rad(self.att_sp_rpy[2]) # in radians
            #TODO limit horizontal thrust to avoid high tilt angles
            att_sp, thrust = thrust_to_attitude(thrust_inertial, yaw_sp)
            ypr = Rotation.from_quat(
                [att_sp[1], att_sp[2], att_sp[3], att_sp[0]]).as_euler('zyx')
            rpy = np.rad2deg(np.array([ypr[2], ypr[1], ypr[0]]))
            self.get_logger().info(f"att_sp={rpy.round(2)}, thrust={thrust}",
                                    throttle_duration_sec=0.5)
            th_sp = np.zeros(3)
            th_sp[2] = thrust
            

            att_sp_ned = ros_to_px4_orientation(att_sp)
            thrust_body_frd = ros_to_px4_body_vector(th_sp)
            # print(f'Tw: {thrust_inertial},  Tb: {thrust_body_frd}')

            # send AttitudeSetpoint to PX4
            att_sp_msg = VehicleAttitudeSetpoint()
            att_sp_msg.q_d[0] = att_sp_ned[0]
            att_sp_msg.q_d[1] = att_sp_ned[1]
            att_sp_msg.q_d[2] = att_sp_ned[2]
            att_sp_msg.q_d[3] = att_sp_ned[3]
            att_sp_msg.thrust_body[0] = thrust_body_frd [0]
            att_sp_msg.thrust_body[1] = thrust_body_frd [1]
            att_sp_msg.thrust_body[2] = thrust_body_frd [2]
            self.px4_att_sp_pub.publish(att_sp_msg)


def main(args=None):
    """Main function to execute"""
    rclpy.init(args=args)

    offboard_control = OffboardVelControl()

    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
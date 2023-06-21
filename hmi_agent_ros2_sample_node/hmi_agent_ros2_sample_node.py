#!/usr/bin/python3
import signal
import sys

import rclpy
import rclpy.qos
from ck_utilities_ros2_py_node.ckmath import *
from hmi_agent_ros2_sample_node.generated.parameters import ParameterizedNode
from ck_utilities_ros2_py_node.joystick import Joystick
from frc_robot_utilities_ros2_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_ros2_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy
from frc_robot_utilities_ros2_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode
from ck_ros2_msgs_node.msg import HMISignals
from nav_msgs.msg import Odometry
from ck_ros2_base_msgs_node.msg import JoystickStatus

from dataclasses import dataclass
import numpy as np


from std_msgs.msg import String


class LocalNode(ParameterizedNode):
    def __init__(self):
        super().__init__('hmi_agent_ros2_sample_node')
        self.driver_joystick = Joystick(0)
        self.operator_button_box = Joystick(1)
        self.operator_joystick = Joystick(2)
        self.drivetrain_orientation = HMISignals.FIELD_CENTRIC
        self.heading = 0.0

        self.hmi_publisher = self.create_publisher(topic="/HMISignals", msg_type=HMISignals, qos_profile=10)

        self.odometry_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.odometry_subscriber.register_for_updates("odometry/filtered")

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        self.__joystick_subscriber = self.create_subscription(topic="/JoystickStatus", msg_type=JoystickStatus, callback=self.joystick_callback, qos_profile=qos_profile)

        register_for_robot_updates()


    def joystick_callback(self, message: JoystickStatus):
        """
        Joystick callback function. This runs everytime a new joystick status message is received.
        """

        #DO NOT REMOVE THIS CHECK!!!!!!!!!! DID YOU LEARN NOTHING FROM 2022?!
        if robot_status.get_mode() != RobotMode.TELEOP:
            return

        Joystick.update(message)

        hmi_update_message = HMISignals()
        hmi_update_message.drivetrain_brake = True


        #######################################################################
        ###                         DRIVER CONTROLS                         ###
        #######################################################################
        invert_axis_fwd_back = -1 if self.Parameters.drive_fwd_back_axis_inverted else 1
        invert_axis_left_right = -1 if self.Parameters.drive_left_right_axis_inverted else 1

        fwd_back_value = self.driver_joystick.getFilteredAxis(self.Parameters.drive_fwd_back_axis_id, self.Parameters.drive_axis_deadband)
        hmi_update_message.drivetrain_fwd_back = invert_axis_fwd_back * fwd_back_value

        left_right_value = self.driver_joystick.getFilteredAxis(self.Parameters.drive_left_right_axis_id, self.Parameters.drive_axis_deadband)
        hmi_update_message.drivetrain_left_right = invert_axis_left_right * left_right_value

        x = hmi_update_message.drivetrain_fwd_back
        y = hmi_update_message.drivetrain_left_right

        invert_axis_z = -1 if self.Parameters.drive_z_axis_inverted else 1
        z = invert_axis_z * self.driver_joystick.getFilteredAxis(self.Parameters.drive_z_axis_id, self.Parameters.drive_z_axis_deadband, self.Parameters.drive_z_axis_min_value_after_deadband)

        r = hypotenuse(x, y)
        theta = polar_angle_rad(x, y)

        z = np.sign(z) * pow(z, 2)
        active_theta = theta
        if r > self.Parameters.drive_axis_deadband:
            active_theta = theta

        hmi_update_message.drivetrain_swerve_direction = active_theta

        # Scale the drive power based on current arm position.
        # arm_status_message = self.arm_subscriber.get()
        # limited_forward_velocity, limited_angular_rotation = limit_drive_power(arm_status_message, r, z)
        limited_forward_velocity, limited_angular_rotation = r,z

        hmi_update_message.drivetrain_swerve_percent_fwd_vel = limited_forward_velocity
        hmi_update_message.drivetrain_swerve_percent_angular_rot = limited_angular_rotation

        # Swap between field centric and robot oriented drive.
        if self.driver_joystick.getButton(self.Parameters.robot_orient_button_id):
            self.drivetrain_orientation = HMISignals.ROBOT_ORIENTED
        elif self.driver_joystick.getButton(self.Parameters.field_centric_button_id):
            self.drivetrain_orientation = HMISignals.FIELD_CENTRIC

        hmi_update_message.drivetrain_orientation = self.drivetrain_orientation

        hmi_update_message.drivetrain_xmode = False
        if self.driver_joystick.getButton(self.Parameters.robot_xmode_driver_id): # or self.operator_joystick.getButton(self.Parameters.robot_xmode_id):
            hmi_update_message.drivetrain_xmode = True

        if self.driver_joystick.getRisingEdgeButton(self.Parameters.reset_odometry_button_id):
            reset_robot_pose(robot_status.get_alliance())

        self.hmi_publisher.publish(hmi_update_message)


def signal_handler(sig, frame):
    sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    node = LocalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/python3
import signal
import sys

import rclpy
from hmi_agent_ros2_sample_node.generated.parameters import ParameterizedNode
from ck_utilities_ros2_py_node.joystick import Joystick

from dataclasses import dataclass
import numpy as np


from std_msgs.msg import String


class LocalNode(ParameterizedNode):
    def __init__(self):
        super().__init__('hmi_agent_ros2_sample_node')
        js = Joystick(0)

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
#!/usr/bin/env python3

import numpy as np
#import heapq as hq
#import cv2
import time

#from typing import Dict, Tuple, List, Callable, Union
#from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AStarNode(Node):

    def __init__(self):
        super().__init__('a_star_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.R = 0.033
        self.L = 0.160

    def a_star(self):
        return [(50,50),(100,50),(0,50)]
    def drive_turtlebot(self,wheel_rpms_path):
        """Run the keyboard control node"""
        
        self.msg = """
        placeholder
        """

        self.get_logger().info(self.msg)
        velocity_message = Twist()
        for UL_rpm, UR_rpm in wheel_rpms_path:
            UL = UL_rpm * 2 * np.pi / 60
            UR = UR_rpm * 2 * np.pi / 60

            linear_x = (self.R / 2) * (UL + UR)
            angular_z = (self.R / self.L )* (UR - UL)

            velocity_message.linear.x = linear_x
            velocity_message.angular.z = angular_z
            self.cmd_vel_pub.publish(velocity_message)
            self.get_logger().info(f"Left rpm: {UL_rpm}. Right rpm: {UR_rpm}")
            time.sleep(10)  # adjust for dt simulation


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    wheel_rpms_path = node.a_star()
    node.drive_turtlebot(wheel_rpms_path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
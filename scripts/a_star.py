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
        self.r = 0.033 #wheel radius
        self.L = 0.287 #distance between wheels
        self.R = 0.220 #robot radius
        self.sim_update_time = 1.0
        self.rpms = [50,100]
        self.actions = [
            (0,self.rpms[0]),
            (self.rpms[0],0),
            (self.rpms[0],self.rpms[0]),
            (0,self.rpms[1]),
            (self.rpms[1],0),
            (self.rpms[1],self.rpms[0]),
            (self.rpms[0],self.rpms[1]),
            (self.rpms[1],self.rpms[1])
        ]


    def get_pred_traj_pt(self,rpms,x0,y0,theta0,T):
        # Convert to rad/s
        wl = rpms[0] * 2 * np.pi / 60
        wr = rpms[1] * 2 * np.pi / 60

        v = (self.r / 2) * (wr + wl)
        omega = (self.r / self.L) * (wr - wl)

        if abs(omega) < 1e-6:
            xf = x0 + v * T * np.cos(theta0)
            yf = y0 + v * T * np.sin(theta0)
            thetaf = theta0
        else:
            R_icc = v / omega
            dtheta = omega * T
            xf = x0 + R_icc * (np.sin(theta0 + dtheta) - np.sin(theta0))
            yf = y0 - R_icc * (np.cos(theta0 + dtheta) - np.cos(theta0))
            thetaf = theta0 + dtheta

        return xf, yf, thetaf
    
    def get_pred_update_time(self,x0,y0,theta0,xf,yf,thetaf):
        # Convert to rad/s
        time_range = np.linspace(0.1,5.0,100)
        min_dist_2 = (xf-x0)**2 + (yf-y0)**2 + (thetaf-theta0)**2
        for rpms in self.actions:
            for T in time_range:
                x,y,theta = self.get_pred_traj_pt(rpms,x0,y0,theta0,T)
                if ((xf-x)**2 + (yf-y)**2 + (thetaf-theta)**2)<min_dist_2:
                    min_dist_2 = (xf-x)**2 + (yf-y)**2 + (thetaf-theta)**2
                    self.sim_update_time = T
    
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

            linear_x = (self.r / 2) * (UL + UR)
            angular_z = (self.r / self.L )* (UR - UL)

            velocity_message.linear.x = linear_x
            velocity_message.angular.z = angular_z
            self.cmd_vel_pub.publish(velocity_message)
            self.get_logger().info(f"Left rpm: {UL_rpm}. Right rpm: {UR_rpm}")
            time.sleep(self.sim_update_time)  # adjust for dt simulation
    
    def get_vels_from_path(self, path, trajectory_map):
        if path is not None:
            rpm_list = []
            have_runtime = False
            for node in path:
                if node in trajectory_map:
                    trajectory = trajectory_map[node]
                    x0, y0, theta0 = trajectory[0]
                    xf, yf, thetaf = trajectory[-1]
                    if not have_runtime:
                        self.get_pred_update_time(x0,y0,np.radians(theta0),xf,yf,np.radians(thetaf))
                        have_runtime = True

                    wheel_rpms = self.actions[0]
                    min_dist_2 = (xf-x0)**2 + (yf-y0)**2
                    for action in self.actions:
                        x,y,_ = self.get_pred_traj_pt(action,x0,y0,np.radians(theta0),self.sim_update_time)
                        if ((xf-x)**2 + (yf-y)**2)<min_dist_2:
                            min_dist_2 = (xf-x)**2 + (yf-y)**2
                            wheel_rpms = action
                    rpm_list.append(wheel_rpms)
            return rpm_list
        return None
                    





def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    path, _, trajectory_map, _ = node.a_star()
    wheel_rpms_path = node.get_vels_from_path(path,trajectory_map)
    node.drive_turtlebot(wheel_rpms_path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
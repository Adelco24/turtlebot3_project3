#!/usr/bin/env python3

import numpy as np
import heapq as hq
#import cv2
import time

from typing import Dict, Tuple, List, Union
#from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AStarNode(Node):

    def __init__(self):
        super().__init__('a_star_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_x = 5400
        self.map_y = 3000
        self.r = 0.033 #wheel radius
        self.L = 0.287 #distance between wheels
        self.R = 0.220 #robot radius
        self.sim_update_time = 1.0
        self.rpms = [25,50]
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

        v = (self.r * 1000 / 2) * (wr + wl)
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
    
    def drive_turtlebot(self,wheel_rpms_path):
        
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
                    self.get_logger().info(f"{x0},{y0},{theta0}")
                    theta0 = np.radians(theta0)
                    xf, yf, thetaf = trajectory[-1]
                    thetaf = np.radians(thetaf)
                    if not have_runtime:
                        self.get_pred_update_time(x0,y0,theta0,xf,yf,thetaf)
                        have_runtime = True

                    wheel_rpms = self.actions[0]
                    min_dist_2 = (xf-x0)**2 + (yf-y0)**2
                    for action in self.actions:
                        x,y,_ = self.get_pred_traj_pt(action,x0,y0,theta0,self.sim_update_time)
                        if ((xf-x)**2 + (yf-y)**2)<min_dist_2:
                            min_dist_2 = (xf-x)**2 + (yf-y)**2
                            wheel_rpms = action
                    self.get_logger().info(f"associated rpms: ({wheel_rpms[0]},{wheel_rpms[1]}])")
                    rpm_list.append(wheel_rpms)
            rpm_list.append((0,0))
            return rpm_list
        return None
    def get_clearances(self,user_clearance):

        clearance = 5 + self.R + user_clearance
        # Define clearances
        clearances = {

                "Clearance 1": [
                    lambda x, y: x >= 1000-clearance,
                    lambda x, y: x <= 1100+clearance,
                    lambda x, y: y >= 0,
                    lambda x, y: y <= 2000+clearance
                ],

                "Clearance 2": [
                    lambda x, y: x >= 2100-clearance,
                    lambda x, y: x <= 2200+clearance,
                    lambda x, y: y >= 1000-clearance,
                    lambda x, y: y <= 3000
                ],

                "Clearance 3": [
                    lambda x, y: x >= 3200-clearance,
                    lambda x, y: x <= 3300+clearance,
                    lambda x, y: y >= 0,
                    lambda x, y: y <= 1000+clearance
                ],

                "Clearance 4": [
                    lambda x, y: x >= 3200-clearance,
                    lambda x, y: x <= 3300+clearance,
                    lambda x, y: y >= 2000-clearance,
                    lambda x, y: y <= 3000
                ],

                "Clearance 5": [
                    lambda x, y: x >= 4300-clearance,
                    lambda x, y: x <= 4400+clearance,
                    lambda x, y: y >= 0,
                    lambda x, y: y <= 2000+clearance
                ],

                "Clearance 6": [
                    #lambda x, y: x >= 0,
                    #lambda x, y: x <= 10+clearance,
                    lambda x, y: x >= -2,
                    lambda x, y: x <= -1,
                    lambda x, y: y >= 0,
                    lambda x, y: y <= self.map_y
                ],

                "Clearance 7": [
                    lambda x, y: x >= self.map_x-10-clearance,
                    lambda x, y: x <= self.map_x,
                    lambda x, y: y >= 0,
                    lambda x, y: y <= self.map_y
                ],

                "Clearance 8": [
                    lambda x, y: x >= 0,
                    lambda x, y: x <= self.map_x,
                    lambda x, y: y >= 0,
                    lambda x, y: y <= 10+clearance
                ],

                "Clearance 9": [
                    lambda x, y: x >= 0,
                    lambda x, y: x <= self.map_x,
                    lambda x, y: y >= self.map_y-10-clearance,
                    lambda x, y: y <= self.map_y
                ],

        }
        return clearances
    
    # get pose after action
    def get_pose(self,x,y,theta_deg,u_l,u_r):


        # Convert RPM to rad/s
        ul_rad = (u_l * 2 * np.pi) / 60
        ur_rad = (u_r * 2 * np.pi) / 60

        # Compute linear and angular velocity
        v = self.r*1000/2 * (ul_rad + ur_rad) #mm/s
        omega = self.r/self.L * (ur_rad - ul_rad) #rad/s

        
        # list for discrete trajectory
        trajectory = []
        trajectory.append((x,y,theta_deg))
        DT = 1.0
        # time step such that 1 mm moved at every step
        time_step = 0.01
        n_steps = max(1,int(DT / time_step))
        theta = np.radians(theta_deg)

        for _ in range(n_steps):
            dx = v * np.cos(theta) * time_step
            dy = v * np.sin(theta) * time_step
            dtheta = omega * time_step
            x += dx
            y += dy
            theta += dtheta
            trajectory.append((x,y,np.degrees(theta)%360))
        # print('trajectory creation time: ', traj_time.stop())
        # print('vel: ', v, ' time step: ', time_step, ' number steps: ', n_steps)
        

        return trajectory
    

    # checks point for validity: Not within obstacles or clearance regions
    def is_valid(self,x: Union[float,int], y: Union[float,int], clearances: Dict) -> bool:

        # If location is within obstacle constraints
        if any(all(constraint(x, y) for constraint in constraints) for constraints in clearances.values()):

            # Return invalid
            return False
        
        # If location is not within obstacle constraints
        else:
            
            # Return valid
            return True

    # A* algorithm definition
    def a_star(self,start: Tuple[float, float, int], goal: Tuple[float, float], clearances: Dict, actions: List, map_size: Tuple[int, int] = (5400, 3000)) -> Union[List, None]:
        trajectory_map = {}
        # Mark start time
        start_time = time.time()
        threshold = self.R * 1000       # distance threshold to goal to consider success - radius of robot?
        early_stop_on = False
        early_stop = 100      # number of nodes to explore before quitting algorithm
        duplicate_distance_threshold = 100 # if within 5mm of other config, consider as duplicate

        # Define function for computing heuristic
        def heuristic(node: Tuple[float, float, int], goal: Tuple[float, float]) -> float:

            euclidean_dist = np.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) 

            # dx = goal[0] - start[0]
            # dy = goal[1] - start[1]
            # direct_to_goal_heading = np.degrees(np.arctan2(dy,dx))
            # heading_difference = abs((direct_to_goal_heading - node[2] +180) % 360 - 180)
            # print(node)
            # print(goal)
            # print(euclidean_dist, direct_to_goal_heading, heading_difference)
            return euclidean_dist #+ heading_difference

        # Define function for backtracking
        def backtrack(goal: Tuple[float, float, int], parent_map: Dict) -> List:
            path = []
            while goal in parent_map:
                path.append(goal)
                goal = parent_map[goal]
            path.reverse()
            return path

        # Define function for getting node neighbors
        def get_neighbors(node: Tuple[float, float, float], visited: np.ndarray, clearances: Dict, actions: List, map_size: Tuple[int, int] = (self.map_x, self.map_y)) -> List:

            x, y, theta = node 
            neighbors = []

            # for every action set generate new node
            #action_i = 0
            for ul, ur in actions:
                
                # get changes
                trajectory = self.get_pose(x, y, theta, u_l=ul, u_r=ur)
                final_x, final_y, final_theta = trajectory[-1] 

                new_theta_30_index = int(round(final_theta / 30)) % 12
                int_x, int_y = int(round(final_x)/duplicate_distance_threshold), int(round(final_y)/duplicate_distance_threshold)

                
                flag = 0
                for point in trajectory:
                    xi, yi, _ = point
                    if not 0 <= xi < map_size[0] or not 0 <= yi < map_size[1] or not self.is_valid(xi,yi, clearances):
                        flag = 1 
                        break


                if not flag and visited[int_y, int_x, new_theta_30_index] == 0:
                    visited[int_y, int_x, new_theta_30_index] = 1
                    neighbors.append((final_x, final_y, final_theta))
                    trajectory_map[(final_x, final_y, final_theta)] = trajectory
                    trajectory_list.append(trajectory)


                #action_i += 1

            return neighbors

        # Create configuration map for visited nodes
        discretized_height = int(map_size[1]/duplicate_distance_threshold)
        discretized_width = int(map_size[0]/duplicate_distance_threshold)
        #print('Discretized Width, Height: ', discretized_width, discretized_height)
        visited = np.zeros((discretized_height,discretized_width, 12), dtype=np.uint8)

        # Initialize open list
        open_list = []
        hq.heappush(open_list, (0, start))

        # Initialize dictionary for storing parent information
        parent_map = {}

        # Initialize dictionary for storing cost information
        cost_map = {start: 0}

        # Initialize list for storing closed nodes and explored nodes
        closed_nodes = []
        explored_nodes = []
        trajectory_list = []

        # Loop until queue is empty
        try:
            while open_list:

                current_node_info = hq.heappop(open_list)
                current_node: Tuple[float, float, int] = current_node_info[1]

                # Add node to closed list
                closed_nodes.append(current_node)

                # Record explored node for visualization
                explored_nodes.append(current_node)
                #print(current_node)

                # Determine if solution is found
                if np.sqrt((current_node[0] - goal[0]) ** 2 + (current_node[1] - goal[1]) ** 2) <= threshold:

                    # Mark end time
                    end_time = time.time()

                    print(f"Time to search: {end_time - start_time:.4f} seconds")

                    # Backtrack to find path from goal
                    return backtrack(current_node, parent_map), explored_nodes, trajectory_map, trajectory_list
                
                # Loop through neighbors
                for neighbor in get_neighbors(current_node, visited, clearances, actions):

                    # cost for action taken -- all actions valued at 1
                    new_cost = cost_map[current_node] + 1  

                    # checks if node cost can be reduce and updates it
                    if neighbor not in cost_map or new_cost < cost_map[neighbor]:
                        cost_map[neighbor] = new_cost
                        total_cost = new_cost + heuristic(neighbor, goal)
                        hq.heappush(open_list, (total_cost, neighbor))
                        parent_map[neighbor] = current_node

                # early stop to give up -- for testing
                if len(explored_nodes) % 100 == 0:
                    self.get_logger().info(f"{len(explored_nodes)} explored...")
                    self.get_logger().info(f"{current_node[0]},{current_node[1]},{current_node[2]}")
                if early_stop_on:
                    if len(explored_nodes) >= early_stop:
                        break
        except KeyboardInterrupt:
            print('Force Quit')



        return None, explored_nodes, trajectory_map, trajectory_list  # Return None if no path is found



def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    start = (0,2000,0)
    goal = (1800,2400)
    clearances = node.get_clearances(1)
    action_set = node.actions

    path, _, trajectory_map, _ = node.a_star(start, goal, clearances, action_set)
    wheel_rpms_path = node.get_vels_from_path(path,trajectory_map)
    node.drive_turtlebot(wheel_rpms_path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
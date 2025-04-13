#!/usr/bin/env python3
import cv2
import numpy as np
import heapq as hq
#import cv2
import time
import sys

from typing import Dict, Tuple, List, Union
#from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class AStarNode(Node):

    def __init__(self):
        super().__init__('a_star_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_x = 5400
        self.map_y = 3000
        self.r = 0.033 #wheel radius
        self.L = 0.287 #distance between wheels
        self.R = 0.220 #robot radius
        #self.sim_update_time = 1.0
        self.current_pose = None
        self.odom_sub = self.create_subscription(Odometry, '/odom',self.odom_callback,10)
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

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion to yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        self.current_pose = (x, y, theta)

    def set_rpms_and_actions(self,rpm1,rpm2):
        self.rpms = [rpm1,rpm2]
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

    def get_action_from_points(self,x0,y0,theta0,xf,yf,thetaf):
        time_range = np.linspace(0.1,5.0,100)
        min_dist_2 = (xf-x0)**2 + (yf-y0)**2 + (thetaf-theta0)**2
        best_action = self.actions[0]
        best_t = 1.0
        for rpms in self.actions:
            for T in time_range:
                x,y,theta = self.rpm_to_point(rpms,x0,y0,theta0,T)
                if ((xf-x)**2 + (yf-y)**2 + (thetaf-theta)**2)<min_dist_2:
                    min_dist_2 = (xf-x)**2 + (yf-y)**2 + (thetaf-theta)**2
                    best_t = T
                    best_action = rpms
        self.get_logger().info(f"sim_time:{best_t}")
        return best_t, best_action

    def rpm_to_point(self,rpms,x0,y0,theta0,T):
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
    
    
    '''def drive_turtlebot(self,wheel_rpms_path,runtimes):
        
        self.msg = """
        placeholder
        """

        self.get_logger().info(self.msg)
        velocity_message = Twist()
        for i,rpm_vals in enumerate(wheel_rpms_path):
            update_time = runtimes[i]
            UL = rpm_vals[0]* 2 * np.pi / 60
            UR = rpm_vals[1] * 2 * np.pi / 60

            linear_x = (self.r / 2) * (UL + UR)
            angular_z = (self.r / self.L )* (UR - UL)

            velocity_message.linear.x = linear_x
            velocity_message.angular.z = angular_z
            start_time = self.get_clock().now().nanoseconds
            duration_ns = int(update_time * 1.4e9)
            while self.get_clock().now().nanoseconds - start_time < duration_ns:
                self.cmd_vel_pub.publish(velocity_message)
                rclpy.spin_once(self,timeout_sec=0.05)
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            rclpy.spin_once(self,timeout_sec=0.1)
            self.get_logger().info(f"Left rpm: {rpm_vals[0]}. Right rpm: {rpm_vals[1]}")
    
    '''
    def get_vels_from_path(self, path, trajectory_map):
        if path is not None:
            rpm_list = []
            wp_list = []
            runtimes = []
            trajectory = []

            for node in path:
                if node in trajectory_map:
                    trajectory = trajectory_map[node]
                    x0, y0, theta0 = trajectory[0]
                    wp_list.append(trajectory[0])
                    self.get_logger().info(f"{x0},{y0},{theta0}")
                    theta0 = np.radians(theta0)
                    xf, yf, thetaf = trajectory[-1]
                    thetaf = np.radians(thetaf)

                    runtime,wheel_rpms = self.get_action_from_points(x0,y0,theta0,xf,yf,thetaf)

                    self.get_logger().info(f"associated rpms: ({wheel_rpms[0]},{wheel_rpms[1]}])")
                    rpm_list.append(wheel_rpms)
                    runtimes.append(runtime)
            wp_list.append(trajectory[-1])

            return rpm_list,wp_list,runtimes
        return None
    
    def glob_frame_to_sim_frame(self,wp):
        return wp[0]/1000.0,(wp[1]/1000.0)-1.5
    
    def sim_frame_to_glob_frame(self,wp):
        return int(wp[0]*1000),int((wp[1]+1.5)*1000)


    def drive_turtlebot(self, waypoints: List[Tuple[float, float]], rpm_pairs: List[Tuple[int, int]], tolerance=100):
        self.get_logger().info("Following waypoints with fixed RPMs...")
        assert len(waypoints) == len(rpm_pairs), "One RPM pair per waypoint required"

        # PID controller for small heading correction if robot drifts off course
        angular_pid = PID(kp=2.5, ki=0.0, kd=0.1)

        for idx, (wp, rpms) in enumerate(zip(waypoints, rpm_pairs)):

            rpm_l = rpms[0]
            rpm_r = rpms[1]
            x_goal = wp[0]
            y_goal = wp[1]

            self.get_logger().info(f"Waypoint {idx+1}: ({x_goal:.2f}, {y_goal:.2f}), RPMs = ({rpm_l}, {rpm_r})")

            # Convert wheel RPMs to velocities
            wl = rpm_l * 2 * np.pi / 60  # rad/s
            wr = rpm_r * 2 * np.pi / 60  # rad/s

            base_v = (self.r / 2) * (wr + wl)
            base_omega = (self.r / self.L) * (wr - wl)

            reached = False
            while rclpy.ok() and not reached:
                rclpy.spin_once(self, timeout_sec=0.01)

                if self.current_pose is None:
                    continue

                x, y, theta = self.current_pose
                x, y = self.sim_frame_to_glob_frame((x,y))
                

                # Compute distance to goal
                dx = x_goal - x
                dy = y_goal - y
                distance = np.hypot(dx, dy)

                if distance < tolerance:
                    self.get_logger().info(f"Reached waypoint {idx+1}")
                    break

                # Desired heading (straight to goal)
                target_theta = np.arctan2(dy, dx)
                heading_error = target_theta - theta
                heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))  # normalize

                # Start with base velocities
                linear_x = base_v
                angular_z = base_omega

                # Apply a little correction only if heading error is significant
                if abs(heading_error) > np.radians(10):
                    angular_z += angular_pid.compute(heading_error, 0.01)

                # Publish command
                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = angular_z
                self.cmd_vel_pub.publish(twist)

            # Stop between waypoints
            self.cmd_vel_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Completed all waypoints.")

    def get_clearances(self,user_clearance):

        clearance = 5 + (self.R*1000) + user_clearance
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
                    #lambda x, y: x >= self.map_x-10-clearance,
                    #lambda x, y: x <= self.map_x,
                    lambda x, y: x >= self.map_x+1,
                    lambda x, y: x <= self.map_x+2,
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
    cli_args = sys.argv[1:]
    if len(cli_args) != 5 and len(cli_args) != 0:
        print("Usage: ros2 run turtlebot3_project3 a_star.py <xg> <yg> <rpm1> <rpm2> <clearance>")
        return

    if len(cli_args) == 5:
        # Parse CLI arguments
        xg = float(cli_args[0])
        yg = float(cli_args[1])
        rpm1 = int(cli_args[2])
        rpm2 = int(cli_args[3])
        clearance = int(cli_args[4])
    else:
        xg = 5.2
        yg = 0.0
        rpm1 = 25
        rpm2 = 50
        clearance = 200

    x0 = 0.5
    y0 = 1.0
    theta0 = 0
    node = AStarNode()
    x0,y0 = node.sim_frame_to_glob_frame((x0,y0))
    xg,yg = node.sim_frame_to_glob_frame((xg,yg))
    start = (x0,y0,theta0) #corresponds to map from other code
    goal = (xg,yg) #corresponds to map from other code
    clearances = node.get_clearances(clearance)
    node.set_rpms_and_actions(rpm1,rpm2)
    action_set = node.actions

    path, _, trajectory_map, _ = node.a_star(start, goal, clearances, action_set)
    wheel_rpms_path,wp_list,runtimes = node.get_vels_from_path(path,trajectory_map)
    node.get_logger().info(f"Wheel RPM path length: {len(wheel_rpms_path)}")
    node.get_logger().info(f"Waypoint path length: {len(wp_list)}")
    node.drive_turtlebot(wp_list[1:],wheel_rpms_path)
    frame = np.ones((node.map_y, node.map_x, 3), dtype=np.uint8) * 255

    # Generate meshgrid of all (x, y) coordinates
    x_grid, y_grid = np.meshgrid(np.arange(node.map_x), np.arange(node.map_y))

    # Compute clearance area and display as gray
    for conditions in clearances.values():
        mask = np.ones_like(x_grid, dtype=bool)
        for cond in conditions:
            mask &= cond(x_grid, y_grid)
        frame[mask] = (150, 150, 150)

    # Flip to match coordinate system
    frame = cv2.flip(frame, 0)

    # Draw final start and goal points
    cv2.circle(frame, (int(start[0]), int(node.map_y - start[1])), 50, (0, 0, 255), -1)  # Red (start)
    cv2.circle(frame, (int(goal[0]), int(node.map_y - goal[1])), 50, (0, 255, 0), -1)  # Green (goal)
    for wp in wp_list:
        cv2.circle(frame,(int(wp[0]), int(node.map_y - wp[1])), 25, (0, 0, 255), -1)  # Red
    scale_frame = cv2.resize(frame, (int(node.map_x * .2), int(node.map_y * .2)), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("A* Path Visualization", scale_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
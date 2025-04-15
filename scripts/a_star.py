#!/usr/bin/env python3

# Standard and third-party libraries
import cv2
import numpy as np
import heapq as hq
import time
import sys
import math

# Type hinting and ROS2 libraries
from typing import Dict, Tuple, List, Union
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Simple PID controller for angular correction
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    # PID computation method
    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# Main ROS2 node class for A* planning and robot control
class AStarNode(Node):

    def __init__(self):
        super().__init__('a_star_node')

        # Set up publisher to /cmd_vel and subscriber to /odom
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Robot and map configuration
        self.map_x = 5400  # map width in mm
        self.map_y = 3000  # map height in mm
        self.r = 0.033     # wheel radius in meters
        self.L = 0.287     # wheel base (distance between wheels)
        self.R = 0.220     # robot radius for clearance
        self.threshold = self.R * 1000  # goal threshold (radius of robot)


        self.current_pose = None  # Will be updated from /odom

        # Define default RPM set and the corresponding 8 motion primitives
        self.rpms = [25, 50]
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
        
    # Callback function for Odometry updates from /odom topic
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion orientation to yaw angle
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Store the current pose as (x, y, theta)
        self.current_pose = (x, y, theta)

    # Allows external RPM settings (used for path generation)
    def set_rpms_and_actions(self, rpm1, rpm2):
        self.rpms = [rpm1, rpm2]

        # Re-generate the action set with updated RPMs
        self.actions = [
            (0, rpm1),
            (rpm1, 0),
            (rpm1, rpm1),
            (0, rpm2),
            (rpm2, 0),
            (rpm2, rpm1),
            (rpm1, rpm2),
            (rpm2, rpm2)
        ]


        # Finds the best (action, time) combo that ends closest to the trajectory segment
    def get_action_from_points(self, x0, y0, theta0, xf, yf, thetaf):
        time_range = np.linspace(0.1, 5.0, 100)
        min_dist_2 = (xf - x0)**2 + (yf - y0)**2 + (thetaf - theta0)**2
        best_action = self.actions[0]
        best_t = 1.0

        # Try each motion primitive and simulate result
        for rpms in self.actions:
            for T in time_range:
                x, y, theta = self.rpm_to_point(rpms, x0, y0, theta0, T)
                dist_2 = (xf - x)**2 + (yf - y)**2 + (thetaf - theta)**2
                if dist_2 < min_dist_2:
                    min_dist_2 = dist_2
                    best_t = T
                    best_action = rpms

        return best_t, best_action


        # Simulates robot motion given RPMs and initial pose for time T
    def rpm_to_point(self, rpms, x0, y0, theta0, T):
        wl = rpms[0] * 2 * np.pi / 60
        wr = rpms[1] * 2 * np.pi / 60

        # Linear and angular velocities
        v = (self.r * 1000 / 2) * (wr + wl)       # mm/s
        omega = (self.r / self.L) * (wr - wl)     # rad/s

        if abs(omega) < 1e-6:
            # Straight line motion
            xf = x0 + v * T * np.cos(theta0)
            yf = y0 + v * T * np.sin(theta0)
            thetaf = theta0
        else:
            # Circular arc motion
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
        # Extract RPMs and durations for each segment from the planned path
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
                    wp_list.append(trajectory[0])  # Store initial pose of each trajectory

                    #self.get_logger().info(f"{x0},{y0},{theta0}")
                    theta0 = np.radians(theta0)
                    xf, yf, thetaf = trajectory[-1]
                    thetaf = np.radians(thetaf)

                    # Find best matching action and time
                    runtime, wheel_rpms = self.get_action_from_points(x0, y0, theta0, xf, yf, thetaf)
                    #self.get_logger().info(f"associated rpms: ({wheel_rpms[0]},{wheel_rpms[1]}])")

                    rpm_list.append(wheel_rpms)
                    runtimes.append(runtime)

            # Append the last goal pose to waypoints
            wp_list.append(trajectory[-1])
            return rpm_list, wp_list, runtimes

        return None

    # Convert from global map frame (mm) to sim (meters) with offset
    def glob_frame_to_sim_frame(self, wp):
        return wp[0] / 1000.0, (wp[1] / 1000.0) - 1.5

    # Convert from sim frame (meters) back to global map (mm)
    def sim_frame_to_glob_frame(self, wp):
        return int(wp[0] * 1000), int((wp[1] + 1.5) * 1000)



        # Drives robot through a sequence of waypoints using specified RPMs per segment
    def drive_turtlebot(self, waypoints: List[Tuple[float, float]], rpm_pairs: List[Tuple[int, int]], tolerance=100):
        self.get_logger().info("Following waypoints with fixed RPMs...")
        assert len(waypoints) == len(rpm_pairs), "One RPM pair per waypoint required"

        # Create angular PID controller
        angular_pid = PID(kp=2.5, ki=0.0, kd=0.1)

        # Loop through waypoints and corresponding RPMs
        for idx, (wp, rpms) in enumerate(zip(waypoints, rpm_pairs)):
            rpm_l = rpms[0]
            rpm_r = rpms[1]
            x_goal = wp[0]
            y_goal = wp[1]

            xsim,ysim = self.glob_frame_to_sim_frame(wp)
            self.get_logger().info(f"Waypoint {idx+1}: ({xsim:.2f}, {ysim:.2f}), RPMs = ({rpm_l}, {rpm_r})")

            # Convert wheel RPMs to linear and angular velocities
            wl = rpm_l * 2 * np.pi / 60
            wr = rpm_r * 2 * np.pi / 60
            base_v = (self.r / 2) * (wr + wl)
            base_omega = (self.r / self.L) * (wr - wl)

            # Loop until robot reaches the waypoint
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)

                if self.current_pose is None:
                    continue

                x, y, theta = self.current_pose
                x, y = self.sim_frame_to_glob_frame((x, y))  # convert to map frame

                dx = x_goal - x
                dy = y_goal - y
                distance = np.hypot(dx, dy)

                # Stop condition if close enough
                if distance < tolerance:
                    self.get_logger().info(f"Reached waypoint {idx+1} of {len(waypoints)}")
                    break

                # Calculate heading error
                target_theta = np.arctan2(dy, dx)
                heading_error = np.arctan2(np.sin(target_theta - theta), np.cos(target_theta - theta))

                # Use base velocities, apply small angular correction if needed
                linear_x = base_v
                angular_z = base_omega
                if abs(heading_error) > np.radians(10):
                    angular_z += angular_pid.compute(heading_error, 0.01)

                # Publish twist message
                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = angular_z
                self.cmd_vel_pub.publish(twist)

            # Stop after each waypoint
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
    
    # Simulate the trajectory from a given pose and wheel RPMs over 1 second
    def get_pose(self, x, y, theta_deg, u_l, u_r):
        # Convert RPMs to rad/s
        ul_rad = (u_l * 2 * np.pi) / 60
        ur_rad = (u_r * 2 * np.pi) / 60

        # Compute linear and angular velocities
        v = self.r * 1000 / 2 * (ul_rad + ur_rad)  # mm/s
        omega = self.r / self.L * (ur_rad - ul_rad)  # rad/s

        trajectory = [(x, y, theta_deg)]  # store initial pose
        DT = 1.0
        time_step = 0.01
        n_steps = max(1, int(DT / time_step))
        theta = np.radians(theta_deg)

        # Simulate motion over time
        for _ in range(n_steps):
            dx = v * np.cos(theta) * time_step
            dy = v * np.sin(theta) * time_step
            dtheta = omega * time_step

            x += dx
            y += dy
            theta += dtheta

            trajectory.append((x, y, np.degrees(theta) % 360))

        return trajectory

    

    # checks point for validity: Not within obstacles or clearance regions
    def is_valid(self,x: Union[float,int], y: Union[float,int], clearances: Dict) -> bool:

        # If location is within obstacle constraints
        if any(all(constraint(x, y) for constraint in constraints) for constraints in clearances.values()):

            # Return invalid
            return False
    
        # Return valid
        return True

    # A* Search algorithm with custom motion primitives and map clearance checking
    def a_star(self, start: Tuple[float, float, int], goal: Tuple[float, float],
               clearances: Dict, actions: List, map_size: Tuple[int, int] = (5400, 3000)) -> Union[List, None]:
        
        trajectory_map = {}
        start_time = time.time()
        early_stop_on = False
        early_stop = 100
        duplicate_distance_threshold = 100

        # Heuristic = Euclidean distance
        def heuristic(node: Tuple[float, float, int], goal: Tuple[float, float]) -> float:
            return np.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)

        # Backtrack from goal to start using parent map
        def backtrack(goal: Tuple[float, float, int], parent_map: Dict) -> List:
            path = []
            while goal in parent_map:
                path.append(goal)
                goal = parent_map[goal]
            path.reverse()
            return path

        # Generate neighbor nodes from motion primitives
        def get_neighbors(node: Tuple[float, float, float], visited: np.ndarray,
                          clearances: Dict, actions: List,
                          map_size: Tuple[int, int] = (self.map_x, self.map_y)) -> List:

            x, y, theta = node
            neighbors = []

            for ul, ur in actions:
                trajectory = self.get_pose(x, y, theta, u_l=ul, u_r=ur)
                final_x, final_y, final_theta = trajectory[-1]

                # Discretize position and angle for visited check
                new_theta_30_index = int(round(final_theta / 30)) % 12
                int_x = int(round(final_x) / duplicate_distance_threshold)
                int_y = int(round(final_y) / duplicate_distance_threshold)

                # Check if the full trajectory is valid
                flag = 0
                for point in trajectory:
                    xi, yi, _ = point
                    if not 0 <= xi < map_size[0] or not 0 <= yi < map_size[1] or not self.is_valid(xi, yi, clearances):
                        flag = 1
                        break

                if not flag and visited[int_y, int_x, new_theta_30_index] == 0:
                    visited[int_y, int_x, new_theta_30_index] = 1
                    neighbors.append((final_x, final_y, final_theta))
                    trajectory_map[(final_x, final_y, final_theta)] = trajectory
                    trajectory_list.append(trajectory)

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
                if np.sqrt((current_node[0] - goal[0]) ** 2 + (current_node[1] - goal[1]) ** 2) <= self.threshold:

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

                if len(explored_nodes) % 100 == 0:
                    self.get_logger().info(f"{len(explored_nodes)} explored...")
                    #self.get_logger().info(f"{current_node[0]},{current_node[1]},{current_node[2]}")
                
                # early stop to give up -- for testing
                if early_stop_on:
                    if len(explored_nodes) >= early_stop:
                        break
        except KeyboardInterrupt:
            print('Force Quit')



        return None, explored_nodes, trajectory_map, trajectory_list  # Return None if no path is found



def main(args=None):
    rclpy.init(args=args)
    cli_args = sys.argv[1:]

    # Expecting exactly 5 CLI arguments or none (for defaults)
    if len(cli_args) != 2 and len(cli_args) != 0:
        print("Not right number of args.\nUsage: ros2 run turtlebot3_project3 a_star.py <xg> <yg>\nOR\nros2 run turtlebot3_project3 a_star.py\nto use defaults")
        return

    # Parse arguments or use defaults
    if len(cli_args) == 2:
        xg = float(cli_args[0])
        yg = float(cli_args[1])
        #rpm1 = int(cli_args[2])
        #rpm2 = int(cli_args[3])
        #clearance = float(cli_args[4])
    else:
        xg = 5.2
        yg = 0.0
    rpm1 = 25
    rpm2 = 50
    clearance = 50 #mm

    # Initial pose in simulation frame, then convert to map
    node = AStarNode()
    starttime = time.time()
    while rclpy.ok() and node.current_pose is None and (time.time()-starttime < 5.0):
        rclpy.spin_once(node,timeout_sec=0.1)
    
    if node.current_pose is None:
        print("Cannot get starting position of robot and therefore cannot start. Try rerunning.")
        return

    x0, y0, theta0 = node.current_pose
    theta0 = int(round(np.degrees(theta0)/30.0)*30)

    x0, y0 = node.sim_frame_to_glob_frame((x0, y0))
    xg, yg = node.sim_frame_to_glob_frame((xg, yg))
    start = (x0, y0, theta0)
    goal = (xg, yg)
    if ((xg-x0)**2)+((yg-y0)**2)<(node.threshold**2):
        print("Already within goal radius!")
        return


    # Setup clearance and actions
    clearances = node.get_clearances(clearance)
    if not node.is_valid(x0,y0,clearances):
        print("Start position is not valid, try another.")
        return
    if not node.is_valid(xg,yg,clearances):
        print("Goal position is not valid, try another.")
        return
    node.set_rpms_and_actions(rpm1, rpm2)
    action_set = node.actions

    # Run A* path planner
    path, _, trajectory_map, _ = node.a_star(start, goal, clearances, action_set)
    if path is not None:
        node.get_logger().info("A* found path!")
    else:
        node.get_logger().info("A* did not find path. Likely that the robot cannot go forward without hit obstacle/clearance boundary.")
        return

    # Convert path to RPMs and waypoint poses
    wheel_rpms_path,wp_list,_ = node.get_vels_from_path(path, trajectory_map)

    # Drive the robot through waypoints (excluding the start)
    node.drive_turtlebot(wp_list[1:], wheel_rpms_path)

    # --- Visualization ---

    # Create white canvas for map
    frame = np.ones((node.map_y, node.map_x, 3), dtype=np.uint8) * 255

    # Generate coordinate grid for visualization
    x_grid, y_grid = np.meshgrid(np.arange(node.map_x), np.arange(node.map_y))

    # Mark clearance zones as gray
    for conditions in clearances.values():
        mask = np.ones_like(x_grid, dtype=bool)
        for cond in conditions:
            mask &= cond(x_grid, y_grid)
        frame[mask] = (150, 150, 150)

    # Flip image to match plotting coordinates
    frame = cv2.flip(frame, 0)

    # Draw start (red) and goal (green)
    cv2.circle(frame, (int(start[0]), int(node.map_y - start[1])), 50, (0, 0, 255), -1)
    cv2.circle(frame, (int(goal[0]), int(node.map_y - goal[1])), 50, (0, 255, 0), -1)

    # Draw all intermediate waypoints (red)
    for wp in wp_list:
        cv2.circle(frame, (int(wp[0]), int(node.map_y - wp[1])), 25, (0, 0, 255), -1)

    # Resize and show
    scale_frame = cv2.resize(frame, (int(node.map_x * .2), int(node.map_y * .2)), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("A* Path Visualization", scale_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

# Run the node
if __name__ == '__main__':
    main()

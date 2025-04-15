import numpy as np
import heapq as hq
import cv2
import time

from typing import Dict, Tuple, List, Union

import matplotlib.pyplot as plt



# class for timekeeping
class Timer():

    # initialize and start timer 
    def __init__(self):
        
        self.start_time = None
        self.end_time = None
        self.start()

    # start timer
    def start(self):
        self.start_time = time.perf_counter()

    # stop timer
    def stop(self):
        self.end_time = time.perf_counter()
        elapsed = self._get_time()
        return elapsed

    # get string of time elapsed mins:seconds
    def _get_time(self):
        time_elapsed = self.end_time - self.start_time
        return self._convert_times(time_elapsed)

    # convert seconds to min/secs
    def _convert_times(self, time):
        mins = time//60
        rem = time%60
        return f'{mins}:{rem}'

#import matplotlib.pyplot as plt


# link to turtlbot3 waffle specs:
# https://www.robotis.us/turtlebot-3-waffle-pi-rpi4-4gb-us/?srsltid=AfmBOooiyGxavAjLuccnF81d3mxxE8KWS26Qk2dWRsSpROtfKBzxHcfu
WR = 33         # Robot Wheel Radius in mm
RR = 220        # Robot Radius in mm
WB = 287        # Wheel Base in mm
DT = 1.0

# map dims
map_x = 5400
map_y = 3000
scale = .2


# prompts user for desired clearance, generates clearances dict
def get_clearance() -> int:

    base_clearance = 5 + RR  # needs to include radius of robot so robot does not touch obstacles

    # Loop until values is valid
    while True:

        # Gather user input
        user_input = input("\nClearance (in mm) of the robot [> 0]\nEnter: ").strip()

        # default for quick testing
        if user_input == '':
            user_input = '1'

        try:
            
            # Assign input clearance
            user_clearance = int(user_input.strip())
            
            # If clearance is within bounds
            if user_clearance > 0:
                break
                                
            # Inform user of invalid values
            else:
                print("Invalid input. Please ensure clearance is a positive integer.")
        
        # Inform user of invalid input format
        except ValueError:
            print("Invalid input. Please enter clearance as an integer.")

    clearance = base_clearance + user_clearance
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
                lambda x, y: x >= 0,
                lambda x, y: x <= 10+clearance,
                lambda x, y: y >= 0,
                lambda x, y: y <= map_y
            ],

            "Clearance 7": [
                lambda x, y: x >= map_x-10-clearance,
                lambda x, y: x <= map_x,
                lambda x, y: y >= 0,
                lambda x, y: y <= map_y
            ],

            "Clearance 8": [
                lambda x, y: x >= 0,
                lambda x, y: x <= map_x,
                lambda x, y: y >= 0,
                lambda x, y: y <= 10+clearance
            ],

            "Clearance 9": [
                lambda x, y: x >= 0,
                lambda x, y: x <= map_x,
                lambda x, y: y >= map_y-10-clearance,
                lambda x, y: y <= map_y
            ],

    }

    return clearances, user_clearance

# get map obstacle defintions
def get_obstacles():

    # Define obstacles
    obstacles = {

            "Obstacle 1": [
                lambda x, y: x >= 1000,
                lambda x, y: x <= 1100,
                lambda x, y: y >= 0,
                lambda x, y: y <= 2000
            ],

            "Obstacle 2": [
                lambda x, y: x >= 2100,
                lambda x, y: x <= 2200,
                lambda x, y: y >= 1000,
                lambda x, y: y <= 3000
            ],

            "Obstacle 3": [
                lambda x, y: x >= 3200,
                lambda x, y: x <= 3300,
                lambda x, y: y >= 0,
                lambda x, y: y <= 1000
            ],

            "Obstacle 4": [
                lambda x, y: x >= 3200,
                lambda x, y: x <= 3300,
                lambda x, y: y >= 2000,
                lambda x, y: y <= 3000
            ],

            "Obstacle 5": [
                lambda x, y: x >= 4300,
                lambda x, y: x <= 4400,
                lambda x, y: y >= 0,
                lambda x, y: y <= 2000
            ],

    }

    return obstacles

# prompt user for start location
def get_start(clearances: Dict) -> Tuple:

    # Loop until pose is valid
    while True:

        # Gather user input

        user_input = input(f"\nStart pose separated by commas in the format of: x, y, θ\n- x: 1 - {map_x}\n- y: 1 - {map_y}\n- θ: Intervals of 30\nEnter: ").strip()

        # default for quick testing
        if user_input == '':
            user_input = '400,400,0'


        if user_input is None:
            return ("Please enter a pose.")
        
        # Break user input into pose coordinates
        parts = user_input.split(",")
        
        # Ensure all three pose coordinates are present
        if len(parts) == 3:
            try:
                
                # Assign input coordinates
                x = float(parts[0].strip())
                y = float(parts[1].strip())
                theta = int(parts[2].strip())
                
                # If coordinates are within bounds
                if 1 <= x <= map_x and 1 <= y <= map_y and theta in [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360]:
                    
                    # Convert positional coordinates to 1 - n scale
                    x = x - 1
                    y = y - 1
                    
                    
                    # If location is not an obstacle, return pose
                    if is_valid(x + 1, y + 1, clearances):
                        return (x, y, (theta % 360))
                    
                    # Inform user of invalid location
                    else:
                        print("Sorry, this point is within the obstacle space. Try again.")
                
                # Inform user of invalid location
                else:
                    print("Invalid input. Please ensure both x and y are within the bounds of the space and theta is in [-60,-30,0,30,60].")
            
            # Inform user of invalid input format
            except ValueError:
                print("Invalid input. Please enter integers for x, y, and theta.")
        
        # Inform user of invalid input dimension
        else:
            print("Invalid input. Please enter exactly three integers separated by a comma.")

# prompt user for goal location
def get_goal(clearances: Dict) -> Tuple:

    # Loop until pose is valid
    while True:

        # Gather user input
        user_input = input(f"\nGoal pose separated by commas in the format of: x, y\n- x: 1 - {map_x}\n- y: 1 - {map_y}\nEnter: ").strip()

        # default for quick testing
        if user_input == '':
            user_input = '5000,500'

        if user_input is None:
            return ("Please enter a pose.")
        
        # Break user input into pose coordinates
        parts = user_input.split(",")
        
        # Ensure all two pose coordinates are present
        if len(parts) == 2:
            try:
                
                # Assign input coordinates
                x = float(parts[0].strip())
                y = float(parts[1].strip())
                # If coordinates are within bounds
                if 1 <= x <= map_x and 1 <= y <= map_y:
                    
                    # Convert positional coordinates to 1 - n scale
                    x = x - 1
                    y = y - 1
                    
                    # If location is not an obstacle, return pose
                    if is_valid(x + 1, y + 1, clearances):
                        return (x, y)
                    
                    # Inform user of invalid location
                    else:
                        print("Sorry, this point is within the obstacle space. Try again.")
                
                # Inform user of invalid location
                else:
                    print("Invalid input. Please ensure both x and y are within the bounds of the space.")
            
            # Inform user of invalid input format
            except ValueError:
                print("Invalid input. Please enter integers for x and y.")
        
        # Inform user of invalid input dimension
        else:
            print("Invalid input. Please enter exactly two integers separated by a comma.")

# prompt user for wheel rpms of robot
def get_wheel_rpms() -> Tuple:
    # Loop until values is valid
    while True:

        # Gather user input
        user_input = input("\nTwo positive wheel RPM values separated by commas in the format of: rpm1, rpm2\nEnter: ").strip()

        # default for quick testing
        if user_input == '':
            user_input = '50,100'

        if user_input is None:
            return ("Please enter a pair of wheel RPMs.")
        
        # Break user input into each rpm
        parts = user_input.split(",")
        
        # Ensure all values are present
        if len(parts) == 2:
            try:
                
                # Assign input rpms
                rpm1 = float(parts[0].strip())
                rpm2 = float(parts[1].strip())
                
                # If rpms are within bounds
                if rpm1 > 0 and rpm2 > 0:
                    return (rpm1,rpm2)
                                  
                # Inform user of invalid values
                else:
                    print("Invalid input. Please ensure both wheel RPM values are positive.")
            
            # Inform user of invalid input format
            except ValueError:
                print("Invalid input. Please enter numbers for both wheel RPM values.")
        
        # Inform user of invalid input dimension
        else:
            print("Invalid input. Please enter exactly two numbers separated by a comma.")

# define action set given rpms
def get_action_set(rpms):

    actions = [
        (0,rpms[0]),
        (rpms[0],0),
        (rpms[0],rpms[0]),
        (0,rpms[1]),
        (rpms[1],0),
        (rpms[1],rpms[0]),
        (rpms[0],rpms[1]),
        (rpms[1],rpms[1])
    ]
        
    return actions

# checks point for validity: Not within obstacles or clearance regions
def is_valid(x: Union[float,int], y: Union[float,int], clearances: Dict) -> bool:

    # If location is within obstacle constraints
    if any(all(constraint(x, y) for constraint in constraints) for constraints in clearances.values()):

        # Return invalid
        return False
    
    # If location is not within obstacle constraints
    else:
        
        # Return valid
        return True

# visualization of map with robot movements
def visualize_environment(obstacles, clearances, start, goal, path, explored_nodes, trajectory_map, trajectory_list):
    # Create a blank 5400x3000 white frame
    frame = np.ones((map_y, map_x, 3), dtype=np.uint8) * 255

    # Generate meshgrid of all (x, y) coordinates
    x_grid, y_grid = np.meshgrid(np.arange(map_x), np.arange(map_y))

    # Compute clearance area and display as gray
    for conditions in clearances.values():
        mask = np.ones_like(x_grid, dtype=bool)
        for cond in conditions:
            mask &= cond(x_grid, y_grid)
        frame[mask] = (150, 150, 150)

    # Compute obstacle area and display as black
    obstacle_mask = np.zeros_like(x_grid, dtype=bool)
    for conditions in obstacles.values():
        temp_mask = np.ones_like(x_grid, dtype=bool)
        for cond in conditions:
            temp_mask &= cond(x_grid, y_grid)
        obstacle_mask |= temp_mask
    frame[np.where(obstacle_mask)] = (0, 0, 0)

    # Flip to match coordinate system
    frame = cv2.flip(frame, 0)

    # Draw final start and goal points
    cv2.circle(frame, (int(start[0]), int(map_y - start[1])), 50, (0, 0, 255), -1)  # Red (start)
    cv2.circle(frame, (int(goal[0]), int(map_y - goal[1])), RR, (0, 255, 0), -1)  # Green (goal)
    cv2.circle(frame,(int(goal[0]), int(map_y - goal[1])), 15, (0, 0, 0), -1)  # Green (goal)

    print(('Plotting Explored Nodes...'))
    # Draw explored node trajectories
    for i, trajectory in enumerate(trajectory_list):
        for j in range(len(trajectory) - 1):
            x1, y1, _ = trajectory[j]
            x2, y2, _ = trajectory[j + 1]

            y1_flipped = map_y - y1
            y2_flipped = map_y - y2

            cv2.line(frame, (int(x1), int(y1_flipped)), (int(x2), int(y2_flipped)),
                        (0, 200, 200), 5)
        #else:
            #print("not there")

        # Update display every 100 steps
        if i % 2 == 0:
            scale_frame = cv2.resize(frame, (int(map_x * scale), int(map_y * scale)), interpolation=cv2.INTER_LINEAR)
            cv2.imshow("A* Path Visualization", scale_frame)
            cv2.waitKey(1)

    print('Finished')
    # Draw path node trajectories
    if path is not None:

        print('Plotting Path...')
        for node in path:
            if node in trajectory_map:
                trajectory = trajectory_map[node]
                for j in range(len(trajectory) - 1):
                    x1, y1, _ = trajectory[j]
                    x2, y2, _ = trajectory[j + 1]

                    y1_flipped = map_y - y1
                    y2_flipped = map_y - y2

                    cv2.line(frame, (int(x1), int(y1_flipped)), (int(x2), int(y2_flipped)),
                             (255, 0, 0), 5)  # Blue path

        # Draw final start and goal points
        cv2.circle(frame, (int(start[0]), int(map_y - start[1])), 50, (0, 0, 255), -1)  # Red (start)
        cv2.circle(frame, (int(goal[0]), int(map_y - goal[1])), RR, (0, 255, 0), -1)  # Green (goal)
        cv2.circle(frame,(int(goal[0]), int(map_y - goal[1])), 15, (0, 0, 0), -1)  # Green (goal)


        scale_frame = cv2.resize(frame, (int(map_x * scale), int(map_y * scale)), interpolation=cv2.INTER_LINEAR)

        # Final visualization
        print('Finished')
        cv2.imshow("A* Path Visualization", scale_frame)

    scale_frame = cv2.resize(frame, (int(map_x * scale), int(map_y * scale)), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("A* Path Visualization", scale_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# get pose after action
def get_pose(x,y,theta_deg,u_l,u_r):


    # Convert RPM to rad/s
    ul_rad = (u_l * 2 * np.pi) / 60
    ur_rad = (u_r * 2 * np.pi) / 60

    # Compute linear and angular velocity
    v = WR/2 * (ul_rad + ur_rad) #mm/s
    omega = WR/WB * (ur_rad - ul_rad) #rad/s

    
    # list for discrete trajectory
    trajectory = []
    trajectory.append((x,y,theta_deg))

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
    
# A* algorithm definition
def a_star(start: Tuple[float, float, int], goal: Tuple[float, float], clearances: Dict, actions: List, map_size: Tuple[int, int] = (5400, 3000)) -> Union[List, None]:
    trajectory_map = {}
    # Mark start time
    start_time = time.time()
    threshold = RR       # distance threshold to goal to consider success - radius of robot?
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
    def get_neighbors(node: Tuple[float, float, float], visited: np.ndarray, clearances: Dict, actions: List, map_size: Tuple[int, int] = (map_x, map_y)) -> List:

        x, y, theta = node 
        neighbors = []

        # for every action set generate new node
        #action_i = 0
        for ul, ur in actions:
            
            # get changes
            trajectory = get_pose(x, y, theta, u_l=ul, u_r=ur)
            final_x, final_y, final_theta = trajectory[-1] 

            new_theta_30_index = int(round(final_theta / 30)) % 12
            int_x, int_y = int(round(final_x)/duplicate_distance_threshold), int(round(final_y)/duplicate_distance_threshold)
            
            flag = 0
            for point in trajectory:
                xi, yi, _ = point
                if not 0 <= xi < map_size[0] or not 0 <= yi < map_size[1] or not is_valid(xi,yi, clearances):
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
        time_per_nodes = Timer()
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

            # logging
            if len(explored_nodes)%1000 == 0:
                print('Nodes Explored: ', len(explored_nodes), ' -- ', time_per_nodes.stop())
                time_per_nodes.start()


            # early stop to give up -- for testing
            if early_stop_on:
                if len(explored_nodes) >= early_stop:
                    break
    except KeyboardInterrupt:
        print('Force Quit')



    return None, explored_nodes, trajectory_map, trajectory_list  # Return None if no path is found
      

# verify action set and trajectory generation
def action_set_verification():

    start = (0,0,0)

    plt.figure()

    ul = 50
    ur = 50
    trajectory = get_pose(start[0], start[1], start[1], u_l=ul, u_r=ur)
    xs, ys, _ = zip(*trajectory)
    plt.plot(xs, ys, label=[f'{ul},{ur}'], color='blue')

    ul = 0
    ur = 50
    trajectory = get_pose(start[0], start[1], start[1], u_l=ul, u_r=ur)
    xs, ys, _ = zip(*trajectory)
    plt.plot(xs, ys, label=[f'{ul},{ur}'], color='red')

    ul = 50
    ur = 0
    trajectory = get_pose(start[0], start[1], start[1], u_l=ul, u_r=ur)
    xs, ys, _ = zip(*trajectory)
    plt.plot(xs, ys, label=[f'{ul},{ur}'], color='purple')

    ul = 100
    ur = 100
    trajectory = get_pose(start[0], start[1], start[1], u_l=ul, u_r=ur)
    xs, ys, _ = zip(*trajectory)
    plt.plot(xs, ys, label=[f'{ul},{ur}'], color='green')

    ul = 0
    ur = 100
    trajectory = get_pose(start[0], start[1], start[1], u_l=ul, u_r=ur)
    xs, ys, _ = zip(*trajectory)
    plt.plot(xs, ys, label=[f'{ul},{ur}'], color='yellow')

    ul = 100
    ur = 0
    trajectory = get_pose(start[0], start[1], start[1], u_l=ul, u_r=ur)
    xs, ys, _ = zip(*trajectory)
    plt.plot(xs, ys, label=[f'{ul},{ur}'], color='orange')

    plt.title('Action Set Visualization [ul, ur]')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.legend()
    plt.grid()
    plt.show()

# verify A* algortihm implementation and visualization
def a_star_verification():

    obstacles = get_obstacles()
    clearances, user_clearance = get_clearance()
    start = get_start(clearances)
    goal = get_goal(clearances)
    rpms = get_wheel_rpms()
    action_set = get_action_set(rpms)

    printstart = f"({start[0]+1}, {start[1]+1}, {start[2]})"
    printgoal = f"({goal[0]+1}, {goal[1]+1})"

    
    print('======================================================')
    print('Parameters:')
    print('- Clearance: ', user_clearance)
    print('- Start: ', printstart)
    print('- Goal: ', printgoal)
    print('- RPMS: ', rpms)
    print('- Action Time Step: ', DT)
    print('Robot Attributes: ')
    print('- Wheel Radius: ', WR, ' mm')
    print('- Chassis Radius: ', RR, ' mm')
    print('- Wheel Base: ', WB, ' mm')
    print('======================================================')

    wait = input('Press Enter to Begin Algorithm')
    print('Running A* Algorithm with given parameters...')

    algo_solve_time = Timer()
    path, explored_nodes, trajectory_map, trajectory_list = a_star(start, goal, clearances, action_set)
    print('Finished -- total algorithm run time: ', algo_solve_time.stop())
    #print('Trajectories: ', len(trajectory_list))

    if path is None:
        print('NO PATH FOUND: -- Explored: ', len(explored_nodes), ' Nodes')

    # Visualize the environment
    visualize_environment(obstacles, clearances, start, goal, path, explored_nodes, trajectory_map, trajectory_list)




if __name__ == '__main__':
    # action_set_verification()
    a_star_verification()


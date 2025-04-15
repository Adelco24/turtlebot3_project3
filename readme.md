# ENPM 661: Path Planning for Autonomous Robots
### Instructions for Project3- Phase2
Riley Albert,
Directory ID: ralbert8
UID: 120985195

Adam Del Colliano,
Directory ID: adelcoll
UID: 115846982

Joseph Shaheen,
Directory ID: jshaheen
UID: 116534321

## ACCESS TO OUR VIDEOS
https://drive.google.com/drive/folders/1iDzi45sgHjE6V_lIN9iSaKgw5NmZLQ8m?usp=sharing

## Setup

Create a workspace

```sh
mkdir -p project3_ws/src
cd ~/project3_ws/src
```

import the repository

```sh
unzip turtlebot3_project3.zip
```
or

```sh
git clone https://github.com/Adelco24/turtlebot3_project3.git
```


Source ROS (Enable ROS commands)

```sh
source /opt/ros/galactic/setup.bash
```

Build the workspace

```sh
cd ~/project3_ws
colcon build --packages-select turtlebot3_project3
```


Source ROS (Package will be identified)

```sh
source install/setup.bash
```

Install dependencies
Most packages in this project are native on Python and/or ROS installs, but opencv, numpy, and matplotlib require an extra install, and can be done with the following command:
```sh
sudo apt-get install python3-opencv && sudo apt install python3-numpy && sudo apt-get install python3-matplotlib

```

## Part 1 (No ROS)
```sh
python src/turtlebot3_project3/scripts/exp_script.py
```
All inputs for part 1 are outlined in the terminal via user input.


## Part 2 (ROS): Launch Environment

```sh
ros2 launch turtlebot3_project3 competition_world.launch.py

```
to spawn at x = 0.5 meters, y = 1.0 meters,
or

```sh
ros2 launch turtlebot3_project3 competition_world.launch.py x_pose:=0.5 y_pose:=1.0

```
where the values of x_pose and y_pose, correspond to the values of the spawn location of turtlebot and can be chosen by the user. Spawn within the bounds of the maze.

You should see the turtlebot3 along with the maze in gazebo.

![gazebo](gazebo.png)


## Run a_star node script

In a second terminal, you can run the a_star node script using

```sh
ros2 run turtlebot3_project3 a_star.py
```
This runs with the default values of:
(x,y)_goal = (5.2,0.0) meters
left wheel rpm = 25 rpm
right wheel rpm = 50 rpm
clearance = 0.2 meters

For custom goal locations, run the following instead, with your location for (x_goal,y_goal), within the bounds of the maze:
```sh
ros2 run turtlebot3_project3 a_star.py x_goal y_goal
```
This will drive the robot through the maze, and at the end, will display a map showing all A* waypoints and the space that the robot was allowed to drive in (its clearance area).


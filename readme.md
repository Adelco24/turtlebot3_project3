# ENPM 661: Path Planning for Autonomous Robots
### Instructions for Project3- Phase2

## Map Dimensions

All dimensions are in meters.

![map](map.png)

## Setup

Create a workpace

```sh
mkdir -p project3_ws/src
cd ~/project3_ws/src
```

import the repository

```sh
unzip turtlebot3_project3.zip
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

## Launch Environment


```sh
ros2 launch turtlebot3_project3 competition_world.launch.py x_pose:=0.5 y_pose:=1.0

```
where the values of x_pose and y_pose, corresponding to the values of the spawn location of turtlebot, are chosen by the user.

You should see the turtlebot3 along with the maze in gazebo.

![gazebo](gazebo.png)


## Run python script


You can run the script using

```sh
ros2 run turtlebot3_project3 a_star.py
```
This runs with the default values of:
(x,y)_goal = (5.2,0.0) meters
left wheel rpm = 25 rpm
right wheel rpm = 50 rpm
clearance = 0.2 meters

For custom values, run the following instead, with your values for x_goal, y_goal, left_rpm, right_rpm, and clearance:
```sh
ros2 run turtlebot3_project3 a_star.py x_goal y_goal left_rpm right_rpm clearance
```


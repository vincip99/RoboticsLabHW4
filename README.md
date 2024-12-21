# RoboticsLabHW3
Homework 4 for Robotics Lab 2024/2025

## :package: About
Control a mobile robot to follow a trajectory

## :hammer: Build
First build all the packages by using:

```
colcon build 
```
In each terminal you open, source the install directory:
```
source install/setup.bash
```

## :white_check_mark: Usage
### Run a Nav2 Simple Commander to enable an autonomous navigation task:

Launch the simulation with the command
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py rviz_config_name:=explore.rviz
```

launch the nav2 stack with the command
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```

Run 
```
ros2 run rl_fra2mo_description follow_waypoints.py
```

### Run a Nav2 explorer to enable an autonomous navigation task:
Launch the simulation with the command
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py rviz_config_name:=explore.rviz
```

launch the nav2 stack with the command
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py use_explore:=true
```


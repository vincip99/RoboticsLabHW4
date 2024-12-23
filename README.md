# RoboticsLabHW4
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
### Run a Nav2 Simple Commander to enable follow a set of waypoints:

Launch the simulation with the command
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py rviz_config_name:=explore.rviz
```

In another terminal launch the nav2 stack with the command
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```

Run in another terminal the commander script 
```
ros2 run rl_fra2mo_description follow_waypoints.py
```

### Run a Nav2 explorer to enable an autonomous navigation task to explore the map:
Launch the simulation with the command
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py rviz_config_name:=explore.rviz
```
To launch the expolre setup:
launch the large areas configuration
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py use_explore:=true params_file:=explore_larger_areas.yaml slam_params_file:=slam_larger_areas.yaml
```
launch the small areas configuration
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py use_explore:=true params_file:=explore_smaller_areas.yaml slam_params_file:=slam_smaller_areas.yaml
```
launch the large areas configuration
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py use_explore:=true params_file:=explore_dynamic.yaml slam_params_file:=slam_dynamic.yaml
```
launch the large areas configuration
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py use_explore:=true params_file:=explore_aggressive.yaml slam_params_file:=slam_aggressive.yaml
```

### Run a Nav2 vision task to detect the aruco marker:
Launch the simulation with the command
```
ros2 launch rl_fra2mo_description vision_nav_fra2mo.launch.py
```

launch the aruco tf publisher to have the aruco marker pose in the world frame
```
ros2 run rl_fra2mo_description aruco_pose_tf
```
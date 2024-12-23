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
### :pushpin: Run a Nav2 Simple Commander to follow a set of waypoints:

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

### :globe_with_meridians: Run a Nav2 explorer to enable an autonomous navigation task to explore the map:

#### Step 1: Start the Simulation Environment
Run the following command to launch the simulation and RViz with the appropriate configuration file:
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py rviz_config_name:=explore.rviz
```
#### Step 2: Launch the Exploration Setup
Choose one from the following configurations based on your task requirements:


#### 1. Large Areas Configuration
For exploring large areas, use the following command:
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py use_explore:=true params_file:=explore_larger_areas.yaml slam_params_file:=slam_larger_areas.yaml
```

#### 2. Small Areas Configuration
For exploring smaller, confined areas, use the following command:
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py use_explore:=true params_file:=explore_smaller_areas.yaml slam_params_file:=slam_smaller_areas.yaml
```

#### 3. Dynamic Areas Configuration
For exploring dynamic environments where the map frequently changes, use:
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py use_explore:=true params_file:=explore_dynamic.yaml slam_params_file:=slam_dynamic.yaml
```
#### 4. Aggressive Exploration Configuration
For fast and aggressive exploration of the environment, use the following command:
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py use_explore:=true params_file:=explore_aggressive.yaml slam_params_file:=slam_aggressive.yaml
```

#### Notes:
```text
- use_explore:=true: Ensures the exploration module is activated.
- params_file and slam_params_file: Specify the respective parameter files for exploration and SLAM configurations tailored to the task.
- Modify or extend these configurations based on your navigation and mapping requirements.
```

### :eyes: Run a Nav2 vision task to detect the aruco marker:
Launch the simulation with the command
```
ros2 launch rl_fra2mo_description vision_nav_fra2mo.launch.py
```

launch the aruco tf publisher to have the aruco marker pose in the world frame
```
ros2 run rl_fra2mo_description aruco_pose_tf
```
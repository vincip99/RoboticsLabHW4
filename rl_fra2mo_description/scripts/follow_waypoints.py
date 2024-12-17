#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import math

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

def load_waypoints(yaml_file):
    """Load waypoints from a YAML file."""
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def rpy_to_quaternion(roll, pitch, yaw):
    """Convert RPY angles to a quaternion."""
    q = quaternion_from_euler(roll, pitch, yaw)
    return {"x": q[0], "y": q[1], "z": q[2], "w": q[3]}

def print_goal(goal_pose, waypoint_number):
    """Prints the current goal's position and orientation as RPY angles."""
    # Extract position
    x = goal_pose.pose.position.x
    y = goal_pose.pose.position.y

    # Extract orientation (quaternion)
    qx = goal_pose.pose.orientation.x
    qy = goal_pose.pose.orientation.y
    qz = goal_pose.pose.orientation.z
    qw = goal_pose.pose.orientation.w

    # Convert quaternion to RPY
    roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

    print(f"Current Goal (Waypoint {waypoint_number}):")
    print(f"  Position: x={x}, y={y}")
    print(f"  Orientation: roll={roll}, pitch={pitch}, yaw={yaw}")

def world_to_map(world_pose, translation, rotation):
    """
    Transform a pose from the world frame to the map frame.

    Args:
        world_pose (PoseStamped): The pose in the world frame.
        translation (dict): The translation vector (x, y, z) from the world frame to the map frame.
        rotation (float): The rotation (in radians) from the world frame to the map frame.

    Returns:
        PoseStamped: The transformed pose in the map frame.
    """
    # Extract the position of the pose in the world frame
    px_world = world_pose.pose.position.x
    py_world = world_pose.pose.position.y
    pz_world = world_pose.pose.position.z

    # Apply the translation
    px_translated = px_world - translation["x"]
    py_translated = py_world - translation["y"]
    pz_translated = pz_world - translation.get("z", 0.0)  # Default z translation to 0 if not provided

    # Apply the rotation
    # Rotation is a simple 2D transformation on the (x, y) coordinates
    px_map = math.cos(-rotation) * px_translated - math.sin(-rotation) * py_translated
    py_map = math.sin(-rotation) * px_translated + math.cos(-rotation) * py_translated

    # Create the new pose in the map frame
    map_pose = PoseStamped()
    map_pose.header.frame_id = 'map'
    map_pose.header.stamp = world_pose.header.stamp
    map_pose.pose.position.x = px_map
    map_pose.pose.position.y = py_map
    map_pose.pose.position.z = pz_translated

    # Transform the orientation
    # Convert the world quaternion to RPY
    qx = world_pose.pose.orientation.x
    qy = world_pose.pose.orientation.y
    qz = world_pose.pose.orientation.z
    qw = world_pose.pose.orientation.w
    roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

    # Apply the rotation to the yaw
    yaw_map = yaw + (-rotation)

    # Convert back to a quaternion
    quaternion_map = quaternion_from_euler(roll, pitch, yaw_map)
    map_pose.pose.orientation.x = quaternion_map[0]
    map_pose.pose.orientation.y = quaternion_map[1]
    map_pose.pose.orientation.z = quaternion_map[2]
    map_pose.pose.orientation.w = quaternion_map[3]

    return map_pose


def main():
    # Initialize ROS 2 node
    rclpy.init()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0

    # Orientation in RPY (roll, pitch, yaw)
    roll = 0.0   # Roll angle in radians
    pitch = 0.0  # Pitch angle in radians
    yaw = 0.0   # Yaw angle in radians 
    # Convert RPY to Quaternion
    quaternion = rpy_to_quaternion(roll, pitch, yaw)

    # Assign Quaternion to PoseStamped
    initial_pose.pose.orientation.x = quaternion["x"]
    initial_pose.pose.orientation.y = quaternion["y"]
    initial_pose.pose.orientation.z = quaternion["z"]
    initial_pose.pose.orientation.w = quaternion["w"]
    navigator.setInitialPose(initial_pose)

    # Get the package share directory and load the YAML file
    package_share_directory = get_package_share_directory('rl_fra2mo_description')
    yaml_file = os.path.join(package_share_directory, 'config', 'aruco_path.yaml')
    waypoints = load_waypoints(yaml_file)

    def create_pose(transform):

        # Transformation from world frame to map frame (only for slam)
        translation = {"x": -3.0, "y": 3.5}
        rotation = -math.pi / 2  # 90 degrees

        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]

        # Convert RPY to quaternion
        quaternion = rpy_to_quaternion(
            transform["orientation"]["roll"],
            transform["orientation"]["pitch"],
            transform["orientation"]["yaw"]
        )

        pose.pose.orientation.x = quaternion["x"]
        pose.pose.orientation.y = quaternion["y"]
        pose.pose.orientation.z = quaternion["z"]
        pose.pose.orientation.w = quaternion["w"]

        map_pose = world_to_map(pose, translation, rotation)

        return map_pose

    # Convert loaded waypoints into PoseStamped messages
    goal_poses = list(map(create_pose, waypoints["waypoints"]))

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            current_waypoint = feedback.current_waypoint + 1
            current_goal = goal_poses[feedback.current_waypoint]
            print_goal(current_goal, current_waypoint)

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException, ExtrapolationException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math


class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')

        # Initialize the navigator
        self.navigator = BasicNavigator()
        self.aruco_pose = None

        # TF broadcaster for publishing Aruco pose
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF2 buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # Aruco pose subscriber
        self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',  # Topic where Aruco pose is published
            self.aruco_pose_callback,
            10
        )

        # Map transformation parameters (world frame to map frame)
        self.translation = {"x": -3.0, "y": 3.5}
        self.rotation = -math.pi / 2  # -90 degrees

    def transform_to_map_frame(self, position, orientation):
        """
        Transform a position and orientation from the world frame to the map frame.
        Args:
            position (tuple): (x, y, z) position in the world frame.
            orientation (tuple): (roll, pitch, yaw) orientation in the world frame.
        Returns:
            PoseStamped: Transformed pose in the map frame.
        """
        # Unpack inputs
        px_world, py_world, pz_world = position
        roll_world, pitch_world, yaw_world = orientation

        # Apply translation
        px_translated = px_world - self.translation["x"]
        py_translated = py_world - self.translation["y"]

        # Apply rotation (2D transformation)
        px_map = math.cos(-self.rotation) * px_translated - math.sin(-self.rotation) * py_translated
        py_map = math.sin(-self.rotation) * px_translated + math.cos(-self.rotation) * py_translated

        # Transform orientation
        yaw_map = yaw_world + (-self.rotation)
        q_map = quaternion_from_euler(roll_world, pitch_world, yaw_map)

        # Create the PoseStamped
        map_pose = PoseStamped()
        map_pose.header.frame_id = 'map'
        map_pose.header.stamp = self.get_clock().now().to_msg()
        map_pose.pose.position.x = px_map
        map_pose.pose.position.y = py_map
        map_pose.pose.position.z = pz_world
        map_pose.pose.orientation.x = q_map[0]
        map_pose.pose.orientation.y = q_map[1]
        map_pose.pose.orientation.z = q_map[2]
        map_pose.pose.orientation.w = q_map[3]
        return map_pose

    def aruco_pose_callback(self, msg):
        """ Callback when the Aruco marker is detected. """
        self.get_logger().info("Aruco marker detected!")
        self.aruco_pose = msg

        # Extract the position and orientation from the Aruco marker pose
        position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        orientation_quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

        # Convert quaternion to roll, pitch, yaw
        roll, pitch, yaw = euler_from_quaternion(orientation_quat)
        orientation = (roll, pitch, yaw)

        transformed_pose = self.transform_to_map_frame(position, orientation)

        # Print the transformed pose in the map frame
        self.get_logger().info(
            f"Aruco marker pose in map frame: \n"
            f"Position: x = {transformed_pose.pose.position.x:.3f}, "
            f"y = {transformed_pose.pose.position.y:.3f}, "
            f"z = {transformed_pose.pose.position.z:.3f}\n"
            f"Orientation: x = {transformed_pose.pose.orientation.x:.3f}, "
            f"y = {transformed_pose.pose.orientation.y:.3f}, "
            f"z = {transformed_pose.pose.orientation.z:.3f}, "
            f"w = {transformed_pose.pose.orientation.w:.3f}"
        )

        # Publish the Aruco pose as a TF transform
        self.publish_aruco_tf(transformed_pose)

    def publish_aruco_tf(self, pose_msg):
        """ Broadcast Aruco pose as a TF transform. """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "aruco_marker"

        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        t.transform.rotation.x = pose_msg.pose.orientation.x
        t.transform.rotation.y = pose_msg.pose.orientation.y
        t.transform.rotation.z = pose_msg.pose.orientation.z
        t.transform.rotation.w = pose_msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

    def navigate_to_position(self, pose_stamped):
        """ Send a navigation goal and wait for completion. """
        self.navigator.goToPose(pose_stamped)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} meters")

    def main(self):
        """ Main task: navigate, detect Aruco, and return. """
        self.navigator.waitUntilNav2Active(localizer="smoother_server")

        # Define start and goal positions in the world frame
        start_position_world = (-3.0, 3.5, 0.1)
        start_orientation_world = (0.0, 0.0, 1.57)  # Yaw = 90 degrees

        final_position_world = (-3.87, -4.02, 0.1)
        final_orientation_world = (0.0, 0.0, 1.54)  # Yaw slightly different

        # Transform start and final positions to the map frame
        start_pose = self.transform_to_map_frame(start_position_world, start_orientation_world)
        final_pose = self.transform_to_map_frame(final_position_world, final_orientation_world)

        # Navigate to goal
        self.get_logger().info("Navigating to obstacle 9...")
        self.navigate_to_position(final_pose)

        # Wait for Aruco marker detection
        self.get_logger().info("Looking for Aruco marker...")
        while not self.aruco_pose:
            rclpy.spin_once(self, timeout_sec=0.5)

        # Log Aruco pose in the map frame
        self.get_logger().info("Aruco marker detected and TF broadcasted.")

        # Return to start position
        self.get_logger().info("Returning to start position...")
        self.navigate_to_position(start_pose)

        self.get_logger().info("Task complete. Shutting down.")

def main():
    rclpy.init()
    navigator = ArucoNavigator()
    navigator.main()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

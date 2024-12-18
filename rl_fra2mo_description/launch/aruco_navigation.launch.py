import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Directories for packages
    fra2mo_dir = FindPackageShare('rl_fra2mo_description')
    aruco_ros_dir = FindPackageShare('aruco_ros')

    # Use simulation time argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include gazebo_fra2mo.launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fra2mo_dir, 'launch', 'gazebo_fra2mo.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include fra2mo_explore.launch file
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fra2mo_dir, 'launch', 'fra2mo_explore.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include aruco_ros single.launch.py
    aruco_ros_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([aruco_ros_dir, 'launch', 'single.launch.py'])
        ),
        launch_arguments={
            'marker_size': '0.1',  # Marker size in meters
            'marker_id': '115',   # Marker ID to detect
            'use_sim_time': use_sim_time
        }.items()
    )

    # Return the complete LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        gazebo_launch,
        explore_launch,
        aruco_ros_node
    ])

import os
from math import pi

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Percorsi ai file
    xacro_file_name = "fra2mo.urdf.xacro"
    xacro = os.path.join(get_package_share_directory('rl_fra2mo_description'), "urdf", xacro_file_name)

    models_path = os.path.join(get_package_share_directory('rl_fra2mo_description'), 'models')
    world_file = os.path.join(get_package_share_directory('rl_fra2mo_description'), "worlds", "leonardo_race_field.sdf")

    # Genera la descrizione del robot usando xacro
    robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', xacro]),value_type=str)}
    
    # use_sim_time_arg = DeclareLaunchArgument(
    #     'use_sim_time', default_value='true', description='Use simulation/Gazebo clock')

    declared_arguments = []

    # Declare launch argument for Gazebo world file
    declared_arguments.append(
        DeclareLaunchArgument(
            'gz_args',
            default_value=world_file,
            description='Path to world file'
        )
    )

    # Declare rviz launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_name",
            default_value="aruco_nav.rviz",
            description="Name of the RViz config file to load."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rl_fra2mo_description"), "rviz_conf", LaunchConfiguration("rviz_config_name")]
            ),
            description="Absolute path to the RViz configuration file to load."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time if true."
        ),
    )

    # Declare launch arguments for robot pose
    declared_arguments.append(
        DeclareLaunchArgument(
            'x',
            default_value='-3.0',
            description='Initial X position of the robot'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'y',
            default_value='3.5',
            description='Initial Y position of the robot'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'yaw',
            default_value=str(-pi / 2),
            description='Initial yaw (orientation in radians) of the robot'
        )
    )

    # Substitutions for position and yaw
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_yaw = LaunchConfiguration('yaw')
    # Rviz config file
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": True}
            ]
    )

    # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{"use_sim_time": True}]
    )
    
    # Gazebo simulation launch description
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    # Define a Node to spawn the robot in the Gazebo simulation
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'fra2mo',
                   '-allow_renaming', 'true',
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', '0.1',  # Slightly above ground
                    '-Y', spawn_yaw]   # Yaw in radians
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',],
                   #'/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked']
        remappings=[('/camera_info', '/stereo/left/camera_info')], # Remap Gazebo topic /camera_info
        output='screen'
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera'],
        remappings=[('/camera', '/stereo/left/image_rect_color')], # Remap Gazebo topic /camera
        output='screen',
    )

    odom_tf = Node(
        package='rl_fra2mo_description',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}]
    )

    laser_id_link_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='lidar_staticTF',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'laser_frame', 'fra2mo/base_footprint/laser_frame'],
                     parameters=[{"use_sim_time": True}]
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(get_package_share_directory('rl_fra2mo_description'), "config/ekf.yaml"),
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ign_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        namespace="fra2mo"
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ign = [gazebo_ignition, gz_spawn_entity]
    nodes_to_start = [robot_state_publisher_node, 
                      joint_state_publisher_node, 
                      *ign, bridge,
                      odom_tf, 
                      laser_id_link_tf,
                      start_gazebo_ros_image_bridge_cmd,
                      ign_clock_bridge,
                      #robot_localization_node,
                      rviz_node]

    return LaunchDescription([SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value = models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))] + declared_arguments + nodes_to_start)
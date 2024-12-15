import os

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Percorsi ai file
    xacro_file_name = "fra2mo.urdf.xacro"
    xacro = os.path.join(get_package_share_directory('rl_fra2mo_description'), "urdf", xacro_file_name)

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config_name",
            default_value="conf_base.rviz",
            description="Name of the RViz config file to load."
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rl_fra2mo_description"), "rviz_conf", LaunchConfiguration("rviz_config_name")]
            ),
            description="Absolute path to the RViz configuration file to load."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time if true."
        ),
    ]

    # Retrieve launch configurations
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Configurazione per l'uso del tempo di simulazione
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Genera la descrizione del robot usando xacro
    robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', xacro]),value_type=str)}
    
    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": use_sim_time}
            ]
    )

    # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # nodes_to_start = [robot_state_publisher_node, joint_state_publisher_node, rviz_node]
    nodes_to_start = [robot_state_publisher_node, joint_state_publisher_node, rviz_node]

    return LaunchDescription(declared_arguments + nodes_to_start)
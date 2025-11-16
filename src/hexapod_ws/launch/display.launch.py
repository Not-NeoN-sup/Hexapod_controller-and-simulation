from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("hexapod_ws")
    model_path = PathJoinSubstitution([pkg_share, "description", "hexapod.urdf.xacro"])
    controllers_path = PathJoinSubstitution([pkg_share, "config", "parameters.yaml"])

    declared_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
    ]

    robot_description = Command(['xacro ', model_path])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description
        }],
        output='screen'
    )

    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description},
            controllers_path
        ],
        output='screen'
    )

    # Spawner nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    hexapod_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_controller'],
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        hexapod_controller_spawner
    ])

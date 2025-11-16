from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare  # ✅ correct import for Jazzy

def generate_launch_description():

    pkg_share = FindPackageShare('hexapod_ws')

    urdf_path = PathJoinSubstitution([
        pkg_share, 'description', 'hexapod.urdf.xacro'
    ])

    parameters_file = PathJoinSubstitution([
        pkg_share, 'config', 'parameters.yaml'
    ])

    from launch.actions import ExecuteProcess

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf', '--force-version', '8'],
        output='screen'
    )
    from launch.substitutions import Command

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_path])
        }],
        output='screen'
    )


    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'hexapod'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    hexapod_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', parameters_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        hexapod_controller_spawner,
    ])

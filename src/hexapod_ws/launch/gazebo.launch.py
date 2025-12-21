#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare 
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('hexapod_ws')
    urdf_path = PathJoinSubstitution([pkg_share, 'description', 'hexapod.urdf.xacro'])
    parameters_file = PathJoinSubstitution([pkg_share, 'config', 'parameters.yaml'])

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', 'empty.sdf'],
        output='screen'
    )
    
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str) 
        }],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'hexapod'],
        output='screen'
    )

    # Spawners remain the same
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    hexapod_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_controller', '--controller-manager', '/controller_manager', '--param-file', parameters_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        hexapod_controller_spawner,
    ])
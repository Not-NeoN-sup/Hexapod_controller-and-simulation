from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare 
from launch_ros.parameter_descriptions import ParameterValue # <--- 1. ADDED IMPORT

def generate_launch_description():

    pkg_share = FindPackageShare('hexapod_ws')

    urdf_path = PathJoinSubstitution([
        pkg_share, 'description', 'hexapod.urdf.xacro'
    ])

    parameters_file = PathJoinSubstitution([
        pkg_share, 'config', 'parameters.yaml'
    ])

    # Using the default 'empty.sdf' world as requested.
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf', '--force-version', '8'],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            # 2. WRAP THE COMMAND OUTPUT as a string
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

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}], # 3. ADDED use_sim_time
        output='screen'
    )

    hexapod_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', parameters_file],
        parameters=[{'use_sim_time': True}], # 4. ADDED use_sim_time
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        hexapod_controller_spawner,
    ])
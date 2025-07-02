from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

import xacro  # 🔥 Enables XACRO parsing

def generate_launch_description():
    # Package paths
    pkg_path = get_package_share_directory('cori_description')
    xacro_path = os.path.join(pkg_path, 'urdf', 'cori.urdf.xacro')

    # Use proper relative path to world file
    world_path = os.path.join(pkg_path, 'worlds', 'laundry_world.sdf')

    # Parse XACRO to generate robot_description
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = robot_description_config.toxml()

    return LaunchDescription([
        # Launch Gazebo with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # State publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Spawn CORI in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'cori', '-topic', 'robot_description'],
            output='screen'
        )
    ])

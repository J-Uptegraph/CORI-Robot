from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the SDF world file
    package_share_dir = get_package_share_directory('cori_description')
    world_file = os.path.join(package_share_dir, 'worlds', 'simple_world.sdf')

    # Launch Gazebo server with ROS plugins
    gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Launch Gazebo client (optional, for visualization)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cori_bot',
            '-file', os.path.join(package_share_dir, 'urdf', 'cori_bot.urdf.xacro'),
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        gazebo_client,
        spawn_entity
    ])
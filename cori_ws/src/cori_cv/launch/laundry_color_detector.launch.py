from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cori_cv',
            executable='laundry_color_detector',
            name='laundry_color_detector',
            output='screen'
        )
    ])

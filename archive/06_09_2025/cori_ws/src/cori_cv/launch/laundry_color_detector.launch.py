import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

share_dir = get_package_share_directory('cori_cv')
config_path = os.path.join(share_dir, 'color_piles.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cori_cv',
            executable='laundry_color_detector',
            name='laundry_color_detector',
            output='screen',
            parameters=[{'config_path': config_path}]
        )
    ])

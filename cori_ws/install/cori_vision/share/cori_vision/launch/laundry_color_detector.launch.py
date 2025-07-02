from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [800, 600],
                'pixel_format': 'YUYV',
                'camera_frame_id': 'camera_frame',
                'brightness': 128,    
                'contrast': 128,          
                'saturation': 128,         
                'sharpness': 140,        
                'gain': 5,                  
                'auto_exposure': 1,    
                'auto_white_balance': 1   
            }],
            remappings=[
                ('/image_raw', '/camera/color/image_raw')
            ]
        ),
        Node(
            package='cori_vision',
            executable='laundry_color_detector',
            name='laundry_color_detector',
            output='log',  # Suppress terminal output
            remappings=[
                ('/image_raw', '/camera/color/image_raw')
            ]
        )
    ])
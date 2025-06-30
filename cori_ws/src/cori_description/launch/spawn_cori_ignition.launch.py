from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Paths
    package_share_dir = get_package_share_directory('cori_description')
    world_path = os.path.join(package_share_dir, 'worlds', 'laundry_world.sdf')
    xacro_file = os.path.join(package_share_dir, 'urdf', 'cori.urdf.xacro')
    
    # Validate file paths
    if not os.path.exists(world_path):
        raise FileNotFoundError(f"World file not found: {world_path}")
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")
    
    # Process Xacro file to generate URDF
    try:
        robot_desc = xacro.process_file(xacro_file).toxml()
        print("DEBUG - Processed URDF contains:", robot_desc[:1000])  # Print first 1000 chars
    except Exception as e:
        raise RuntimeError(f"Failed to process Xacro file: {str(e)}")
    
    # Gazebo Simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'{world_path} -r --render-engine ogre2'}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )
    
    # Joint State Publisher - THIS WAS MISSING!
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # ROS-Gazebo Bridge for Joint States - THIS WAS MISSING!
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=['/world/laundry_world/model/cori/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # ROS-Gazebo Bridge for Joint Commands - THIS WAS MISSING!
    joint_cmd_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_cmd_bridge',
        arguments=['/model/cori/joint/head_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Spawn Robot in Gazebo
    spawn_cori = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_cori',
        arguments=['-name', 'cori', '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '4', 
                '-R', '0', '-P', '0', '-Y', '1.57'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch Description
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        joint_state_publisher,        # NEW!
        joint_state_bridge,          # NEW!
        joint_cmd_bridge,            # NEW!
        spawn_cori
    ])

if __name__ == '__main__':
    generate_launch_description()
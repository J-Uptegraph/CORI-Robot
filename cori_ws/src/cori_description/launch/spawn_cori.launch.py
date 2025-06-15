#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Directories
    pkg_cori_description = FindPackageShare('cori_description')
    
    # Paths
    default_model_path = os.path.join(pkg_cori_description.perform(context=None), 'urdf', 'cori.urdf')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Robot description - read URDF file directly
    with open(default_model_path, 'r') as infp:
        robot_description_content = infp.read()
    
    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Gazebo launch (use default empty world for now)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), 
            '/launch/gazebo.launch.py'
        ])
    )

    # Spawn robot in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_cori',
        arguments=['-entity', 'cori_bot', 
                  '-topic', 'robot_description',
                  '-x', '0.0',
                  '-y', '0.0', 
                  '-z', '0.1'],
        output='screen'
    )

    # Create and return launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes to launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot_node)

    return ld
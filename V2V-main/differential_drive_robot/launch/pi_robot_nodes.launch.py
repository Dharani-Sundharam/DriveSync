#!/usr/bin/env python3

"""
Launch file for Raspberry Pi - Robot control nodes only
Runs all robot control without RViz or teleop
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    
    # Get URDF file path
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'differential_drive_robot_static.urdf'
    )
    
    # Robot description
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    # Joint state publisher (our custom one)
    joint_state_publisher_node = Node(
        package='differential_drive_robot',
        executable='robot_state_publisher.py',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_diameter': 0.20
        }]
    )
    
    # Arduino interface node
    arduino_interface_node = Node(
        package='differential_drive_robot',
        executable='arduino_interface.py',
        name='arduino_interface',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': port,
            'baud': 57600,
            'timeout': 0.5,
            'wheel_diameter': 0.20,
            'wheel_track': 0.35,
            'encoder_resolution': 360,
            'max_speed': 1.0
        }]
    )
    
    # Odometry node
    odometry_node = Node(
        package='differential_drive_robot',
        executable='odometry_node.py',
        name='odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_diameter': 0.20,
            'wheel_track': 0.35,
            'encoder_resolution': 360,
            'publish_tf': True,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint'
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for Arduino'
        ),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        arduino_interface_node,
        odometry_node
    ])

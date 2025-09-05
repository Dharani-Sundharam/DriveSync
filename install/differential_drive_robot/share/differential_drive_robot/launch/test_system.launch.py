#!/usr/bin/env python3

"""
Test launch file for differential drive robot system
Tests the system without requiring Arduino connection
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Get file paths
    package_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_file = os.path.join(package_dir, '..', 'urdf', 'differential_drive_robot_static.urdf')
    rviz_config_file = os.path.join(package_dir, '..', 'rviz', 'robot_view.rviz')
    
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
    
    # System test node (replaces Arduino interface for testing)
    test_node = Node(
        package='differential_drive_robot',
        executable='test_system.py',
        name='system_tester',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz if true'
        ),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        odometry_node,
        test_node,
        rviz_node
    ])

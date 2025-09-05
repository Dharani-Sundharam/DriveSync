#!/usr/bin/env python3

"""
Launch file for PC - Visualization and control only
Runs RViz and optionally teleop keyboard
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_teleop = LaunchConfiguration('use_teleop', default='true')
    
    # Get RViz config file path
    rviz_config_file = os.path.join(
        os.path.dirname(__file__), '..', 'rviz', 'robot_view.rviz'
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
    
    # Teleop keyboard node (optional)
    teleop_node = Node(
        package='differential_drive_robot',
        executable='teleop_keyboard.py',
        name='teleop_keyboard',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'speed_increment': 0.1
        }],
        condition=IfCondition(use_teleop)
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Start teleop keyboard if true'
        ),
        
        rviz_node,
        teleop_node
    ])

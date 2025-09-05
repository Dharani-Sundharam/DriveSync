#!/usr/bin/env python3

"""
Complete launch file for differential drive robot with RViz and teleop
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_teleop = LaunchConfiguration('use_teleop', default='true')
    
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
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(use_rviz)
    )
    
    # Teleop keyboard node
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
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for Arduino'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz if true'
        ),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Start teleop keyboard if true'
        ),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        arduino_interface_node,
        odometry_node,
        rviz_node,
        teleop_node
    ])

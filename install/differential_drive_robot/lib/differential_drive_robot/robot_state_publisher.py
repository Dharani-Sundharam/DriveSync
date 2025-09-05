#!/usr/bin/env python3

"""
Robot State Publisher for Differential Drive Robot
Publishes robot description and joint states
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher_node')
        
        # Declare parameters
        self.declare_parameter('wheel_diameter', 0.20)
        
        # Get parameters
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.wheel_radius = self.wheel_diameter / 2.0
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribers
        self.wheel_speed_sub = self.create_subscription(
            Float32MultiArray, 'wheel_speeds', self.wheel_speed_callback, 10)
        
        # Initialize joint states
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = self.get_clock().now()
        
        # Timer for publishing joint states
        self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        self.get_logger().info('Robot State Publisher Node Started')
    
    def wheel_speed_callback(self, msg):
        """Update wheel positions based on wheel speeds"""
        if len(msg.data) < 2:
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0:
            left_speed = msg.data[0]  # m/s
            right_speed = msg.data[1]  # m/s
            
            # Convert linear speed to angular velocity
            left_angular_vel = left_speed / self.wheel_radius
            right_angular_vel = right_speed / self.wheel_radius
            
            # Update wheel positions
            self.left_wheel_pos += left_angular_vel * dt
            self.right_wheel_pos += right_angular_vel * dt
            
            self.last_time = current_time
    
    def publish_joint_states(self):
        """Publish joint states for the robot"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['drivewhl_l_joint', 'drivewhl_r_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [0.0, 0.0]  # Could be calculated from wheel speeds
        joint_state.effort = [0.0, 0.0]
        
        self.joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        robot_state_publisher = RobotStatePublisher()
        rclpy.spin(robot_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

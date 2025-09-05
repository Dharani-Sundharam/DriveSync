#!/usr/bin/env python3

"""
Test script for differential drive robot system
Publishes fake encoder data to test odometry and visualization
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist
import math
import time


class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        
        # Publishers
        self.encoder_pub = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)
        self.wheel_speed_pub = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize test data
        self.left_encoder = 0
        self.right_encoder = 0
        self.time_start = time.time()
        
        # Create timers
        self.create_timer(0.1, self.publish_test_data)  # 10 Hz
        self.create_timer(2.0, self.send_test_commands)  # Change commands every 2 seconds
        
        self.command_phase = 0
        self.get_logger().info('System Tester Started - Publishing fake data for testing')
    
    def publish_test_data(self):
        """Publish fake encoder and wheel speed data"""
        # Simulate robot movement with sinusoidal pattern
        current_time = time.time() - self.time_start
        
        # Simulate encoder increments (fake movement)
        left_increment = int(10 * math.sin(current_time * 0.5))
        right_increment = int(8 * math.cos(current_time * 0.3))
        
        self.left_encoder += left_increment
        self.right_encoder += right_increment
        
        # Publish encoder data
        encoder_msg = Int32MultiArray()
        encoder_msg.data = [self.left_encoder, self.right_encoder]
        self.encoder_pub.publish(encoder_msg)
        
        # Calculate and publish wheel speeds (fake)
        wheel_circumference = math.pi * 0.20  # 20cm diameter wheels
        ticks_per_meter = 360 / wheel_circumference
        
        left_speed = left_increment / (ticks_per_meter * 0.1)  # m/s
        right_speed = right_increment / (ticks_per_meter * 0.1)  # m/s
        
        speed_msg = Float32MultiArray()
        speed_msg.data = [left_speed, right_speed]
        self.wheel_speed_pub.publish(speed_msg)
    
    def send_test_commands(self):
        """Send different test velocity commands"""
        twist = Twist()
        
        if self.command_phase == 0:
            # Move forward
            twist.linear.x = 0.3
            twist.angular.z = 0.0
            self.get_logger().info('Test: Moving forward')
        elif self.command_phase == 1:
            # Turn left
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().info('Test: Turning left')
        elif self.command_phase == 2:
            # Move backward
            twist.linear.x = -0.2
            twist.angular.z = 0.0
            self.get_logger().info('Test: Moving backward')
        elif self.command_phase == 3:
            # Turn right
            twist.linear.x = 0.0
            twist.angular.z = -0.5
            self.get_logger().info('Test: Turning right')
        elif self.command_phase == 4:
            # Stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Test: Stopping')
        
        self.cmd_vel_pub.publish(twist)
        self.command_phase = (self.command_phase + 1) % 5


def main(args=None):
    rclpy.init(args=args)
    
    try:
        system_tester = SystemTester()
        rclpy.spin(system_tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

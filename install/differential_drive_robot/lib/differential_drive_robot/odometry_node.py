#!/usr/bin/env python3

"""
Odometry Node for Differential Drive Robot
Calculates and publishes robot odometry based on wheel encoder data
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import time


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Declare parameters
        self.declare_parameter('wheel_diameter', 0.20)  # 20cm wheels
        self.declare_parameter('wheel_track', 0.35)     # 35cm between wheels
        self.declare_parameter('encoder_resolution', 360)  # ticks per revolution
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
        # Get parameters
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.wheel_track = self.get_parameter('wheel_track').get_parameter_value().double_value
        self.encoder_resolution = self.get_parameter('encoder_resolution').get_parameter_value().integer_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        # Calculate conversion factors
        self.wheel_radius = self.wheel_diameter / 2.0
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.ticks_per_meter = self.encoder_resolution / self.wheel_circumference
        
        # Initialize pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Initialize velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        # Initialize encoder values
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.last_time = self.get_clock().now()
        self.first_reading = True
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.encoder_sub = self.create_subscription(
            Int32MultiArray, 'encoder_ticks', self.encoder_callback, 10)
        
        self.get_logger().info('Odometry Node Started')
    
    def encoder_callback(self, msg):
        """Process encoder data and update odometry"""
        if len(msg.data) < 2:
            self.get_logger().warn('Invalid encoder data received')
            return
        
        current_time = self.get_clock().now()
        left_encoder = msg.data[0]
        right_encoder = msg.data[1]
        
        if self.first_reading:
            self.last_left_encoder = left_encoder
            self.last_right_encoder = right_encoder
            self.last_time = current_time
            self.first_reading = False
            return
        
        # Calculate time difference
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        
        # Calculate encoder differences
        left_delta = left_encoder - self.last_left_encoder
        right_delta = right_encoder - self.last_right_encoder
        
        # Convert to distances
        left_distance = left_delta / self.ticks_per_meter
        right_distance = right_delta / self.ticks_per_meter
        
        # Calculate robot motion
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_track
        
        # Update pose
        delta_x = distance * math.cos(self.theta + delta_theta/2.0)
        delta_y = distance * math.sin(self.theta + delta_theta/2.0)
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize angle
        self.theta = self.normalize_angle(self.theta)
        
        # Calculate velocities
        self.vx = distance / dt
        self.vy = 0.0  # No lateral movement for differential drive
        self.vtheta = delta_theta / dt
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Update last values
        self.last_left_encoder = left_encoder
        self.last_right_encoder = right_encoder
        self.last_time = current_time
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def publish_odometry(self, current_time):
        """Publish odometry message and transform"""
        # Create quaternion from yaw
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Set velocities
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vtheta
        
        # Set covariance matrices (simple diagonal values)
        pose_covariance = [0.1, 0, 0, 0, 0, 0,
                          0, 0.1, 0, 0, 0, 0,
                          0, 0, 0.1, 0, 0, 0,
                          0, 0, 0, 0.1, 0, 0,
                          0, 0, 0, 0, 0.1, 0,
                          0, 0, 0, 0, 0, 0.1]
        
        twist_covariance = [0.05, 0, 0, 0, 0, 0,
                           0, 0.05, 0, 0, 0, 0,
                           0, 0, 0.05, 0, 0, 0,
                           0, 0, 0, 0.05, 0, 0,
                           0, 0, 0, 0, 0.05, 0,
                           0, 0, 0, 0, 0, 0.05]
        
        odom.pose.covariance = pose_covariance
        odom.twist.covariance = twist_covariance
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Publish transform
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        odometry_node = OdometryNode()
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

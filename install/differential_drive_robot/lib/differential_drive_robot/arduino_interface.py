#!/usr/bin/env python3

"""
Arduino Interface Node for Differential Drive Robot
Communicates with ROSArduinoBridge firmware via serial connection
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import serial
import threading
import time
import math


class ArduinoInterface(Node):
    def __init__(self):
        super().__init__('arduino_interface')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('wheel_diameter', 0.20)  # 20cm wheels
        self.declare_parameter('wheel_track', 0.35)     # 35cm between wheels
        self.declare_parameter('encoder_resolution', 360)  # ticks per revolution
        self.declare_parameter('max_speed', 1.0)        # m/s
        
        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.wheel_track = self.get_parameter('wheel_track').get_parameter_value().double_value
        self.encoder_resolution = self.get_parameter('encoder_resolution').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        
        # Calculate conversion factors
        self.wheel_radius = self.wheel_diameter / 2.0
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.ticks_per_meter = self.encoder_resolution / self.wheel_circumference
        
        # Initialize serial connection
        self.serial_port = None
        self.connect_to_arduino()
        
        # Publishers
        self.encoder_pub = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)
        self.wheel_speeds_pub = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Initialize variables
        self.left_encoder = 0
        self.right_encoder = 0
        self.last_encoder_time = time.time()
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        
        # Start encoder reading thread
        self.encoder_thread = threading.Thread(target=self.read_encoders_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()
        
        # Timer for publishing encoder data
        self.create_timer(0.1, self.publish_encoder_data)  # 10 Hz
        
        self.get_logger().info('Arduino Interface Node Started')
        
    def connect_to_arduino(self):
        """Connect to Arduino via serial port"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            
            # Reset encoders
            self.send_command('r')
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial_port = None
    
    def send_command(self, command):
        """Send command to Arduino"""
        if self.serial_port and self.serial_port.is_open:
            try:
                command_str = command + '\r'
                self.serial_port.write(command_str.encode('utf-8'))
                return True
            except Exception as e:
                self.get_logger().error(f'Error sending command: {e}')
                return False
        return False
    
    def read_response(self):
        """Read response from Arduino"""
        if self.serial_port and self.serial_port.is_open:
            try:
                response = self.serial_port.readline().decode('utf-8').strip()
                return response
            except Exception as e:
                self.get_logger().error(f'Error reading response: {e}')
                return None
        return None
    
    def read_encoders(self):
        """Read encoder values from Arduino"""
        if self.send_command('e'):
            response = self.read_response()
            if response:
                try:
                    left_enc, right_enc = map(int, response.split())
                    return left_enc, right_enc
                except ValueError:
                    self.get_logger().warn(f'Invalid encoder response: {response}')
        return None, None
    
    def read_encoders_loop(self):
        """Continuously read encoders in a separate thread"""
        while rclpy.ok():
            left_enc, right_enc = self.read_encoders()
            if left_enc is not None and right_enc is not None:
                self.left_encoder = left_enc
                self.right_encoder = right_enc
            time.sleep(0.05)  # 20 Hz
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages and convert to motor commands"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Limit speeds
        linear_vel = max(-self.max_speed, min(self.max_speed, linear_vel))
        angular_vel = max(-2.0, min(2.0, angular_vel))  # Limit angular velocity
        
        # Convert to wheel velocities (differential drive kinematics)
        left_vel = linear_vel - (angular_vel * self.wheel_track / 2.0)
        right_vel = linear_vel + (angular_vel * self.wheel_track / 2.0)
        
        # Convert velocities to ticks per frame (PID runs at 30 Hz)
        pid_rate = 30.0
        left_ticks_per_frame = (left_vel * self.ticks_per_meter) / pid_rate
        right_ticks_per_frame = (right_vel * self.ticks_per_meter) / pid_rate
        
        # Send motor command
        motor_cmd = f'm {int(left_ticks_per_frame)} {int(right_ticks_per_frame)}'
        self.send_command(motor_cmd)
        
        self.get_logger().debug(f'Motor command: {motor_cmd}')
    
    def publish_encoder_data(self):
        """Publish encoder data and wheel speeds"""
        current_time = time.time()
        dt = current_time - self.last_encoder_time
        
        if dt > 0:
            # Calculate wheel speeds
            left_delta = self.left_encoder - self.last_left_encoder
            right_delta = self.right_encoder - self.last_right_encoder
            
            left_speed = (left_delta / self.ticks_per_meter) / dt
            right_speed = (right_delta / self.ticks_per_meter) / dt
            
            # Publish encoder ticks
            encoder_msg = Int32MultiArray()
            encoder_msg.data = [self.left_encoder, self.right_encoder]
            self.encoder_pub.publish(encoder_msg)
            
            # Publish wheel speeds
            speed_msg = Float32MultiArray()
            speed_msg.data = [left_speed, right_speed]
            self.wheel_speeds_pub.publish(speed_msg)
            
            # Update last values
            self.last_encoder_time = current_time
            self.last_left_encoder = self.left_encoder
            self.last_right_encoder = self.right_encoder
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        if self.serial_port and self.serial_port.is_open:
            # Stop the robot
            self.send_command('m 0 0')
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        arduino_interface = ArduinoInterface()
        rclpy.spin(arduino_interface)
    except KeyboardInterrupt:
        pass
    finally:
        if 'arduino_interface' in locals():
            arduino_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

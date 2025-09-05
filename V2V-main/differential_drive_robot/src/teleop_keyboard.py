#!/usr/bin/env python3

"""
Keyboard Teleoperation Node for Differential Drive Robot
Allows manual control of the robot using keyboard input
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Declare parameters
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('speed_increment', 0.1)
        
        # Get parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.speed_increment = self.get_parameter('speed_increment').get_parameter_value().double_value
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Current speeds
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Teleop Keyboard Node Started')
        self.print_instructions()
        
        # Start keyboard input loop
        self.run()
    
    def print_instructions(self):
        """Print control instructions"""
        msg = """
        Differential Drive Robot Teleop Control
        ======================================
        
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        
        space key, k : force stop
        
        CTRL-C to quit
        """
        print(msg)
        self.get_logger().info(f'Max Linear Speed: {self.max_linear_speed:.2f} m/s')
        self.get_logger().info(f'Max Angular Speed: {self.max_angular_speed:.2f} rad/s')
    
    def get_key(self):
        """Get keyboard input"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def constrain(self, input_vel, low_bound, high_bound):
        """Constrain velocity within bounds"""
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        return input_vel
    
    def check_linear_limit_velocity(self, velocity):
        """Check and limit linear velocity"""
        return self.constrain(velocity, -self.max_linear_speed, self.max_linear_speed)
    
    def check_angular_limit_velocity(self, velocity):
        """Check and limit angular velocity"""
        return self.constrain(velocity, -self.max_angular_speed, self.max_angular_speed)
    
    def make_simple_profile(self, output, input_vel, slop):
        """Create smooth velocity profile"""
        if input_vel > output:
            output = min(input_vel, output + slop)
        elif input_vel < output:
            output = max(input_vel, output - slop)
        else:
            output = input_vel
        return output
    
    def run(self):
        """Main teleop loop"""
        target_linear_velocity = 0.0
        target_angular_velocity = 0.0
        control_linear_velocity = 0.0
        control_angular_velocity = 0.0
        
        try:
            while True:
                key = self.get_key()
                
                if key == 'i':
                    target_linear_velocity = self.check_linear_limit_velocity(target_linear_velocity + self.speed_increment)
                elif key == ',':
                    target_linear_velocity = self.check_linear_limit_velocity(target_linear_velocity - self.speed_increment)
                elif key == 'j':
                    target_angular_velocity = self.check_angular_limit_velocity(target_angular_velocity + self.speed_increment)
                elif key == 'l':
                    target_angular_velocity = self.check_angular_limit_velocity(target_angular_velocity - self.speed_increment)
                elif key == 'u':
                    target_linear_velocity = self.check_linear_limit_velocity(target_linear_velocity + self.speed_increment)
                    target_angular_velocity = self.check_angular_limit_velocity(target_angular_velocity + self.speed_increment)
                elif key == 'o':
                    target_linear_velocity = self.check_linear_limit_velocity(target_linear_velocity + self.speed_increment)
                    target_angular_velocity = self.check_angular_limit_velocity(target_angular_velocity - self.speed_increment)
                elif key == 'm':
                    target_linear_velocity = self.check_linear_limit_velocity(target_linear_velocity - self.speed_increment)
                    target_angular_velocity = self.check_angular_limit_velocity(target_angular_velocity + self.speed_increment)
                elif key == '.':
                    target_linear_velocity = self.check_linear_limit_velocity(target_linear_velocity - self.speed_increment)
                    target_angular_velocity = self.check_angular_limit_velocity(target_angular_velocity - self.speed_increment)
                elif key == ' ' or key == 'k':
                    target_linear_velocity = 0.0
                    control_linear_velocity = 0.0
                    target_angular_velocity = 0.0
                    control_angular_velocity = 0.0
                elif key == 'q':
                    self.max_linear_speed = self.max_linear_speed * 1.1
                    self.max_angular_speed = self.max_angular_speed * 1.1
                    self.get_logger().info(f'Max speeds increased - Linear: {self.max_linear_speed:.2f}, Angular: {self.max_angular_speed:.2f}')
                elif key == 'z':
                    self.max_linear_speed = self.max_linear_speed * 0.9
                    self.max_angular_speed = self.max_angular_speed * 0.9
                    self.get_logger().info(f'Max speeds decreased - Linear: {self.max_linear_speed:.2f}, Angular: {self.max_angular_speed:.2f}')
                elif key == 'w':
                    self.max_linear_speed = self.max_linear_speed * 1.1
                    self.get_logger().info(f'Max linear speed increased: {self.max_linear_speed:.2f}')
                elif key == 'x':
                    self.max_linear_speed = self.max_linear_speed * 0.9
                    self.get_logger().info(f'Max linear speed decreased: {self.max_linear_speed:.2f}')
                elif key == 'e':
                    self.max_angular_speed = self.max_angular_speed * 1.1
                    self.get_logger().info(f'Max angular speed increased: {self.max_angular_speed:.2f}')
                elif key == 'c':
                    self.max_angular_speed = self.max_angular_speed * 0.9
                    self.get_logger().info(f'Max angular speed decreased: {self.max_angular_speed:.2f}')
                else:
                    if key == '\x03':  # Ctrl+C
                        break
                
                # Create smooth velocity profiles
                control_linear_velocity = self.make_simple_profile(
                    control_linear_velocity, target_linear_velocity, 0.01)
                control_angular_velocity = self.make_simple_profile(
                    control_angular_velocity, target_angular_velocity, 0.1)
                
                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = control_linear_velocity
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = control_angular_velocity
                
                self.cmd_vel_pub.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
        
        finally:
            # Send stop command
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_keyboard = TeleopKeyboard()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

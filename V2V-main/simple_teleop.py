#!/usr/bin/env python3
"""
Simple Robot Teleop Control
Use keyboard to control your differential drive robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

class SimpleTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        
        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Speed settings
        self.linear_speed = 0.3   # m/s
        self.angular_speed = 0.8  # rad/s
        self.speed_increment = 0.1
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Simple Robot Teleop Started!')
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("🤖 ROBOT TELEOP CONTROL")
        print("="*50)
        print("Movement Controls:")
        print("  w : Move Forward")
        print("  s : Move Backward") 
        print("  a : Turn Left")
        print("  d : Turn Right")
        print("  x : Stop Robot")
        print("\nSpeed Controls:")
        print("  q : Increase Linear Speed")
        print("  z : Decrease Linear Speed")
        print("  e : Increase Angular Speed")
        print("  c : Decrease Angular Speed")
        print("\nOther:")
        print("  SPACE : Emergency Stop")
        print("  h : Show this help")
        print("  CTRL+C : Quit")
        print("="*50)
        print(f"Current Speeds - Linear: {self.linear_speed:.1f} m/s, Angular: {self.angular_speed:.1f} rad/s")
        print("Press keys to control the robot...")
        
    def get_key(self):
        """Get a single keypress"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def publish_twist(self, linear, angular):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)
        
        # Update current velocities
        self.current_linear = linear
        self.current_angular = angular
        
        # Print status
        if linear != 0 or angular != 0:
            direction = ""
            if linear > 0:
                direction += "Forward "
            elif linear < 0:
                direction += "Backward "
            if angular > 0:
                direction += "Left"
            elif angular < 0:
                direction += "Right"
            if direction == "":
                direction = "Stopped"
                
            print(f"🚀 {direction} | Linear: {linear:.1f} m/s, Angular: {angular:.1f} rad/s")
        else:
            print("🛑 Robot Stopped")
            
    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x03':  # Ctrl+C
                    break
                    
                elif key.lower() == 'w':
                    # Move forward
                    self.publish_twist(self.linear_speed, 0.0)
                    
                elif key.lower() == 's':
                    # Move backward
                    self.publish_twist(-self.linear_speed, 0.0)
                    
                elif key.lower() == 'a':
                    # Turn left
                    self.publish_twist(0.0, self.angular_speed)
                    
                elif key.lower() == 'd':
                    # Turn right
                    self.publish_twist(0.0, -self.angular_speed)
                    
                elif key.lower() == 'x' or key == ' ':
                    # Stop
                    self.publish_twist(0.0, 0.0)
                    
                elif key.lower() == 'q':
                    # Increase linear speed
                    self.linear_speed = min(1.0, self.linear_speed + self.speed_increment)
                    print(f"📈 Linear speed increased to {self.linear_speed:.1f} m/s")
                    
                elif key.lower() == 'z':
                    # Decrease linear speed
                    self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
                    print(f"📉 Linear speed decreased to {self.linear_speed:.1f} m/s")
                    
                elif key.lower() == 'e':
                    # Increase angular speed
                    self.angular_speed = min(2.0, self.angular_speed + self.speed_increment)
                    print(f"📈 Angular speed increased to {self.angular_speed:.1f} rad/s")
                    
                elif key.lower() == 'c':
                    # Decrease angular speed
                    self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
                    print(f"📉 Angular speed decreased to {self.angular_speed:.1f} rad/s")
                    
                elif key.lower() == 'h':
                    # Help
                    self.print_instructions()
                    
        except KeyboardInterrupt:
            pass
        finally:
            # Stop robot and restore terminal
            self.publish_twist(0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\n🛑 Robot stopped. Goodbye!")

def main():
    rclpy.init()
    
    try:
        teleop = SimpleTeleop()
        teleop.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

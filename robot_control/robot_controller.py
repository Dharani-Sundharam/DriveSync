#!/usr/bin/env python3
"""
Standalone Robot Controller with Real-time Mapping
Controls differential drive robot via serial and displays real-time position
"""

import serial
import time
import threading
import math
import pygame
import sys
import logging
from collections import deque
from datetime import datetime

class RobotController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=57600, log_level=logging.INFO):
        # Setup logging
        self.setup_logging(log_level)
        self.logger = logging.getLogger('RobotController')
        
        # Robot specifications based on your measurements
        self.MOTOR_A_TICKS_PER_REV = 4993  # Left motor
        self.MOTOR_B_TICKS_PER_REV = 4966  # Right motor
        self.WHEEL_DIAMETER = 0.05  # 5 cm in meters
        self.WHEEL_RADIUS = self.WHEEL_DIAMETER / 2
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.WHEEL_BASE = 0.20  # Estimate - distance between wheels (20cm)
        
        # Calculate meters per tick for each motor
        self.MOTOR_A_METERS_PER_TICK = self.WHEEL_CIRCUMFERENCE / self.MOTOR_A_TICKS_PER_REV
        self.MOTOR_B_METERS_PER_TICK = self.WHEEL_CIRCUMFERENCE / self.MOTOR_B_TICKS_PER_REV
        
        self.logger.info(f"Robot Controller initializing...")
        self.logger.info(f"Motor A: {self.MOTOR_A_TICKS_PER_REV} ticks/rev, {self.MOTOR_A_METERS_PER_TICK:.6f} m/tick")
        self.logger.info(f"Motor B: {self.MOTOR_B_TICKS_PER_REV} ticks/rev, {self.MOTOR_B_METERS_PER_TICK:.6f} m/tick")
        
        # Serial connection
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.connect_to_arduino()
        
        # Robot state
        self.x = 0.0  # Position in meters
        self.y = 0.0
        self.theta = 0.0  # Orientation in radians
        
        # Encoder tracking
        self.last_encoder_a = 0
        self.last_encoder_b = 0
        self.encoder_a = 0
        self.encoder_b = 0
        
        # Motor speeds
        self.left_speed = 0
        self.right_speed = 0
        self.max_speed = 200
        
        # Threading
        self.running = True
        self.encoder_thread = None
        
        # Path tracking for visualization
        self.path = deque(maxlen=1000)  # Store last 1000 positions
        
        self.logger.info("🤖 Robot Controller Initialized")
        print(f"🤖 Robot Controller Initialized")
        print(f"Motor A: {self.MOTOR_A_TICKS_PER_REV} ticks/rev, {self.MOTOR_A_METERS_PER_TICK:.6f} m/tick")
        print(f"Motor B: {self.MOTOR_B_TICKS_PER_REV} ticks/rev, {self.MOTOR_B_METERS_PER_TICK:.6f} m/tick")
        print(f"Wheel diameter: {self.WHEEL_DIAMETER*100:.1f} cm")
        
    def setup_logging(self, log_level):
        """Setup logging configuration"""
        # Create logs directory if it doesn't exist
        import os
        if not os.path.exists('logs'):
            os.makedirs('logs')
            
        # Create timestamp for log file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f'logs/robot_controller_{timestamp}.log'
        
        # Configure logging
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_filename),
                logging.StreamHandler(sys.stdout)
            ]
        )
        
        print(f"📝 Logging to: {log_filename}")
        
    def connect_to_arduino(self):
        """Connect to Arduino via serial"""
        self.logger.info(f"Attempting to connect to Arduino on {self.port} at {self.baudrate} baud")
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            self.logger.info("Serial connection established, waiting for Arduino reset...")
            
            # Reset encoders
            self.serial_port.write(b'r\r')
            response = self.serial_port.readline().decode().strip()
            self.logger.info(f"✅ Connected to Arduino on {self.port}")
            self.logger.info(f"Reset response: {response}")
            print(f"✅ Connected to Arduino on {self.port}")
            print(f"Reset response: {response}")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to connect to Arduino: {e}")
            print(f"❌ Failed to connect to Arduino: {e}")
            self.serial_port = None
            
    def send_motor_command(self, left_speed, right_speed):
        """Send motor speed command to Arduino"""
        if self.serial_port:
            try:
                command = f'm {int(left_speed)} {int(right_speed)}\r'
                self.serial_port.write(command.encode())
                self.left_speed = left_speed
                self.right_speed = right_speed
                self.logger.debug(f"Motor command sent: Left={left_speed}, Right={right_speed}")
                return True
            except Exception as e:
                self.logger.error(f"Error sending motor command: {e}")
                print(f"Error sending motor command: {e}")
                return False
        else:
            self.logger.warning("Cannot send motor command: No serial connection")
            return False
        
    def read_encoders(self):
        """Read encoder values from Arduino"""
        if self.serial_port:
            try:
                self.serial_port.write(b'e\r')
                response = self.serial_port.readline().decode().strip()
                self.logger.debug(f"Encoder response: '{response}'")
                
                if response and ' ' in response:
                    parts = response.split()
                    if len(parts) >= 2:
                        try:
                            new_encoder_a = int(parts[0])  # Left motor
                            new_encoder_b = int(parts[1])  # Right motor
                            
                            # Log significant encoder changes
                            if hasattr(self, 'encoder_a') and hasattr(self, 'encoder_b'):
                                delta_a = new_encoder_a - self.encoder_a
                                delta_b = new_encoder_b - self.encoder_b
                                if abs(delta_a) > 10 or abs(delta_b) > 10:
                                    self.logger.debug(f"Encoder change: A={delta_a}, B={delta_b}")
                            
                            self.encoder_a = new_encoder_a
                            self.encoder_b = new_encoder_b
                            return True
                        except ValueError as e:
                            self.logger.warning(f"Invalid encoder values in response: '{response}' - {e}")
                else:
                    self.logger.warning(f"Invalid encoder response format: '{response}'")
            except Exception as e:
                self.logger.error(f"Error reading encoders: {e}")
                print(f"Error reading encoders: {e}")
        else:
            self.logger.warning("Cannot read encoders: No serial connection")
        return False
        
    def update_odometry(self):
        """Update robot position based on encoder changes"""
        # Calculate encoder deltas
        delta_a = self.encoder_a - self.last_encoder_a  # Left motor (Motor A)
        delta_b = self.encoder_b - self.last_encoder_b  # Right motor (Motor B)
        
        # Log significant movements
        if abs(delta_a) > 5 or abs(delta_b) > 5:
            self.logger.debug(f"Odometry update - Encoder deltas: A={delta_a}, B={delta_b}")
        
        # Convert to distance traveled by each wheel
        distance_left = delta_a * self.MOTOR_A_METERS_PER_TICK   # Left wheel
        distance_right = delta_b * self.MOTOR_B_METERS_PER_TICK  # Right wheel
        
        # Calculate robot motion (differential drive kinematics)
        distance_center = (distance_left + distance_right) / 2.0
        delta_theta = (distance_right - distance_left) / self.WHEEL_BASE
        
        # Log movement calculations
        if abs(distance_center) > 0.001 or abs(delta_theta) > 0.01:
            self.logger.debug(f"Movement: center={distance_center:.4f}m, rotation={math.degrees(delta_theta):.2f}°")
        
        # Store old position for comparison
        old_x, old_y, old_theta = self.x, self.y, self.theta
        
        # Update position using improved odometry
        if abs(delta_theta) < 1e-6:  # Straight line motion
            self.x += distance_center * math.cos(self.theta)
            self.y += distance_center * math.sin(self.theta)
        else:  # Curved motion
            radius = distance_center / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Log position changes
        pos_change = math.sqrt((self.x - old_x)**2 + (self.y - old_y)**2)
        if pos_change > 0.005:  # 5mm threshold
            self.logger.info(f"Position: ({self.x:.3f}, {self.y:.3f})m, θ={math.degrees(self.theta):.1f}°")
            
        # Store position for path visualization (only if robot moved significantly)
        if abs(delta_a) > 1 or abs(delta_b) > 1:  # Only add if robot moved
            self.path.append((self.x, self.y))
        
        # Update last encoder values
        self.last_encoder_a = self.encoder_a
        self.last_encoder_b = self.encoder_b
        
    def encoder_loop(self):
        """Continuously read encoders and update odometry"""
        while self.running:
            if self.read_encoders():
                self.update_odometry()
            time.sleep(0.05)  # 20 Hz
            
    def start_encoder_thread(self):
        """Start encoder reading thread"""
        self.encoder_thread = threading.Thread(target=self.encoder_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()
        
    def stop_motors(self):
        """Stop all motors"""
        self.send_motor_command(0, 0)
        
    def cleanup(self):
        """Clean up resources"""
        self.logger.info("Starting robot controller cleanup...")
        self.running = False
        self.stop_motors()
        if self.serial_port:
            self.serial_port.close()
            self.logger.info("Serial port closed")
        self.logger.info("🛑 Robot controller cleaned up")
        print("🛑 Robot controller cleaned up")

class RobotVisualizer:
    def __init__(self, robot_controller):
        self.robot = robot_controller
        
        # Initialize Pygame
        pygame.init()
        self.width = 1200
        self.height = 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Robot Real-time Map")
        
        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.YELLOW = (255, 255, 0)
        self.GRAY = (128, 128, 128)
        self.ORANGE = (255, 165, 0)
        
        # Map parameters
        self.scale = 1000  # pixels per meter (1000 pixels = 1 meter)
        self.center_x = self.width // 2
        self.center_y = self.height // 2
        
        # Font for text
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 24)
        
        # Control state
        self.keys_pressed = set()
        
        print("🖥️  Real-time map visualization started")
        print("🎮 Controls:")
        print("   W/S - Forward/Backward")
        print("   A/D - Turn Left/Right")
        print("   Q/E - Increase/Decrease Speed")
        print("   SPACE - Stop")
        print("   ESC - Quit")
        
    def world_to_screen(self, x, y):
        """Convert world coordinates to screen coordinates"""
        screen_x = int(self.center_x + x * self.scale)
        screen_y = int(self.center_y - y * self.scale)  # Flip Y axis
        return screen_x, screen_y
        
    def draw_robot(self):
        """Draw the robot on the screen"""
        # Get robot position in screen coordinates
        robot_screen_x, robot_screen_y = self.world_to_screen(self.robot.x, self.robot.y)
        
        # Robot dimensions in pixels
        robot_length = int(0.15 * self.scale)  # 15cm robot
        robot_width = int(0.10 * self.scale)   # 10cm robot
        
        # Calculate robot corners
        cos_theta = math.cos(self.robot.theta)
        sin_theta = math.sin(self.robot.theta)
        
        # Robot body (rectangle)
        half_length = robot_length // 2
        half_width = robot_width // 2
        
        corners = [
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width)
        ]
        
        # Rotate and translate corners
        rotated_corners = []
        for dx, dy in corners:
            rotated_x = dx * cos_theta - dy * sin_theta
            rotated_y = dx * sin_theta + dy * cos_theta
            screen_x = robot_screen_x + rotated_x
            screen_y = robot_screen_y - rotated_y  # Flip Y
            rotated_corners.append((screen_x, screen_y))
            
        # Draw robot body
        pygame.draw.polygon(self.screen, self.BLUE, rotated_corners)
        
        # Draw direction arrow
        arrow_length = robot_length * 0.8
        arrow_end_x = robot_screen_x + arrow_length * cos_theta
        arrow_end_y = robot_screen_y - arrow_length * sin_theta  # Flip Y
        pygame.draw.line(self.screen, self.YELLOW, 
                        (robot_screen_x, robot_screen_y), 
                        (arrow_end_x, arrow_end_y), 3)
        
        # Draw center point
        pygame.draw.circle(self.screen, self.RED, (robot_screen_x, robot_screen_y), 3)
        
    def draw_path(self):
        """Draw the robot's path"""
        if len(self.robot.path) > 1:
            screen_points = []
            for x, y in self.robot.path:
                screen_x, screen_y = self.world_to_screen(x, y)
                screen_points.append((screen_x, screen_y))
            
            # Draw path as connected lines
            if len(screen_points) > 1:
                pygame.draw.lines(self.screen, self.GREEN, False, screen_points, 2)
                
    def draw_grid(self):
        """Draw coordinate grid"""
        # Draw grid lines every 10cm
        grid_spacing = 0.1 * self.scale  # 10cm in pixels
        
        # Vertical lines
        for i in range(-15, 16):
            x = self.center_x + i * grid_spacing
            if 0 <= x <= self.width:
                if i == 0:
                    color = self.WHITE
                    width = 2
                elif i % 5 == 0:  # Every 50cm
                    color = (100, 100, 100)
                    width = 1
                else:
                    color = (50, 50, 50)
                    width = 1
                pygame.draw.line(self.screen, color, (x, 0), (x, self.height), width)
                
        # Horizontal lines  
        for i in range(-15, 16):
            y = self.center_y + i * grid_spacing
            if 0 <= y <= self.height:
                if i == 0:
                    color = self.WHITE
                    width = 2
                elif i % 5 == 0:  # Every 50cm
                    color = (100, 100, 100)
                    width = 1
                else:
                    color = (50, 50, 50)
                    width = 1
                pygame.draw.line(self.screen, color, (0, y), (self.width, y), width)
                
        # Add grid labels
        for i in range(-10, 11, 5):  # Every 50cm
            if i != 0:
                # X-axis labels
                x = self.center_x + i * grid_spacing
                if 0 <= x <= self.width:
                    label = self.small_font.render(f"{i*10}cm", True, (150, 150, 150))
                    self.screen.blit(label, (x - 15, self.center_y + 5))
                
                # Y-axis labels
                y = self.center_y + i * grid_spacing
                if 0 <= y <= self.height:
                    label = self.small_font.render(f"{-i*10}cm", True, (150, 150, 150))
                    self.screen.blit(label, (self.center_x + 5, y - 10))
                
    def draw_info(self):
        """Draw robot information"""
        info_lines = [
            f"Position: ({self.robot.x:.3f}, {self.robot.y:.3f}) m",
            f"Angle: {math.degrees(self.robot.theta):.1f}°",
            f"Motors: L={self.robot.left_speed}, R={self.robot.right_speed}",
            f"Encoders: A={self.robot.encoder_a}, B={self.robot.encoder_b}",
            f"Distance: {math.sqrt(self.robot.x**2 + self.robot.y**2):.3f} m"
        ]
        
        y_offset = 10
        for line in info_lines:
            text = self.small_font.render(line, True, self.WHITE)
            self.screen.blit(text, (10, y_offset))
            y_offset += 25
            
        # Control instructions
        controls = [
            "Controls:",
            "W/S - Forward/Back",
            "A/D - Turn L/R", 
            "Q/E - Speed +/-",
            "SPACE - Stop",
            "ESC - Quit"
        ]
        
        y_offset = self.height - 150
        for line in controls:
            text = self.small_font.render(line, True, self.YELLOW)
            self.screen.blit(text, (10, y_offset))
            y_offset += 20
            
    def handle_keyboard(self):
        """Handle keyboard input for robot control"""
        base_speed = 120
        turn_speed = 100
        
        # Get current key states
        keys = pygame.key.get_pressed()
        
        left_speed = 0   # Motor A
        right_speed = 0  # Motor B
        command_type = "STOP"
        
        # Movement commands (only one at a time for clearer control)
        if keys[pygame.K_w]:  # Forward
            left_speed = base_speed
            right_speed = base_speed
            command_type = "FORWARD"
        elif keys[pygame.K_s]:  # Backward
            left_speed = -base_speed
            right_speed = -base_speed
            command_type = "BACKWARD"
        elif keys[pygame.K_a]:  # Turn left (counter-clockwise)
            left_speed = -turn_speed  # Left motor backward
            right_speed = turn_speed  # Right motor forward
            command_type = "TURN_LEFT"
        elif keys[pygame.K_d]:  # Turn right (clockwise)
            left_speed = turn_speed   # Left motor forward
            right_speed = -turn_speed # Right motor backward
            command_type = "TURN_RIGHT"
            
        # Stop
        if keys[pygame.K_SPACE]:
            left_speed = 0
            right_speed = 0
            command_type = "STOP"
        
        # Log command changes
        if hasattr(self, 'last_command_type') and self.last_command_type != command_type:
            self.robot.logger.info(f"Command change: {command_type} (L={left_speed}, R={right_speed})")
        self.last_command_type = command_type
            
        # Send motor commands
        self.robot.send_motor_command(left_speed, right_speed)
        
    def run(self):
        """Main visualization loop"""
        clock = pygame.time.Clock()
        
        while True:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return
                        
            # Handle keyboard input
            self.handle_keyboard()
            
            # Clear screen
            self.screen.fill(self.BLACK)
            
            # Draw everything
            self.draw_grid()
            self.draw_path()
            self.draw_robot()
            self.draw_info()
            
            # Update display
            pygame.display.flip()
            clock.tick(30)  # 30 FPS

def main():
    print("🚀 Starting Robot Control System")
    
    # Initialize robot controller with debug logging
    robot = RobotController(log_level=logging.DEBUG)
    
    if robot.serial_port is None:
        robot.logger.error("❌ Cannot connect to Arduino. Exiting.")
        print("❌ Cannot connect to Arduino. Exiting.")
        return
        
    try:
        robot.logger.info("Starting encoder reading thread...")
        # Start encoder reading
        robot.start_encoder_thread()
        
        robot.logger.info("Starting visualization...")
        # Start visualization
        visualizer = RobotVisualizer(robot)
        visualizer.run()
        
    except KeyboardInterrupt:
        robot.logger.info("⚠️  Interrupted by user")
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        robot.logger.error(f"❌ Error: {e}", exc_info=True)
        print(f"❌ Error: {e}")
    finally:
        robot.cleanup()
        pygame.quit()
        robot.logger.info("👋 Application shutdown complete")
        print("👋 Goodbye!")

if __name__ == "__main__":
    main()

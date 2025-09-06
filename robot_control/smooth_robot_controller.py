#!/usr/bin/env python3
"""
Optimized Smooth Robot Controller with Real-time Mapping
- High-performance serial communication
- Smooth 60 FPS visualization
- Minimal latency motor control
- Clean data processing
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
import queue

class OptimizedRobotController:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        # Setup logging (less verbose for performance)
        self.setup_logging(logging.INFO)
        self.logger = logging.getLogger('OptimizedRobot')
        
        # Robot specifications - YOUR EXACT MEASUREMENTS
        self.MOTOR_A_TICKS_PER_REV = 4993  # Left motor
        self.MOTOR_B_TICKS_PER_REV = 4966  # Right motor
        self.WHEEL_DIAMETER = 0.05  # 5 cm in meters
        self.WHEEL_RADIUS = self.WHEEL_DIAMETER / 2
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.WHEEL_BASE = 0.15  # Estimated 15cm between wheel centers (adjust as needed)
        
        # Calculate meters per tick for each wheel
        self.LEFT_METERS_PER_TICK = self.WHEEL_CIRCUMFERENCE / self.MOTOR_A_TICKS_PER_REV
        self.RIGHT_METERS_PER_TICK = self.WHEEL_CIRCUMFERENCE / self.MOTOR_B_TICKS_PER_REV
        
        # Differential drive calculations
        self.TICKS_PER_METER_LEFT = self.MOTOR_A_TICKS_PER_REV / self.WHEEL_CIRCUMFERENCE
        self.TICKS_PER_METER_RIGHT = self.MOTOR_B_TICKS_PER_REV / self.WHEEL_CIRCUMFERENCE
        
        # For one full rotation (360°) calculations
        self.CIRCUMFERENCE_FOR_ROTATION = math.pi * self.WHEEL_BASE  # Distance each wheel travels for 360° turn
        self.TICKS_FOR_360_TURN_LEFT = self.CIRCUMFERENCE_FOR_ROTATION * self.TICKS_PER_METER_LEFT
        self.TICKS_FOR_360_TURN_RIGHT = self.CIRCUMFERENCE_FOR_ROTATION * self.TICKS_PER_METER_RIGHT
        
        # Serial connection
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.connect_to_arduino()
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Encoder tracking
        self.encoder_a = 0
        self.encoder_b = 0
        self.last_encoder_a = 0
        self.last_encoder_b = 0
        
        # Motor speeds
        self.left_speed = 0
        self.right_speed = 0
        
        # Threading and communication
        self.running = True
        self.data_queue = queue.Queue(maxsize=100)
        self.command_queue = queue.Queue(maxsize=50)
        
        # Path tracking
        self.path = deque(maxlen=500)  # Smaller for performance
        
        # Performance tracking
        self.last_update_time = time.time()
        self.update_count = 0
        self.fps = 0
        
        self.logger.info("🚀 Optimized Robot Controller Initialized")
        self.logger.info(f"📐 Differential Drive Specifications:")
        self.logger.info(f"   Wheel diameter: {self.WHEEL_DIAMETER*100:.1f} cm")
        self.logger.info(f"   Wheel base: {self.WHEEL_BASE*100:.1f} cm") 
        self.logger.info(f"   Left motor: {self.MOTOR_A_TICKS_PER_REV} ticks/rev")
        self.logger.info(f"   Right motor: {self.MOTOR_B_TICKS_PER_REV} ticks/rev")
        self.logger.info(f"   Left resolution: {self.LEFT_METERS_PER_TICK*1000:.3f} mm/tick")
        self.logger.info(f"   Right resolution: {self.RIGHT_METERS_PER_TICK*1000:.3f} mm/tick")
        self.logger.info(f"   360° turn needs: L={self.TICKS_FOR_360_TURN_LEFT:.0f} ticks, R={self.TICKS_FOR_360_TURN_RIGHT:.0f} ticks")
        
    def setup_logging(self, log_level):
        """Setup optimized logging"""
        import os
        if not os.path.exists('logs'):
            os.makedirs('logs')
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f'logs/smooth_robot_{timestamp}.log'
        
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_filename),
                logging.StreamHandler(sys.stdout)
            ]
        )
        
    def connect_to_arduino(self):
        """Connect to Arduino with optimized settings"""
        try:
            self.serial_port = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=0.01,  # Very short timeout for responsiveness
                write_timeout=0.01
            )
            time.sleep(2)  # Wait for Arduino reset
            
            # Wait for READY signal
            start_time = time.time()
            while time.time() - start_time < 5:  # 5 second timeout
                if self.serial_port.in_waiting:
                    response = self.serial_port.readline().decode().strip()
                    if response == "READY":
                        self.logger.info("✅ Arduino ready and optimized")
                        print("✅ Arduino ready and optimized")
                        return
                        
            self.logger.warning("Arduino didn't send READY signal")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to connect: {e}")
            self.serial_port = None
            
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command with minimal latency"""
        if self.serial_port and not self.command_queue.full():
            command = f'm {int(left_speed)} {int(right_speed)}\r'
            self.command_queue.put(command)
            self.left_speed = left_speed
            self.right_speed = right_speed
            return True
        return False
        
    def communication_thread(self):
        """High-performance communication thread"""
        encoder_request_time = time.time()
        
        while self.running:
            try:
                # Send queued commands
                while not self.command_queue.empty() and self.serial_port:
                    try:
                        command = self.command_queue.get_nowait()
                        self.serial_port.write(command.encode())
                        self.serial_port.flush()  # Force immediate send
                    except queue.Empty:
                        break
                    except Exception as e:
                        self.logger.warning(f"Command send error: {e}")
                
                # Read incoming data
                if self.serial_port and self.serial_port.in_waiting:
                    try:
                        line = self.serial_port.readline().decode().strip()
                        if line and ' ' in line and not line.startswith('OK'):
                            # Only process encoder data
                            parts = line.split()
                            if len(parts) == 2:
                                try:
                                    encoder_a = int(parts[0])
                                    encoder_b = int(parts[1])
                                    if not self.data_queue.full():
                                        self.data_queue.put((encoder_a, encoder_b))
                                except ValueError:
                                    pass  # Ignore invalid data silently
                    except Exception as e:
                        self.logger.debug(f"Read error: {e}")
                
                # Request encoder data every 20ms (50Hz)
                current_time = time.time()
                if current_time - encoder_request_time >= 0.02:
                    if self.serial_port:
                        try:
                            self.serial_port.write(b'e\r')
                            encoder_request_time = current_time
                        except:
                            pass
                
                time.sleep(0.001)  # 1ms sleep for CPU efficiency
                
            except Exception as e:
                self.logger.error(f"Communication thread error: {e}")
                time.sleep(0.01)
                
    def update_odometry(self, new_encoder_a, new_encoder_b):
        """Accurate differential drive odometry with your exact specifications"""
        # Calculate encoder deltas
        delta_left = new_encoder_a - self.last_encoder_a   # Left wheel (Motor A)
        delta_right = new_encoder_b - self.last_encoder_b  # Right wheel (Motor B)
        
        # Only update if there's movement
        if abs(delta_left) < 1 and abs(delta_right) < 1:
            return
            
        # Convert encoder ticks to distances traveled by each wheel
        distance_left = delta_left * self.LEFT_METERS_PER_TICK
        distance_right = delta_right * self.RIGHT_METERS_PER_TICK
        
        # Differential drive kinematics
        # Linear velocity (forward/backward motion)
        distance_center = (distance_left + distance_right) / 2.0
        
        # Angular velocity (rotation)
        delta_theta = (distance_right - distance_left) / self.WHEEL_BASE
        
        # Calculate new position using differential drive equations
        if abs(delta_theta) < 1e-6:
            # Straight line motion (no rotation)
            dx = distance_center * math.cos(self.theta)
            dy = distance_center * math.sin(self.theta)
        else:
            # Curved motion (arc)
            # Calculate instantaneous center of curvature
            radius = distance_center / delta_theta
            
            # Calculate position change
            dx = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            dy = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        # Update robot position
        self.x += dx
        self.y += dy
        self.theta += delta_theta
        
        # Normalize angle to [-π, π]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
            
        # Add to path for visualization (only if significant movement)
        if abs(distance_center) > 0.001 or abs(delta_theta) > 0.01:  # 1mm or 0.57°
            self.path.append((self.x, self.y))
        
        # Update last encoder values
        self.encoder_a = new_encoder_a
        self.encoder_b = new_encoder_b
        self.last_encoder_a = new_encoder_a
        self.last_encoder_b = new_encoder_b
        
        # Debug output for significant movements
        if abs(delta_left) > 10 or abs(delta_right) > 10:
            turn_angle_deg = math.degrees(delta_theta)
            self.logger.info(f"🔄 Movement: L={delta_left} ticks ({distance_left*1000:.1f}mm), "
                           f"R={delta_right} ticks ({distance_right*1000:.1f}mm), "
                           f"Center={distance_center*1000:.1f}mm, Turn={turn_angle_deg:.1f}°")
            
            # Check for unexpected circular motion
            if abs(delta_left - delta_right) > 50 and abs(distance_center) < 0.005:
                self.logger.warning(f"⚠️  CIRCULAR MOTION DETECTED! Large wheel difference: "
                                  f"L={delta_left}, R={delta_right} (diff={delta_left-delta_right})")
        
    def data_processing_thread(self):
        """Process encoder data in separate thread"""
        while self.running:
            try:
                if not self.data_queue.empty():
                    encoder_a, encoder_b = self.data_queue.get(timeout=0.01)
                    self.update_odometry(encoder_a, encoder_b)
                    
                    # Update FPS counter
                    self.update_count += 1
                    current_time = time.time()
                    if current_time - self.last_update_time >= 1.0:
                        self.fps = self.update_count
                        self.update_count = 0
                        self.last_update_time = current_time
                else:
                    time.sleep(0.005)  # 5ms sleep when no data
                    
            except queue.Empty:
                time.sleep(0.005)
            except Exception as e:
                self.logger.error(f"Data processing error: {e}")
                
    def start_threads(self):
        """Start all background threads"""
        self.comm_thread = threading.Thread(target=self.communication_thread, daemon=True)
        self.data_thread = threading.Thread(target=self.data_processing_thread, daemon=True)
        
        self.comm_thread.start()
        self.data_thread.start()
        
        self.logger.info("🔄 High-performance threads started")
        
    def cleanup(self):
        """Clean up resources"""
        self.logger.info("🛑 Cleaning up...")
        self.running = False
        
        # Stop motors
        if self.serial_port:
            try:
                self.serial_port.write(b's\r')
                time.sleep(0.1)
                self.serial_port.close()
            except:
                pass
                
        print("🛑 Optimized robot controller cleaned up")

class SmoothVisualizer:
    def __init__(self, robot_controller):
        self.robot = robot_controller
        
        # Initialize Pygame with optimized settings
        pygame.init()
        self.width = 1400
        self.height = 1000
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Smooth Real-time Robot Control")
        
        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 100, 255)
        self.YELLOW = (255, 255, 0)
        self.GRAY = (64, 64, 64)
        self.LIGHT_GRAY = (128, 128, 128)
        
        # Map parameters
        self.scale = 1000  # pixels per meter
        self.center_x = self.width // 2
        self.center_y = self.height // 2
        
        # Fonts
        self.font = pygame.font.Font(None, 32)
        self.small_font = pygame.font.Font(None, 24)
        
        # Control state
        self.current_command = "STOP"
        
        print("🖥️  Smooth 60 FPS visualization ready")
        
    def world_to_screen(self, x, y):
        """Convert world coordinates to screen coordinates"""
        screen_x = int(self.center_x + x * self.scale)
        screen_y = int(self.center_y - y * self.scale)
        return screen_x, screen_y
        
    def draw_grid(self):
        """Draw optimized grid"""
        grid_spacing = 0.1 * self.scale  # 10cm
        
        # Only draw visible grid lines
        start_x = max(0, int(self.center_x - 10 * grid_spacing))
        end_x = min(self.width, int(self.center_x + 10 * grid_spacing))
        start_y = max(0, int(self.center_y - 6 * grid_spacing))
        end_y = min(self.height, int(self.center_y + 6 * grid_spacing))
        
        # Vertical lines
        for i in range(-10, 11):
            x = self.center_x + i * grid_spacing
            if start_x <= x <= end_x:
                color = self.WHITE if i == 0 else self.GRAY
                width = 2 if i == 0 else 1
                pygame.draw.line(self.screen, color, (x, start_y), (x, end_y), width)
                
        # Horizontal lines
        for i in range(-6, 7):
            y = self.center_y + i * grid_spacing
            if start_y <= y <= end_y:
                color = self.WHITE if i == 0 else self.GRAY
                width = 2 if i == 0 else 1
                pygame.draw.line(self.screen, color, (start_x, y), (end_x, y), width)
                
    def draw_robot(self):
        """Draw robot with differential drive visualization"""
        robot_x, robot_y = self.world_to_screen(self.robot.x, self.robot.y)
        
        # Robot body
        robot_size = 24
        cos_theta = math.cos(self.robot.theta)
        sin_theta = math.sin(self.robot.theta)
        
        # Robot corners (rectangular body)
        corners = []
        for dx, dy in [(-robot_size//2, -robot_size//3), (robot_size//2, -robot_size//3),
                      (robot_size//2, robot_size//3), (-robot_size//2, robot_size//3)]:
            rotated_x = dx * cos_theta - dy * sin_theta
            rotated_y = dx * sin_theta + dy * cos_theta
            corners.append((robot_x + rotated_x, robot_y - rotated_y))
            
        pygame.draw.polygon(self.screen, self.BLUE, corners)
        
        # Draw wheels to show differential drive
        wheel_width = 6
        wheel_height = 12
        
        # Left wheel position
        left_wheel_x = robot_x + (-robot_size//3) * cos_theta - (-robot_size//2) * sin_theta
        left_wheel_y = robot_y - ((-robot_size//3) * sin_theta + (-robot_size//2) * cos_theta)
        
        # Right wheel position  
        right_wheel_x = robot_x + (-robot_size//3) * cos_theta - (robot_size//2) * sin_theta
        right_wheel_y = robot_y - ((-robot_size//3) * sin_theta + (robot_size//2) * cos_theta)
        
        # Draw wheels with speed indication
        left_color = self.GREEN if self.robot.left_speed > 0 else self.RED if self.robot.left_speed < 0 else self.GRAY
        right_color = self.GREEN if self.robot.right_speed > 0 else self.RED if self.robot.right_speed < 0 else self.GRAY
        
        pygame.draw.circle(self.screen, left_color, (int(left_wheel_x), int(left_wheel_y)), wheel_width)
        pygame.draw.circle(self.screen, right_color, (int(right_wheel_x), int(right_wheel_y)), wheel_width)
        
        # Direction arrow (front of robot)
        arrow_length = robot_size * 1.2
        arrow_end_x = robot_x + arrow_length * cos_theta
        arrow_end_y = robot_y - arrow_length * sin_theta
        pygame.draw.line(self.screen, self.YELLOW, (robot_x, robot_y), 
                        (arrow_end_x, arrow_end_y), 3)
        
        # Draw arrowhead
        arrow_size = 8
        arrowhead_left_x = arrow_end_x - arrow_size * math.cos(self.robot.theta - 0.5)
        arrowhead_left_y = arrow_end_y + arrow_size * math.sin(self.robot.theta - 0.5)
        arrowhead_right_x = arrow_end_x - arrow_size * math.cos(self.robot.theta + 0.5)
        arrowhead_right_y = arrow_end_y + arrow_size * math.sin(self.robot.theta + 0.5)
        
        pygame.draw.polygon(self.screen, self.YELLOW, [
            (arrow_end_x, arrow_end_y),
            (arrowhead_left_x, arrowhead_left_y),
            (arrowhead_right_x, arrowhead_right_y)
        ])
        
        # Center dot
        pygame.draw.circle(self.screen, self.WHITE, (robot_x, robot_y), 2)
        
    def draw_path(self):
        """Draw smooth path"""
        if len(self.robot.path) > 1:
            screen_points = []
            for x, y in list(self.robot.path):  # Convert to list for thread safety
                screen_x, screen_y = self.world_to_screen(x, y)
                # Only add points that are on screen
                if 0 <= screen_x <= self.width and 0 <= screen_y <= self.height:
                    screen_points.append((screen_x, screen_y))
            
            if len(screen_points) > 1:
                pygame.draw.lines(self.screen, self.GREEN, False, screen_points, 2)
                
    def draw_info(self):
        """Draw detailed differential drive info display"""
        # Calculate some useful differential drive metrics
        total_distance = math.sqrt(self.robot.x**2 + self.robot.y**2)
        
        # Calculate theoretical turns based on encoder values
        left_rotations = self.robot.encoder_a / self.robot.MOTOR_A_TICKS_PER_REV
        right_rotations = self.robot.encoder_b / self.robot.MOTOR_B_TICKS_PER_REV
        
        info_lines = [
            f"Position: ({self.robot.x:.3f}, {self.robot.y:.3f}) m",
            f"Heading: {math.degrees(self.robot.theta):.1f}°",
            f"Distance: {total_distance:.3f} m",
            f"Motors: L={self.robot.left_speed}, R={self.robot.right_speed}",
            f"Encoders: L={self.robot.encoder_a}, R={self.robot.encoder_b}",
            f"Rotations: L={left_rotations:.2f}, R={right_rotations:.2f}",
            f"Update Rate: {self.robot.fps} Hz",
            f"Command: {self.current_command}"
        ]
        
        # Differential drive specs
        specs_lines = [
            "Differential Drive Specs:",
            f"Wheel Diameter: {self.robot.WHEEL_DIAMETER*100:.1f} cm",
            f"Wheel Base: {self.robot.WHEEL_BASE*100:.1f} cm",
            f"Left: {self.robot.MOTOR_A_TICKS_PER_REV} ticks/rev",
            f"Right: {self.robot.MOTOR_B_TICKS_PER_REV} ticks/rev",
            f"L: {self.robot.LEFT_METERS_PER_TICK*1000:.3f} mm/tick",
            f"R: {self.robot.RIGHT_METERS_PER_TICK*1000:.3f} mm/tick"
        ]
        
        # Draw main info
        y_offset = 10
        for line in info_lines:
            text = self.small_font.render(line, True, self.WHITE)
            self.screen.blit(text, (10, y_offset))
            y_offset += 22
            
        # Draw specs in right column
        y_offset = 10
        for line in specs_lines:
            color = self.YELLOW if "Specs:" in line else self.LIGHT_GRAY
            text = self.small_font.render(line, True, color)
            self.screen.blit(text, (self.width - 300, y_offset))
            y_offset += 20
            
    def handle_keyboard(self):
        """Optimized keyboard handling"""
        keys = pygame.key.get_pressed()
        
        left_speed = 0
        right_speed = 0
        command = "STOP"
        
        # Movement commands - FIXED CONTROLS
        if keys[pygame.K_w]:  # Forward
            left_speed = 150   # Left motor forward
            right_speed = 150  # Right motor forward  
            command = "FORWARD"
        elif keys[pygame.K_s]:  # Backward
            left_speed = -150  # Left motor backward
            right_speed = -150 # Right motor backward
            command = "BACKWARD"
        elif keys[pygame.K_a]:  # Turn left (counterclockwise)
            left_speed = -120  # Left motor backward
            right_speed = 120  # Right motor forward
            command = "TURN_LEFT"
        elif keys[pygame.K_d]:  # Turn right (clockwise)  
            left_speed = 120   # Left motor forward
            right_speed = -120 # Right motor backward
            command = "TURN_RIGHT"
            
        # Only send command if it changed
        if command != self.current_command:
            self.current_command = command
            
            # DEBUGGING: Uncomment next line if controls are still reversed
            # left_speed, right_speed = right_speed, left_speed
            
            self.robot.send_motor_command(left_speed, right_speed)
        elif command == "STOP" and (self.robot.left_speed != 0 or self.robot.right_speed != 0):
            self.robot.send_motor_command(0, 0)
            
    def run(self):
        """Smooth 60 FPS main loop"""
        clock = pygame.time.Clock()
        
        while True:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return
                        
            # Handle input
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
            clock.tick(60)  # Smooth 60 FPS

def main():
    print("🚀 Starting Optimized Smooth Robot Controller")
    
    robot = OptimizedRobotController()
    
    if robot.serial_port is None:
        print("❌ Cannot connect to Arduino. Exiting.")
        return
        
    try:
        robot.start_threads()
        time.sleep(1)  # Let threads initialize
        
        visualizer = SmoothVisualizer(robot)
        visualizer.run()
        
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        robot.logger.error(f"❌ Error: {e}", exc_info=True)
        print(f"❌ Error: {e}")
    finally:
        robot.cleanup()
        pygame.quit()
        print("👋 Goodbye!")

if __name__ == "__main__":
    main()

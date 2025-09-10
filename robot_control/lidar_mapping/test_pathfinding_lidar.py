#!/usr/bin/env python3
"""
Test LIDAR-Enhanced Pathfinding Controller
==========================================

Test script to demonstrate the LIDAR-enhanced pathfinding controller with
obstacle detection and avoidance capabilities.
"""

import pygame
import sys
import os
import time
import math
import threading
import logging

# Add parent directory to path to import pathfinding controller
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Handle both relative and absolute imports
try:
    from .pathfinding_integration import LidarEnhancedPathfindingController, integrate_lidar_with_pathfinding
    from .ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
except ImportError:
    from pathfinding_integration import LidarEnhancedPathfindingController, integrate_lidar_with_pathfinding
    from ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig


class MockPathfindingController:
    """Mock pathfinding controller for testing"""
    
    def __init__(self, width=800, height=600):
        self.width = width
        self.height = height
        
        # Mock robot state
        self.robot = MockRobot()
        
        # Mock fonts
        pygame.font.init()
        self.small_font = pygame.font.Font(None, 20)
        self.font = pygame.font.Font(None, 24)
        
        # Control state
        self.running = True
        self.manual_control = False
        
        # Obstacle avoidance callback
        self.obstacle_avoidance_callback = None
    
    def apply_obstacle_avoidance(self, avoidance_command):
        """Apply obstacle avoidance command to robot"""
        left_speed, right_speed = avoidance_command
        
        # Convert to robot movement
        forward_speed = (left_speed + right_speed) / 2
        angular_speed = (right_speed - left_speed) / 2
        
        # Apply to mock robot
        self.robot.apply_movement(forward_speed, angular_speed)
        
        print(f"🚨 Obstacle avoidance: L={left_speed:.2f}, R={right_speed:.2f} -> F={forward_speed:.2f}, A={angular_speed:.2f}")


class MockRobot:
    """Mock robot for testing"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Movement state
        self.forward_speed = 0.0
        self.angular_speed = 0.0
        self.last_update = time.time()
    
    def apply_movement(self, forward_speed, angular_speed):
        """Apply movement commands"""
        self.forward_speed = forward_speed
        self.angular_speed = angular_speed
    
    def update_position(self):
        """Update robot position based on current movement"""
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        
        if dt > 0.1:  # Limit large time steps
            dt = 0.1
        
        # Update position
        self.x += self.forward_speed * math.cos(self.theta) * dt
        self.y += self.forward_speed * math.sin(self.theta) * dt
        self.theta += self.angular_speed * dt
        
        # Normalize angle
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi


class LidarPathfindingDemo:
    """Demo application for LIDAR pathfinding"""
    
    def __init__(self):
        pygame.init()
        
        self.width = 1000
        self.height = 700
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("LIDAR-Enhanced Pathfinding Demo")
        
        # Colors
        self.colors = {
            'background': (30, 30, 40),
            'grid': (60, 60, 70),
            'robot': (0, 255, 0),
            'robot_obstacle': (255, 100, 100),
            'lidar': (0, 200, 255),
            'obstacle': (255, 0, 0),
            'text': (255, 255, 255),
            'warning': (255, 255, 0)
        }
        
        # Create mock pathfinding controller
        self.mock_controller = MockPathfindingController(self.width, self.height)
        
        # Create LIDAR-enhanced controller
        self.enhanced_controller = LidarEnhancedPathfindingController(
            pathfinding_controller=self.mock_controller,
            lidar_port='/dev/ttyUSB1',
            enable_lidar_mapping=True
        )
        
        # Setup fonts
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        self.large_font = pygame.font.Font(None, 32)
        
        # Demo state
        self.clock = pygame.time.Clock()
        self.running = True
        
        # Robot update thread
        self.robot_thread = threading.Thread(target=self.robot_update_loop, daemon=True)
        self.robot_thread.start()
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger('LidarDemo')
    
    def robot_update_loop(self):
        """Update robot position continuously"""
        while self.running:
            try:
                self.mock_controller.robot.update_position()
                time.sleep(0.05)  # 20 Hz update
            except Exception as e:
                self.logger.error(f"Robot update error: {e}")
                time.sleep(0.1)
    
    def draw_grid(self):
        """Draw coordinate grid"""
        grid_size = 50
        
        # Vertical lines
        for x in range(0, self.width, grid_size):
            pygame.draw.line(self.screen, self.colors['grid'], (x, 0), (x, self.height), 1)
        
        # Horizontal lines  
        for y in range(0, self.height, grid_size):
            pygame.draw.line(self.screen, self.colors['grid'], (0, y), (self.width, y), 1)
    
    def world_to_screen(self, world_x, world_y):
        """Convert world coordinates to screen coordinates"""
        # Center the coordinate system
        screen_x = int(self.width / 2 + world_x * 100)  # 100 pixels per meter
        screen_y = int(self.height / 2 - world_y * 100)  # Flip Y axis
        return screen_x, screen_y
    
    def draw_robot(self):
        """Draw robot with current state"""
        robot = self.mock_controller.robot
        screen_x, screen_y = self.world_to_screen(robot.x, robot.y)
        
        # Choose color based on obstacle detection
        if self.enhanced_controller.obstacle_detected:
            robot_color = self.colors['robot_obstacle']
        else:
            robot_color = self.colors['robot']
        
        # Draw robot body (rectangle representing chassis)
        robot_length_pixels = int(self.enhanced_controller.robot_length * 100)
        robot_width_pixels = int(self.enhanced_controller.robot_width * 100)
        
        # Calculate robot corners
        cos_theta = math.cos(robot.theta)
        sin_theta = math.sin(robot.theta)
        
        # Robot corners relative to center
        half_length = robot_length_pixels / 2
        half_width = robot_width_pixels / 2
        
        corners = [
            (-half_length, -half_width),  # Back left
            (half_length, -half_width),   # Front left
            (half_length, half_width),    # Front right
            (-half_length, half_width)    # Back right
        ]
        
        # Transform corners
        screen_corners = []
        for dx, dy in corners:
            rotated_x = dx * cos_theta - dy * sin_theta
            rotated_y = dx * sin_theta + dy * cos_theta
            screen_corners.append((screen_x + rotated_x, screen_y + rotated_y))
        
        # Draw robot body
        pygame.draw.polygon(self.screen, robot_color, screen_corners, 2)
        
        # Draw LIDAR position (center of robot)
        pygame.draw.circle(self.screen, (255, 255, 0), (screen_x, screen_y), 5)
        
        # Draw direction arrow
        arrow_length = 30
        arrow_end_x = screen_x + arrow_length * cos_theta
        arrow_end_y = screen_y - arrow_length * sin_theta  # Flip Y
        pygame.draw.line(self.screen, (255, 255, 255), 
                        (screen_x, screen_y), (arrow_end_x, arrow_end_y), 2)
        
        # Draw clearance zones if obstacle detected
        if self.enhanced_controller.obstacle_detected:
            # Front clearance
            front_distance = (self.enhanced_controller.lidar_to_front + 
                            self.enhanced_controller.clearance_front) * 100
            front_x = screen_x + front_distance * cos_theta
            front_y = screen_y - front_distance * sin_theta
            pygame.draw.line(self.screen, self.colors['warning'], 
                           (screen_x, screen_y), (front_x, front_y), 1)
            
            # Side clearances
            side_distance = (self.enhanced_controller.robot_width/2 + 
                           self.enhanced_controller.clearance_sides) * 100
            
            # Left side
            left_x = screen_x - side_distance * sin_theta
            left_y = screen_y - side_distance * cos_theta
            pygame.draw.line(self.screen, self.colors['warning'],
                           (screen_x, screen_y), (left_x, left_y), 1)
            
            # Right side
            right_x = screen_x + side_distance * sin_theta
            right_y = screen_y + side_distance * cos_theta
            pygame.draw.line(self.screen, self.colors['warning'],
                           (screen_x, screen_y), (right_x, right_y), 1)
    
    def draw_lidar_data(self):
        """Draw LIDAR scan data"""
        if not self.enhanced_controller.lidar or not self.enhanced_controller.lidar_running:
            return
        
        scan = self.enhanced_controller.lidar.get_latest_scan()
        if not scan or not scan.points:
            return
        
        robot = self.mock_controller.robot
        robot_screen_x, robot_screen_y = self.world_to_screen(robot.x, robot.y)
        
        # Draw LIDAR points
        for point in scan.points:
            if 0.05 <= point.distance <= 6.0:  # Valid range
                # Convert to world coordinates
                world_x = robot.x + point.distance * math.cos(point.angle + robot.theta)
                world_y = robot.y + point.distance * math.sin(point.angle + robot.theta)
                
                screen_x, screen_y = self.world_to_screen(world_x, world_y)
                
                # Color based on distance
                if point.distance < 0.5:
                    color = self.colors['obstacle']
                else:
                    intensity = max(50, 255 - int(point.distance * 40))
                    color = (0, intensity, 255)
                
                pygame.draw.circle(self.screen, color, (screen_x, screen_y), 2)
    
    def draw_status_info(self):
        """Draw status information"""
        y_offset = 10
        line_height = 25
        
        # Title
        title = self.large_font.render("LIDAR-Enhanced Pathfinding Demo", True, self.colors['text'])
        self.screen.blit(title, (10, y_offset))
        y_offset += 40
        
        # Robot position
        robot = self.mock_controller.robot
        pos_text = f"Robot Position: X={robot.x:.2f}m, Y={robot.y:.2f}m, θ={math.degrees(robot.theta):.1f}°"
        pos_surface = self.font.render(pos_text, True, self.colors['text'])
        self.screen.blit(pos_surface, (10, y_offset))
        y_offset += line_height
        
        # Robot movement
        move_text = f"Movement: Forward={robot.forward_speed:.2f}m/s, Angular={math.degrees(robot.angular_speed):.1f}°/s"
        move_surface = self.font.render(move_text, True, self.colors['text'])
        self.screen.blit(move_surface, (10, y_offset))
        y_offset += line_height
        
        # LIDAR status
        if self.enhanced_controller.lidar and self.enhanced_controller.lidar_running:
            lidar_text = "LIDAR: CONNECTED"
            lidar_color = (0, 255, 0)
        else:
            lidar_text = "LIDAR: DISCONNECTED"
            lidar_color = (255, 0, 0)
        
        lidar_surface = self.font.render(lidar_text, True, lidar_color)
        self.screen.blit(lidar_surface, (10, y_offset))
        y_offset += line_height
        
        # Obstacle detection
        if self.enhanced_controller.obstacle_detected:
            obstacle_text = f"OBSTACLE DETECTED: {self.enhanced_controller.obstacle_direction or 'UNKNOWN'}"
            obstacle_color = (255, 0, 0)
        else:
            obstacle_text = "CLEARANCE: OK"
            obstacle_color = (0, 255, 0)
        
        obstacle_surface = self.font.render(obstacle_text, True, obstacle_color)
        self.screen.blit(obstacle_surface, (10, y_offset))
        y_offset += line_height
        
        # Avoidance status
        if self.enhanced_controller.avoidance_active:
            avoid_text = "AVOIDANCE: ACTIVE"
            avoid_color = (255, 255, 0)
        else:
            avoid_text = "AVOIDANCE: STANDBY"
            avoid_color = (100, 100, 100)
        
        avoid_surface = self.font.render(avoid_text, True, avoid_color)
        self.screen.blit(avoid_surface, (10, y_offset))
        y_offset += line_height
        
        # Robot dimensions info
        y_offset += 10
        dim_title = self.font.render("Robot Specifications:", True, self.colors['text'])
        self.screen.blit(dim_title, (10, y_offset))
        y_offset += line_height
        
        specs = [
            f"• Chassis Length: {self.enhanced_controller.robot_length*1000:.0f}mm",
            f"• LIDAR to Front: {self.enhanced_controller.lidar_to_front*1000:.0f}mm", 
            f"• Required Clearances: Front {self.enhanced_controller.clearance_front*100:.0f}cm, Sides {self.enhanced_controller.clearance_sides*100:.0f}cm"
        ]
        
        for spec in specs:
            spec_surface = self.small_font.render(spec, True, self.colors['text'])
            self.screen.blit(spec_surface, (20, y_offset))
            y_offset += 20
        
        # Controls
        y_offset += 10
        controls_title = self.font.render("Controls:", True, self.colors['text'])
        self.screen.blit(controls_title, (10, y_offset))
        y_offset += line_height
        
        controls = [
            "• Arrow Keys: Manual robot control",
            "• L: Toggle LIDAR overlay",
            "• M: Clear LIDAR map", 
            "• N: Save LIDAR map",
            "• ESC: Exit"
        ]
        
        for control in controls:
            control_surface = self.small_font.render(control, True, self.colors['text'])
            self.screen.blit(control_surface, (20, y_offset))
            y_offset += 20
    
    def handle_manual_control(self, keys):
        """Handle manual robot control"""
        forward_speed = 0.0
        angular_speed = 0.0
        
        if keys[pygame.K_UP]:
            forward_speed = 1.0
        elif keys[pygame.K_DOWN]:
            forward_speed = -0.5
        
        if keys[pygame.K_LEFT]:
            angular_speed = 1.0
        elif keys[pygame.K_RIGHT]:
            angular_speed = -1.0
        
        self.mock_controller.robot.apply_movement(forward_speed, angular_speed)
    
    def run(self):
        """Run the demo"""
        print("🚀 Starting LIDAR-Enhanced Pathfinding Demo")
        print("📡 Attempting to connect to LIDAR...")
        
        # Try to start LIDAR
        if self.enhanced_controller.start_lidar_mapping():
            print("✅ LIDAR connected successfully")
        else:
            print("⚠️  LIDAR connection failed - running in simulation mode")
        
        print("\n🎮 Controls:")
        print("   Arrow Keys: Manual robot control")
        print("   L: Toggle LIDAR overlay")
        print("   M: Clear LIDAR map")
        print("   N: Save LIDAR map")
        print("   ESC: Exit")
        print()
        
        while self.running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    else:
                        # Pass to enhanced controller
                        self.enhanced_controller.handle_enhanced_events(event)
            
            # Handle manual control
            keys = pygame.key.get_pressed()
            self.handle_manual_control(keys)
            
            # Clear screen
            self.screen.fill(self.colors['background'])
            
            # Draw components
            self.draw_grid()
            self.draw_lidar_data()
            self.draw_robot()
            
            # Draw LIDAR overlay
            self.enhanced_controller.draw_lidar_overlay(self.screen)
            
            # Draw status info
            self.draw_status_info()
            
            # Update display
            pygame.display.flip()
            self.clock.tick(30)  # 30 FPS
        
        # Cleanup
        print("🛑 Shutting down demo...")
        self.enhanced_controller.cleanup()
        pygame.quit()


def main():
    """Main entry point"""
    try:
        demo = LidarPathfindingDemo()
        demo.run()
    except KeyboardInterrupt:
        print("\n🛑 Demo interrupted by user")
    except Exception as e:
        print(f"❌ Demo error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

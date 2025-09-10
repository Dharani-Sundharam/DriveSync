#!/usr/bin/env python3
"""
Simple LIDAR Test with Obstacle Avoidance
==========================================

A simplified test script that focuses on LIDAR obstacle detection and avoidance
without complex GUI elements that might cause the window to close.
"""

import pygame
import sys
import os
import time
import math
import threading
import logging

# Handle both relative and absolute imports
try:
    from .pathfinding_integration import LidarEnhancedPathfindingController
    from .ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
except ImportError:
    from pathfinding_integration import LidarEnhancedPathfindingController
    from ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig


class MockPathfindingController:
    """Simplified mock pathfinding controller"""
    
    def __init__(self, width=800, height=600):
        self.width = width
        self.height = height
        
        # Mock robot state
        self.robot = MockRobot()
        
        # Mock fonts
        pygame.font.init()
        self.small_font = pygame.font.Font(None, 18)
        self.font = pygame.font.Font(None, 24)
        
        # Control state
        self.running = True
        self.manual_control = False
    
    def apply_obstacle_avoidance(self, avoidance_command):
        """Apply obstacle avoidance command to robot"""
        left_speed, right_speed = avoidance_command
        
        # Convert to robot movement
        forward_speed = (left_speed + right_speed) / 2
        angular_speed = (right_speed - left_speed) / 2
        
        # Apply to mock robot
        self.robot.apply_movement(forward_speed, angular_speed)
        
        print(f"🚨 Avoiding obstacle: Forward={forward_speed:.2f}, Turn={angular_speed:.2f}")


class MockRobot:
    """Simplified mock robot"""
    
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


class SimpleLidarTest:
    """Simple LIDAR test application"""
    
    def __init__(self):
        # Initialize pygame
        pygame.init()
        
        self.width = 800
        self.height = 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("LIDAR Obstacle Avoidance Test")
        
        # Colors
        self.colors = {
            'background': (30, 30, 40),
            'robot_safe': (0, 255, 0),
            'robot_danger': (255, 100, 100),
            'lidar': (0, 200, 255),
            'text': (255, 255, 255),
            'warning': (255, 255, 0)
        }
        
        # Create mock controller
        self.mock_controller = MockPathfindingController(self.width, self.height)
        
        # Create LIDAR-enhanced controller
        print("🚀 Initializing LIDAR system...")
        self.enhanced_controller = LidarEnhancedPathfindingController(
            pathfinding_controller=self.mock_controller,
            lidar_port='/dev/ttyUSB1',
            enable_lidar_mapping=True
        )
        
        # Setup fonts
        self.font = pygame.font.Font(None, 24)
        self.large_font = pygame.font.Font(None, 32)
        
        # Demo state
        self.clock = pygame.time.Clock()
        self.running = True
        
        # Robot update thread
        self.robot_thread = threading.Thread(target=self.robot_update_loop, daemon=True)
        self.robot_thread.start()
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger('SimpleLidarTest')
    
    def robot_update_loop(self):
        """Update robot position continuously"""
        while self.running:
            try:
                self.mock_controller.robot.update_position()
                time.sleep(0.05)  # 20 Hz update
            except Exception as e:
                self.logger.error(f"Robot update error: {e}")
                time.sleep(0.1)
    
    def world_to_screen(self, world_x, world_y):
        """Convert world coordinates to screen coordinates"""
        screen_x = int(self.width / 2 + world_x * 100)  # 100 pixels per meter
        screen_y = int(self.height / 2 - world_y * 100)  # Flip Y axis
        return screen_x, screen_y
    
    def draw_robot(self):
        """Draw robot fixed at center of screen"""
        robot = self.mock_controller.robot
        
        # Robot is always at the center of the screen
        screen_x = self.width // 2
        screen_y = self.height // 2
        
        # Choose color based on obstacle detection
        if self.enhanced_controller.obstacle_detected:
            robot_color = self.colors['robot_danger']
        else:
            robot_color = self.colors['robot_safe']
        
        # Draw robot body as circle (simplified)
        pygame.draw.circle(self.screen, robot_color, (screen_x, screen_y), 20)
        
        # Draw direction arrow
        arrow_length = 30
        cos_theta = math.cos(robot.theta)
        sin_theta = math.sin(robot.theta)
        arrow_end_x = screen_x + arrow_length * cos_theta
        arrow_end_y = screen_y - arrow_length * sin_theta  # Flip Y
        pygame.draw.line(self.screen, (255, 255, 255), 
                        (screen_x, screen_y), (arrow_end_x, arrow_end_y), 3)
        
        # Draw clearance circles if obstacle detected
        if self.enhanced_controller.obstacle_detected:
            # Front clearance
            front_radius = int((self.enhanced_controller.lidar_to_front + 
                              self.enhanced_controller.clearance_front) * 100)
            pygame.draw.circle(self.screen, self.colors['warning'], 
                             (screen_x, screen_y), front_radius, 1)
    
    def draw_lidar_data(self):
        """Draw LIDAR scan data relative to robot center"""
        if not self.enhanced_controller.lidar or not self.enhanced_controller.lidar_running:
            return
        
        scan = self.enhanced_controller.lidar.get_latest_scan()
        if not scan or not scan.points:
            return
        
        # Robot is always at center of screen
        robot_screen_x = self.width // 2
        robot_screen_y = self.height // 2
        pixels_per_meter = 100
        
        # Draw LIDAR points relative to robot
        for point in scan.points:
            if 0.05 <= point.distance <= 6.0:  # Valid range
                # Convert polar to cartesian (relative to robot)
                rel_x = point.distance * math.cos(point.angle)
                rel_y = point.distance * math.sin(point.angle)
                
                # Convert to screen coordinates
                screen_x = int(robot_screen_x + rel_x * pixels_per_meter)
                screen_y = int(robot_screen_y - rel_y * pixels_per_meter)  # Flip Y axis
                
                # Only draw if within screen bounds
                if (0 <= screen_x < self.width and 0 <= screen_y < self.height):
                    # Color based on distance
                    if point.distance < 0.5:
                        color = (255, 0, 0)  # Red for close obstacles
                    else:
                        intensity = max(50, 255 - int(point.distance * 40))
                        color = (0, intensity, 255)
                    
                    pygame.draw.circle(self.screen, color, (screen_x, screen_y), 3)
    
    def draw_status_info(self):
        """Draw status information"""
        y_offset = 10
        line_height = 25
        
        # Title
        title = self.large_font.render("LIDAR Obstacle Avoidance Test", True, self.colors['text'])
        self.screen.blit(title, (10, y_offset))
        y_offset += 40
        
        # Robot position
        robot = self.mock_controller.robot
        pos_text = f"Robot: X={robot.x:.2f}m, Y={robot.y:.2f}m, θ={math.degrees(robot.theta):.1f}°"
        pos_surface = self.font.render(pos_text, True, self.colors['text'])
        self.screen.blit(pos_surface, (10, y_offset))
        y_offset += line_height
        
        # LIDAR status
        if self.enhanced_controller.lidar and self.enhanced_controller.lidar_running:
            lidar_text = "LIDAR: CONNECTED & SCANNING"
            lidar_color = (0, 255, 0)
        else:
            lidar_text = "LIDAR: DISCONNECTED"
            lidar_color = (255, 0, 0)
        
        lidar_surface = self.font.render(lidar_text, True, lidar_color)
        self.screen.blit(lidar_surface, (10, y_offset))
        y_offset += line_height
        
        # Obstacle detection
        if self.enhanced_controller.obstacle_detected:
            obstacle_text = f"⚠️ OBSTACLE: {self.enhanced_controller.obstacle_direction or 'DETECTED'}"
            obstacle_color = (255, 0, 0)
        else:
            obstacle_text = "✅ PATH CLEAR"
            obstacle_color = (0, 255, 0)
        
        obstacle_surface = self.font.render(obstacle_text, True, obstacle_color)
        self.screen.blit(obstacle_surface, (10, y_offset))
        y_offset += line_height
        
        # Clearance info
        clearance = self.enhanced_controller.check_robot_clearance()
        if clearance.get('obstacle_distances'):
            distances = clearance['obstacle_distances']
            clearance_text = f"Distances - F:{distances['front']:.2f}m L:{distances['left']:.2f}m R:{distances['right']:.2f}m B:{distances['back']:.2f}m"
            clearance_surface = self.font.render(clearance_text, True, self.colors['text'])
            self.screen.blit(clearance_surface, (10, y_offset))
            y_offset += line_height
        
        # Controls
        y_offset += 10
        controls_title = self.font.render("Controls:", True, self.colors['text'])
        self.screen.blit(controls_title, (10, y_offset))
        y_offset += line_height
        
        controls = [
            "Arrow Keys: Manual robot control",
            "SPACE: Stop robot", 
            "ESC: Exit"
        ]
        
        for control in controls:
            control_surface = pygame.font.Font(None, 18).render(control, True, self.colors['text'])
            self.screen.blit(control_surface, (20, y_offset))
            y_offset += 18
    
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
        
        if keys[pygame.K_SPACE]:
            forward_speed = 0.0
            angular_speed = 0.0
        
        self.mock_controller.robot.apply_movement(forward_speed, angular_speed)
    
    def run(self):
        """Run the test application"""
        print("🚀 Starting Simple LIDAR Test")
        print("📡 Attempting to connect to LIDAR...")
        
        # Try to start LIDAR
        if self.enhanced_controller.start_lidar_mapping():
            print("✅ LIDAR connected successfully")
        else:
            print("⚠️ LIDAR connection failed - running in simulation mode")
        
        print("\n🎮 Controls:")
        print("   Arrow Keys: Manual robot control")
        print("   SPACE: Stop robot")
        print("   ESC: Exit")
        print("\n🔄 Starting main loop...")
        
        frame_count = 0
        
        while self.running:
            frame_count += 1
            
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("🛑 Window close requested")
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("🛑 ESC pressed - exiting")
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
            self.draw_lidar_data()
            self.draw_robot()
            self.draw_status_info()
            
            # Update display
            pygame.display.flip()
            
            # Control frame rate
            self.clock.tick(30)  # 30 FPS
            
            # Print status every 5 seconds
            if frame_count % 150 == 0:  # Every 5 seconds at 30 FPS
                status = "🟢 OBSTACLE" if self.enhanced_controller.obstacle_detected else "🟢 CLEAR"
                print(f"Status: {status} | Frame: {frame_count}")
        
        # Cleanup
        print("🧹 Cleaning up...")
        self.enhanced_controller.cleanup()
        pygame.quit()
        print("✅ Test completed")


def main():
    """Main entry point"""
    try:
        print("=" * 60)
        print("🤖 SIMPLE LIDAR OBSTACLE AVOIDANCE TEST")
        print("=" * 60)
        
        test = SimpleLidarTest()
        test.run()
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("👋 Goodbye!")


if __name__ == "__main__":
    main()

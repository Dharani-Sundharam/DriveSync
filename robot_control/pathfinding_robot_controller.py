#!/usr/bin/env python3
"""
Pathfinding Robot Controller
Main controller integrating map environment, pathfinding, and navigation
"""

import pygame
import sys
import time
import math
import logging
from typing import Optional, Tuple

# Import our modules
from modules.map_environment import MapEnvironment
from modules.pathfinding import AStarPathfinder, RRTPathfinder
from modules.navigation import NavigationController, NavigationState

# Import the robot controller from smooth_robot_controller
try:
    from smooth_robot_controller import OptimizedRobotController
except ImportError:
    # Fallback dynamic import
    import importlib.util
    spec = importlib.util.spec_from_file_location("robot", "smooth_robot_controller.py")
    robot_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot_module)
    OptimizedRobotController = robot_module.OptimizedRobotController

class PathfindingRobotController:
    """Main controller for pathfinding robot system"""
    
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        # Setup logging
        self.setup_logging()
        self.logger = logging.getLogger('PathfindingRobot')
        
        # Initialize robot controller
        self.robot = OptimizedRobotController(port, baudrate)
        
        # Start robot controller threads for Arduino communication
        if self.robot.serial_port:
            self.robot.start_threads()
            time.sleep(1)  # Let threads initialize
            self.logger.info("✅ Robot controller threads started")
        else:
            self.logger.error("❌ Failed to connect to Arduino")
        
        # Set robot to start at center of map (on road intersection)
        self.robot.x = 0.0  # Center X
        self.robot.y = 0.0  # Center Y  
        self.robot.theta = 0.0  # Facing right
        self.robot.path = [(0.0, 0.0)]  # Reset path
        
        # Initialize Pygame with fullscreen
        pygame.init()
        
        # Get display info for fullscreen
        display_info = pygame.display.Info()
        self.width = display_info.current_w
        self.height = display_info.current_h
        
        # Create fullscreen display
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN)
        pygame.display.set_caption("🤖 Pathfinding Robot Controller - Press ESC to exit")
        
        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.BLUE = (0, 100, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)
        self.YELLOW = (255, 255, 0)
        self.CYAN = (0, 255, 255)
        self.MAGENTA = (255, 0, 255)
        self.LIGHT_GRAY = (200, 200, 200)
        
        # Initialize fonts
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        self.large_font = pygame.font.Font(None, 32)
        
        # Initialize map environment with larger scale for better visibility
        self.map_env = MapEnvironment(self.width, self.height, scale=150.0)
        
        # Initialize pathfinding algorithms
        self.astar = AStarPathfinder(self.map_env)
        self.rrt = RRTPathfinder(self.map_env)
        self.current_pathfinder = self.astar  # Default to A*
        
        # Initialize navigation controller
        self.navigator = NavigationController(self.robot, self.map_env)
        
        # UI state
        self.selected_target: Optional[Tuple[float, float]] = None
        self.mouse_world_pos: Optional[Tuple[float, float]] = None
        self.show_help = False
        self.pathfinder_name = "A*"
        self.robot_placement_mode = False  # Mode for placing robot
        
        # Performance tracking
        self.last_frame_time = time.time()
        self.fps = 60
        
        self.logger.info("🚀 Pathfinding Robot Controller initialized")
    
    def setup_logging(self):
        """Setup logging configuration"""
        import os
        if not os.path.exists('logs'):
            os.makedirs('logs')
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('logs/pathfinding_robot.log'),
                logging.StreamHandler()
            ]
        )
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_h:
                    self.show_help = not self.show_help
                elif event.key == pygame.K_SPACE:
                    self.navigator.stop_navigation()
                elif event.key == pygame.K_1:
                    self.current_pathfinder = self.astar
                    self.pathfinder_name = "A*"
                    self.logger.info("🔄 Switched to A* pathfinder")
                elif event.key == pygame.K_2:
                    self.current_pathfinder = self.rrt
                    self.pathfinder_name = "RRT"
                    self.logger.info("🔄 Switched to RRT pathfinder")
                elif event.key == pygame.K_r:
                    # Reset robot position to center
                    self.robot.x = 0.0
                    self.robot.y = 0.0
                    self.robot.theta = 0.0
                    self.robot.path = [(0.0, 0.0)]
                    self.logger.info("🔄 Reset robot position")
                elif event.key == pygame.K_f:
                    # Toggle fullscreen (for testing)
                    pygame.display.toggle_fullscreen()
                elif event.key == pygame.K_p:
                    # Toggle robot placement mode
                    self.robot_placement_mode = not self.robot_placement_mode
                    if self.robot_placement_mode:
                        self.logger.info("🎯 Robot placement mode ON - Click to place robot")
                    else:
                        self.logger.info("🎯 Robot placement mode OFF")
                elif event.key == pygame.K_x:
                    # Debug info
                    print(f"🐛 DEBUG INFO:")
                    print(f"   Robot position: ({self.robot.x:.3f}, {self.robot.y:.3f})")
                    print(f"   Robot on road: {self.map_env.is_point_on_road(self.robot.x, self.robot.y)}")
                    print(f"   Navigation state: {self.navigator.state}")
                    print(f"   Current path length: {len(self.navigator.current_path)}")
                    if self.mouse_world_pos:
                        print(f"   Mouse position: {self.mouse_world_pos}")
                        print(f"   Mouse on road: {self.map_env.is_point_on_road(*self.mouse_world_pos)}")
                    print(f"   Manual control active: {not self.navigator.is_navigating()}")
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    self.handle_mouse_click(event.pos)
            
            elif event.type == pygame.MOUSEMOTION:
                # Update mouse world position for display
                self.mouse_world_pos = self.map_env.screen_to_world(*event.pos)
        
        # Handle keyboard input for manual control (when not navigating)
        if not self.navigator.is_navigating():
            self.handle_manual_control()
        else:
            # If navigating, check for manual override
            keys = pygame.key.get_pressed()
            if any([keys[pygame.K_w], keys[pygame.K_a], keys[pygame.K_s], keys[pygame.K_d]]):
                # Manual override - stop navigation
                self.navigator.stop_navigation()
                self.handle_manual_control()
        
        return True
    
    def handle_mouse_click(self, screen_pos):
        """Handle mouse click for target selection or robot placement"""
        world_x, world_y = self.map_env.screen_to_world(*screen_pos)
        
        if self.robot_placement_mode:
            # Robot placement mode
            if self.map_env.is_point_on_road(world_x, world_y):
                # Place robot on road
                self.robot.x = world_x
                self.robot.y = world_y
                self.robot.theta = 0.0  # Face right
                self.robot.path = [(world_x, world_y)]
                self.logger.info(f"🤖 Robot placed at: ({world_x:.3f}, {world_y:.3f})")
            else:
                # Snap to nearest road
                nearest = self.astar.find_nearest_road_point(world_x, world_y)
                self.robot.x = nearest[0]
                self.robot.y = nearest[1] 
                self.robot.theta = 0.0
                self.robot.path = [(nearest[0], nearest[1])]
                self.logger.info(f"🤖 Robot snapped to road: ({nearest[0]:.3f}, {nearest[1]:.3f})")
        else:
            # Normal navigation mode
            if self.map_env.is_point_on_road(world_x, world_y):
                self.selected_target = (world_x, world_y)
                self.logger.info(f"🎯 Target set: ({world_x:.3f}, {world_y:.3f})")
                
                # Start navigation
                self.navigator.set_target(world_x, world_y, self.current_pathfinder)
            else:
                # Find nearest road point
                nearest = self.astar.find_nearest_road_point(world_x, world_y)
                self.selected_target = nearest
                self.logger.info(f"🎯 Target snapped to road: ({nearest[0]:.3f}, {nearest[1]:.3f})")
                
                # Start navigation
                self.navigator.set_target(nearest[0], nearest[1], self.current_pathfinder)
    
    def handle_manual_control(self):
        """Handle manual keyboard control when not navigating"""
        keys = pygame.key.get_pressed()
        
        left_speed = 0
        right_speed = 0
        
        if keys[pygame.K_w]:  # Forward
            left_speed = right_speed = 150
        elif keys[pygame.K_s]:  # Backward
            left_speed = right_speed = -150
        elif keys[pygame.K_a]:  # Turn left
            left_speed, right_speed = -120, 120
        elif keys[pygame.K_d]:  # Turn right
            left_speed, right_speed = 120, -120
        
        # Always send command (even if 0,0 to stop)
        try:
            self.robot.send_motor_command(left_speed, right_speed)
        except Exception as e:
            print(f"❌ Motor command failed: {e}")
    
    def update(self):
        """Update all systems"""
        # Robot controller runs in its own threads, no need to update manually
        
        # Update navigation
        self.navigator.update()
        
        # Update performance tracking
        current_time = time.time()
        if current_time - self.last_frame_time > 0:
            self.fps = 1.0 / (current_time - self.last_frame_time)
        self.last_frame_time = current_time
    
    def draw(self):
        """Draw everything on screen"""
        # Clear screen
        self.screen.fill(self.BLACK)
        
        # Draw map environment
        self.map_env.draw(self.screen)
        
        # Draw robot path
        self.draw_robot_path()
        
        # Draw current planned path
        if self.navigator.current_path:
            self.map_env.draw_path(self.screen, self.navigator.current_path)
        
        # Draw robot
        self.draw_robot()
        
        # Draw target
        if self.selected_target:
            self.map_env.draw_target(self.screen, *self.selected_target)
        
        # Draw current waypoint
        current_waypoint = self.navigator.get_current_waypoint()
        if current_waypoint:
            waypoint_screen = self.map_env.world_to_screen(*current_waypoint)
            pygame.draw.circle(self.screen, self.CYAN, waypoint_screen, 10)
            pygame.draw.circle(self.screen, self.WHITE, waypoint_screen, 6)
        
        # Draw UI
        self.draw_ui()
        
        # Draw help if enabled
        if self.show_help:
            self.draw_help()
        
        # Update display
        pygame.display.flip()
    
    def draw_robot_path(self):
        """Draw the robot's traveled path"""
        if len(self.robot.path) > 1:
            screen_points = []
            for x, y in self.robot.path:
                screen_x, screen_y = self.map_env.world_to_screen(x, y)
                if 0 <= screen_x <= self.width and 0 <= screen_y <= self.height:
                    screen_points.append((screen_x, screen_y))
            
            if len(screen_points) > 1:
                pygame.draw.lines(self.screen, self.GREEN, False, screen_points, 2)
    
    def draw_robot(self):
        """Draw the robot with enhanced visualization"""
        robot_x, robot_y = self.map_env.world_to_screen(self.robot.x, self.robot.y)
        
        # Robot body
        robot_size = 28
        cos_theta = math.cos(self.robot.theta)
        sin_theta = math.sin(self.robot.theta)
        
        # Robot corners
        corners = []
        for dx, dy in [(-robot_size//2, -robot_size//3), (robot_size//2, -robot_size//3),
                      (robot_size//2, robot_size//3), (-robot_size//2, robot_size//3)]:
            rotated_x = dx * cos_theta - dy * sin_theta
            rotated_y = dx * sin_theta + dy * cos_theta
            corners.append((robot_x + rotated_x, robot_y - rotated_y))
        
        pygame.draw.polygon(self.screen, self.BLUE, corners)
        
        # Direction arrow
        arrow_length = robot_size * 1.3
        arrow_end_x = robot_x + arrow_length * cos_theta
        arrow_end_y = robot_y - arrow_length * sin_theta
        pygame.draw.line(self.screen, self.YELLOW, (robot_x, robot_y), 
                        (arrow_end_x, arrow_end_y), 4)
        
        # Navigation status indicator
        nav_info = self.navigator.get_navigation_info()
        if nav_info['state'] == 'navigating':
            # Green ring when navigating
            pygame.draw.circle(self.screen, self.GREEN, (robot_x, robot_y), robot_size + 5, 3)
        elif nav_info['state'] == 'reached_goal':
            # Cyan ring when reached goal
            pygame.draw.circle(self.screen, self.CYAN, (robot_x, robot_y), robot_size + 5, 3)
        
        # Center dot
        pygame.draw.circle(self.screen, self.WHITE, (robot_x, robot_y), 3)
    
    def draw_ui(self):
        """Draw user interface elements"""
        # Background for UI
        ui_rect = pygame.Rect(10, 10, 350, 250)
        pygame.draw.rect(self.screen, (0, 0, 0, 128), ui_rect)
        pygame.draw.rect(self.screen, self.WHITE, ui_rect, 2)
        
        y_offset = 20
        
        # Title
        title = self.large_font.render("🤖 Pathfinding Robot", True, self.CYAN)
        self.screen.blit(title, (20, y_offset))
        y_offset += 35
        
        # Robot status
        status_lines = [
            f"Position: ({self.robot.x:.3f}, {self.robot.y:.3f}) m",
            f"Heading: {math.degrees(self.robot.theta):.1f}°",
            f"Motors: L={self.robot.left_speed}, R={self.robot.right_speed}",
            f"Encoders: L={self.robot.encoder_a}, R={self.robot.encoder_b}",
        ]
        
        for line in status_lines:
            text = self.small_font.render(line, True, self.WHITE)
            self.screen.blit(text, (20, y_offset))
            y_offset += 20
        
        # Robot placement mode indicator
        if self.robot_placement_mode:
            placement_text = self.font.render("🎯 ROBOT PLACEMENT MODE", True, self.MAGENTA)
            self.screen.blit(placement_text, (20, y_offset))
            y_offset += 25
            
            instruction_text = self.small_font.render("Click anywhere to place robot", True, self.CYAN)
            self.screen.blit(instruction_text, (20, y_offset))
            y_offset += 20
        
        # Navigation status
        nav_info = self.navigator.get_navigation_info()
        y_offset += 10
        
        nav_title = self.font.render("Navigation Status:", True, self.YELLOW)
        self.screen.blit(nav_title, (20, y_offset))
        y_offset += 25
        
        nav_lines = [
            f"State: {nav_info['state'].upper()}",
            f"Algorithm: {self.pathfinder_name}",
            f"Waypoint: {nav_info['waypoint_index']}/{nav_info['total_waypoints']}",
            f"Progress: {nav_info['progress_percent']:.1f}%"
        ]
        
        for line in nav_lines:
            color = self.GREEN if nav_info['state'] == 'navigating' else self.WHITE
            text = self.small_font.render(line, True, color)
            self.screen.blit(text, (20, y_offset))
            y_offset += 18
        
        # Mouse position
        if self.mouse_world_pos:
            mouse_text = f"Mouse: ({self.mouse_world_pos[0]:.3f}, {self.mouse_world_pos[1]:.3f})"
            on_road = "ON ROAD" if self.map_env.is_point_on_road(*self.mouse_world_pos) else "OFF ROAD"
            mouse_color = self.GREEN if "ON" in on_road else self.RED
            
            mouse_surface = self.small_font.render(mouse_text, True, self.LIGHT_GRAY)
            road_surface = self.small_font.render(on_road, True, mouse_color)
            
            self.screen.blit(mouse_surface, (20, self.height - 40))
            self.screen.blit(road_surface, (20, self.height - 20))
        
        # Performance info
        fps_text = self.small_font.render(f"FPS: {self.fps:.1f}", True, self.WHITE)
        self.screen.blit(fps_text, (self.width - 100, 20))
        
        # Instructions
        help_text = self.small_font.render("Press 'H' for help", True, self.LIGHT_GRAY)
        self.screen.blit(help_text, (self.width - 150, self.height - 20))
    
    def draw_help(self):
        """Draw help overlay"""
        # Semi-transparent background
        help_surface = pygame.Surface((self.width, self.height))
        help_surface.set_alpha(200)
        help_surface.fill((0, 0, 0))
        self.screen.blit(help_surface, (0, 0))
        
        # Help content
        help_lines = [
            "🤖 PATHFINDING ROBOT CONTROLLER HELP",
            "",
            "MOUSE CONTROLS:",
            "  • Left Click: Set navigation target",
            "  • Click on roads or system will snap to nearest road",
            "",
            "KEYBOARD CONTROLS:",
            "  • W/A/S/D: Manual control (when not navigating)",
            "  • SPACE: Stop current navigation",
            "  • P: Toggle robot placement mode",
            "  • 1: Switch to A* pathfinding algorithm",
            "  • 2: Switch to RRT pathfinding algorithm", 
            "  • R: Reset robot position to center (0,0)",
            "  • F: Toggle fullscreen mode",
            "  • X: Show debug information",
            "  • H: Toggle this help screen",
            "  • ESC: Exit application",
            "",
            "NAVIGATION:",
            "  • Green ring around robot: Currently navigating",
            "  • Cyan ring: Reached target destination",
            "  • Yellow line: Planned path to target",
            "  • Green line: Robot's traveled path",
            "  • Cyan circles: Current waypoint",
            "",
            "MAP ELEMENTS:",
            "  • Dark gray: Roads (navigable areas)",
            "  • Brown lines: Road boundaries",
            "  • Red circles: Obstacles",
            "  • Green areas: Grass (non-navigable)",
            "",
            "Press 'H' again to close help"
        ]
        
        y_start = 50
        for i, line in enumerate(help_lines):
            if line.startswith("🤖"):
                color = self.CYAN
                font = self.large_font
            elif line.endswith(":"):
                color = self.YELLOW
                font = self.font
            elif line.startswith("  •"):
                color = self.WHITE
                font = self.small_font
            else:
                color = self.LIGHT_GRAY
                font = self.small_font
            
            text = font.render(line, True, color)
            self.screen.blit(text, (50, y_start + i * 22))
    
    def run(self):
        """Main application loop"""
        clock = pygame.time.Clock()
        
        self.logger.info("🚀 Starting pathfinding robot controller...")
        
        try:
            while True:
                # Handle events
                if not self.handle_events():
                    break
                
                # Update systems
                self.update()
                
                # Draw everything
                self.draw()
                
                # Control frame rate
                clock.tick(60)
        
        except KeyboardInterrupt:
            self.logger.info("👋 Received keyboard interrupt")
        
        except Exception as e:
            self.logger.error(f"❌ Application error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.logger.info("🧹 Cleaning up...")
        
        # Stop navigation
        self.navigator.stop_navigation()
        
        # Stop robot
        self.robot.cleanup()
        
        # Quit pygame
        pygame.quit()
        
        self.logger.info("👋 Pathfinding robot controller stopped")

if __name__ == "__main__":
    controller = PathfindingRobotController(port='/dev/ttyUSB1')  # Use correct port
    controller.run()

#!/usr/bin/env python3
"""
Pathfinding Integration Module
==============================

This module integrates the LIDAR mapping system with the existing pathfinding robot controller.
Provides enhanced navigation capabilities using real-time LIDAR mapping.
"""

import pygame
import threading
import time
import math
import logging
from typing import Optional, Tuple, List, Dict, Any

# Handle both relative and absolute imports
try:
    from .ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
    from .lidar_mapper import LidarMapper
    from .lidar_gui import CartesianPlot, DisplayConfig
except ImportError:
    from ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
    from lidar_mapper import LidarMapper
    from lidar_gui import CartesianPlot, DisplayConfig


class LidarEnhancedPathfindingController:
    """
    Enhanced pathfinding controller with LIDAR mapping integration
    
    Extends the existing pathfinding robot controller with real-time LIDAR
    mapping capabilities for improved navigation and obstacle avoidance.
    
    Robot Specifications:
    - Total chassis length: ~210mm
    - LIDAR positioned on top of motors (center of robot)
    - Distance from LIDAR to front: 160mm (16cm)
    - Required clearances: 5cm sides/back, 21cm front
    """
    
    def __init__(self, 
                 pathfinding_controller,
                 lidar_port: str = '/dev/ttyUSB1',  # LIDAR port (main controller on USB0)
                 enable_lidar_mapping: bool = True):
        """
        Initialize LIDAR-enhanced pathfinding controller
        
        Args:
            pathfinding_controller: Existing PathfindingRobotController instance
            lidar_port: Serial port for LIDAR connection
            enable_lidar_mapping: Enable LIDAR mapping functionality
        """
        self.base_controller = pathfinding_controller
        self.enable_lidar_mapping = enable_lidar_mapping
        
        # Robot physical dimensions (in meters)
        self.robot_length = 0.21  # 210mm total chassis length
        self.lidar_to_front = 0.16  # 160mm from LIDAR to front
        self.lidar_to_back = self.robot_length - self.lidar_to_front  # 50mm to back
        self.robot_width = 0.15  # Estimated robot width (150mm)
        
        # Safety clearances (in meters)
        self.clearance_front = 0.21  # 21cm front clearance
        self.clearance_sides = 0.05  # 5cm side clearance
        self.clearance_back = 0.05   # 5cm back clearance
        
        # LIDAR components
        self.lidar: Optional[YDLidarX2Optimized] = None
        self.mapper: Optional[LidarMapper] = None
        self.lidar_plot: Optional[CartesianPlot] = None
        
        # Obstacle detection state
        self.obstacle_detected = False
        self.obstacle_direction = None  # 'front', 'left', 'right', 'back'
        self.avoidance_active = False
        self.last_obstacle_check = time.time()
        
        # Display configuration for LIDAR overlay
        self.lidar_display_config = DisplayConfig()
        
        # LIDAR display area (overlay on main display)
        self.lidar_display_rect = None
        self.show_lidar_overlay = True
        self.lidar_overlay_size = (240, 160)  # Half of 480x320
        
        # Integration state
        self.lidar_thread: Optional[threading.Thread] = None
        self.lidar_running = False
        
        # Performance tracking
        self.lidar_update_rate = 0.0
        self.last_lidar_update = time.time()
        
        # Setup logging
        self.logger = logging.getLogger('LidarEnhancedController')
        self.logger.setLevel(logging.INFO)
        
        if enable_lidar_mapping:
            self.initialize_lidar_system(lidar_port)
    
    def initialize_lidar_system(self, lidar_port: str):
        """Initialize LIDAR mapping system"""
        try:
            self.logger.info("🚀 Initializing LIDAR mapping system...")
            
            # Initialize LIDAR interface with optimized configuration
            config = LidarConfig(port=lidar_port, baudrate=115200)
            self.lidar = YDLidarX2Optimized(config)
            
            # Initialize mapper
            self.mapper = LidarMapper(
                map_width=8.0,
                map_height=8.0,
                resolution=0.05,
                max_range=4.0
            )
            
            # Initialize LIDAR plot for overlay
            self.lidar_plot = CartesianPlot(
                display_width=self.lidar_overlay_size[0],
                display_height=self.lidar_overlay_size[1],
                world_width=8.0,
                world_height=8.0,
                plot_margin=10
            )
            
            # Calculate overlay position (top-right corner)
            main_width = self.base_controller.width
            main_height = self.base_controller.height
            
            self.lidar_display_rect = pygame.Rect(
                main_width - self.lidar_overlay_size[0] - 10,
                10,
                self.lidar_overlay_size[0],
                self.lidar_overlay_size[1]
            )
            
            self.logger.info("✅ LIDAR mapping system initialized")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to initialize LIDAR system: {e}")
            self.enable_lidar_mapping = False
    
    def start_lidar_mapping(self) -> bool:
        """Start LIDAR mapping"""
        if not self.enable_lidar_mapping or not self.lidar:
            return False
        
        try:
            # Connect to LIDAR
            if self.lidar.connect():
                if self.lidar.start_scanning():
                    # Start LIDAR update thread
                    self.lidar_running = True
                    self.lidar_thread = threading.Thread(target=self._lidar_update_loop, daemon=True)
                    self.lidar_thread.start()
                    
                    self.logger.info("✅ LIDAR mapping started")
                    return True
                else:
                    self.logger.error("❌ Failed to start LIDAR scanning")
            else:
                self.logger.error("❌ Failed to connect to LIDAR")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to start LIDAR mapping: {e}")
        
        return False
    
    def stop_lidar_mapping(self):
        """Stop LIDAR mapping"""
        if not self.enable_lidar_mapping:
            return
        
        self.lidar_running = False
        
        if self.lidar_thread and self.lidar_thread.is_alive():
            self.lidar_thread.join(timeout=2.0)
        
        if self.lidar:
            self.lidar.cleanup()
        
        self.logger.info("🛑 LIDAR mapping stopped")
    
    def _lidar_update_loop(self):
        """LIDAR update loop (runs in separate thread)"""
        self.logger.info("🔄 LIDAR update loop started")
        
        while self.lidar_running and self.lidar:
            try:
                # Update robot pose in mapper
                if self.mapper:
                    self.mapper.update_robot_pose(
                        self.base_controller.robot.x,
                        self.base_controller.robot.y,
                        self.base_controller.robot.theta
                    )
                
                # Get latest scan and update map
                scan = self.lidar.get_latest_scan()
                if scan and self.mapper:
                    self.mapper.update_map(scan)
                    
                    # Check for obstacles and update avoidance
                    clearance = self.check_robot_clearance()
                    
                    # Apply obstacle avoidance if needed
                    if not clearance['safe'] and hasattr(self.base_controller, 'apply_obstacle_avoidance'):
                        avoidance_command = self.get_obstacle_avoidance_command()
                        self.base_controller.apply_obstacle_avoidance(avoidance_command)
                    
                    # Update performance tracking
                    current_time = time.time()
                    time_diff = current_time - self.last_lidar_update
                    if time_diff > 0:
                        self.lidar_update_rate = 1.0 / time_diff
                    self.last_lidar_update = current_time
                
                # Sleep briefly to control update rate
                time.sleep(0.05)  # 20 Hz update rate
                
            except Exception as e:
                self.logger.error(f"❌ LIDAR update loop error: {e}")
                time.sleep(0.1)
        
        self.logger.info("🔄 LIDAR update loop ended")
    
    def handle_enhanced_events(self, event) -> bool:
        """
        Handle additional events for LIDAR functionality
        
        Args:
            event: Pygame event
            
        Returns:
            True if event was handled, False to pass to base controller
        """
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_l:
                # Toggle LIDAR overlay
                self.show_lidar_overlay = not self.show_lidar_overlay
                self.logger.info(f"🔄 LIDAR overlay: {'ON' if self.show_lidar_overlay else 'OFF'}")
                return True
            elif event.key == pygame.K_m:
                # Clear LIDAR map
                if self.mapper:
                    self.mapper.clear_map()
                    self.logger.info("🗺️  LIDAR map cleared")
                return True
            elif event.key == pygame.K_n:
                # Save LIDAR map
                if self.mapper:
                    timestamp = int(time.time())
                    filename = f"logs/lidar_map_{timestamp}.npy"
                    self.mapper.save_map(filename)
                    self.logger.info(f"💾 LIDAR map saved: {filename}")
                return True
        
        return False  # Event not handled, pass to base controller
    
    def check_robot_clearance(self) -> Dict[str, Any]:
        """
        Check robot clearance in all directions based on LIDAR data
        
        Returns:
            Dict with clearance information for each direction
        """
        if not self.lidar or not self.lidar_running:
            return {'front': True, 'left': True, 'right': True, 'back': True, 'safe': True}
        
        scan = self.lidar.get_latest_scan()
        if not scan or not scan.points:
            return {'front': True, 'left': True, 'right': True, 'back': True, 'safe': True}
        
        clearance_status = {
            'front': True,
            'left': True, 
            'right': True,
            'back': True,
            'safe': True,
            'closest_obstacle': None,
            'obstacle_distances': {'front': float('inf'), 'left': float('inf'), 'right': float('inf'), 'back': float('inf')}
        }
        
        # Define angular sectors for each direction (in radians)
        sectors = {
            'front': (-math.pi/6, math.pi/6),      # ±30° front
            'right': (-math.pi/2, -math.pi/6),     # -90° to -30° right
            'back': (5*math.pi/6, -5*math.pi/6),   # ±150° to ±180° back  
            'left': (math.pi/6, math.pi/2)         # 30° to 90° left
        }
        
        # Required clearances for each direction
        required_clearances = {
            'front': self.clearance_front + self.lidar_to_front,  # Total front clearance
            'left': self.clearance_sides + self.robot_width/2,   # Side clearance
            'right': self.clearance_sides + self.robot_width/2,  # Side clearance 
            'back': self.clearance_back + self.lidar_to_back     # Back clearance
        }
        
        # Check each LIDAR point
        for point in scan.points:
            angle = point.angle
            distance = point.distance
            
            # Normalize angle to [-π, π]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            # Check which sector this point belongs to
            for direction, (min_angle, max_angle) in sectors.items():
                if direction == 'back':
                    # Special handling for back sector (wraps around ±π)
                    if angle >= min_angle or angle <= max_angle:
                        if distance < required_clearances[direction]:
                            clearance_status[direction] = False
                            clearance_status['safe'] = False
                        clearance_status['obstacle_distances'][direction] = min(
                            clearance_status['obstacle_distances'][direction], distance
                        )
                else:
                    if min_angle <= angle <= max_angle:
                        if distance < required_clearances[direction]:
                            clearance_status[direction] = False
                            clearance_status['safe'] = False
                        clearance_status['obstacle_distances'][direction] = min(
                            clearance_status['obstacle_distances'][direction], distance
                        )
        
        # Find closest obstacle
        min_distance = float('inf')
        closest_direction = None
        for direction, distance in clearance_status['obstacle_distances'].items():
            if distance < min_distance:
                min_distance = distance
                closest_direction = direction
        
        if min_distance < float('inf'):
            clearance_status['closest_obstacle'] = {
                'direction': closest_direction,
                'distance': min_distance
            }
        
        return clearance_status
    
    def get_obstacle_avoidance_command(self) -> Tuple[float, float]:
        """
        Get motor commands for obstacle avoidance
        
        Returns:
            Tuple of (left_speed, right_speed) adjustments (-1.0 to 1.0)
        """
        clearance = self.check_robot_clearance()
        
        # Update obstacle detection state
        self.obstacle_detected = not clearance['safe']
        self.last_obstacle_check = time.time()
        
        if clearance['safe']:
            self.avoidance_active = False
            return (0.0, 0.0)  # No avoidance needed
        
        self.avoidance_active = True
        
        # Determine avoidance strategy based on which directions are blocked
        left_speed = 0.0
        right_speed = 0.0
        
        # Priority: avoid immediate collision
        if not clearance['front']:
            # Front obstacle - stop or back up and turn
            if clearance['left'] and not clearance['right']:
                # Turn left
                left_speed = -0.5
                right_speed = 0.5
                self.obstacle_direction = 'front_turn_left'
            elif clearance['right'] and not clearance['left']:
                # Turn right
                left_speed = 0.5
                right_speed = -0.5
                self.obstacle_direction = 'front_turn_right'
            elif clearance['left'] and clearance['right']:
                # Both sides clear - turn toward more space
                left_dist = clearance['obstacle_distances']['left']
                right_dist = clearance['obstacle_distances']['right']
                if left_dist > right_dist:
                    left_speed = -0.3
                    right_speed = 0.3
                    self.obstacle_direction = 'front_turn_left'
                else:
                    left_speed = 0.3
                    right_speed = -0.3
                    self.obstacle_direction = 'front_turn_right'
            else:
                # Front and sides blocked - back up
                left_speed = -0.3
                right_speed = -0.3
                self.obstacle_direction = 'front_backup'
        
        elif not clearance['left']:
            # Left obstacle - turn right slightly
            left_speed = 0.2
            right_speed = -0.1
            self.obstacle_direction = 'left'
        
        elif not clearance['right']:
            # Right obstacle - turn left slightly
            left_speed = -0.1
            right_speed = 0.2
            self.obstacle_direction = 'right'
        
        elif not clearance['back']:
            # Back obstacle - move forward
            left_speed = 0.2
            right_speed = 0.2
            self.obstacle_direction = 'back'
        
        return (left_speed, right_speed)
    
    def get_lidar_obstacles(self) -> List[Tuple[float, float]]:
        """
        Get obstacle positions from LIDAR mapping
        
        Returns:
            List of (x, y) obstacle positions in world coordinates
        """
        if not self.mapper:
            return []
        
        obstacles = []
        map_data = self.mapper.get_map_data_for_display()
        grid = map_data['grid']
        
        # Find occupied cells and convert to world coordinates
        occupied_threshold = 0.7
        for gy in range(0, map_data['height'], 3):  # Sample every 3rd cell for performance
            for gx in range(0, map_data['width'], 3):
                if grid[gy, gx] > occupied_threshold:
                    world_x, world_y = self.mapper.grid_to_world(gx, gy)
                    obstacles.append((world_x, world_y))
        
        return obstacles
    
    def is_path_clear_lidar(self, start_pos: Tuple[float, float], 
                           end_pos: Tuple[float, float]) -> bool:
        """
        Check if path is clear using LIDAR mapping data
        
        Args:
            start_pos: Starting position (x, y)
            end_pos: Ending position (x, y)
            
        Returns:
            True if path appears clear, False if obstacles detected
        """
        if not self.mapper:
            return True  # No LIDAR data, assume clear
        
        # Simple line-of-sight check using occupancy grid
        map_data = self.mapper.get_map_data_for_display()
        
        # Convert positions to grid coordinates
        start_gx, start_gy = self.mapper.world_to_grid(*start_pos)
        end_gx, end_gy = self.mapper.world_to_grid(*end_pos)
        
        # Use Bresenham's line algorithm to check path
        line_points = self.mapper.bresenham_line(start_gx, start_gy, end_gx, end_gy)
        
        obstacle_threshold = 0.6
        for gx, gy in line_points:
            if self.mapper.is_valid_grid_point(gx, gy):
                if map_data['grid'][gy, gx] > obstacle_threshold:
                    return False  # Obstacle found along path
        
        return True  # Path appears clear
    
    def draw_robot_footprint(self, surface: pygame.Surface, robot_x: float, robot_y: float, robot_theta: float):
        """Draw robot footprint with clearance zones - robot fixed at center"""
        try:
            # Robot is always at the center of the display
            screen_x = int(self.lidar_overlay_size[0] / 2)
            screen_y = int(self.lidar_overlay_size[1] / 2)
            
            # Calculate robot corners in world coordinates
            cos_theta = math.cos(robot_theta)
            sin_theta = math.sin(robot_theta)
            
            # Robot body corners (relative to LIDAR center)
            corners = [
                (-self.lidar_to_back, -self.robot_width/2),  # Back left
                (self.lidar_to_front, -self.robot_width/2),  # Front left  
                (self.lidar_to_front, self.robot_width/2),   # Front right
                (-self.lidar_to_back, self.robot_width/2)    # Back right
            ]
            
            # Transform corners relative to robot center (which is at screen center)
            screen_corners = []
            pixels_per_meter = 50  # Scale factor
            
            for dx, dy in corners:
                # Rotate corner relative to robot orientation
                rotated_x = dx * cos_theta - dy * sin_theta
                rotated_y = dx * sin_theta + dy * cos_theta
                
                # Convert to screen coordinates (robot is at center)
                corner_screen_x = int(screen_x + rotated_x * pixels_per_meter)
                corner_screen_y = int(screen_y - rotated_y * pixels_per_meter)  # Flip Y axis
                
                screen_corners.append((corner_screen_x, corner_screen_y))
            
            # Draw robot body
            color = (0, 255, 0) if not self.obstacle_detected else (255, 100, 100)
            pygame.draw.polygon(surface, color, screen_corners, 2)
            
            # Draw clearance zones if obstacle detected
            if self.obstacle_detected:
                clearance_corners = [
                    (-self.lidar_to_back - self.clearance_back, -self.robot_width/2 - self.clearance_sides),
                    (self.lidar_to_front + self.clearance_front, -self.robot_width/2 - self.clearance_sides),
                    (self.lidar_to_front + self.clearance_front, self.robot_width/2 + self.clearance_sides),
                    (-self.lidar_to_back - self.clearance_back, self.robot_width/2 + self.clearance_sides)
                ]
                
                clearance_screen_corners = []
                for dx, dy in clearance_corners:
                    # Rotate clearance zone relative to robot orientation
                    rotated_x = dx * cos_theta - dy * sin_theta
                    rotated_y = dx * sin_theta + dy * cos_theta
                    
                    # Convert to screen coordinates (robot is at center)
                    clear_screen_x = int(screen_x + rotated_x * pixels_per_meter)
                    clear_screen_y = int(screen_y - rotated_y * pixels_per_meter)
                    
                    clearance_screen_corners.append((clear_screen_x, clear_screen_y))
                
                pygame.draw.polygon(surface, (255, 255, 0), clearance_screen_corners, 1)
            
            # Draw direction arrow (pointing forward)
            arrow_length = 0.1  # 10cm arrow
            arrow_screen_x = int(screen_x + arrow_length * cos_theta * pixels_per_meter)
            arrow_screen_y = int(screen_y - arrow_length * sin_theta * pixels_per_meter)  # Flip Y axis
            
            pygame.draw.line(surface, (255, 255, 255),
                           (screen_x, screen_y),
                           (arrow_screen_x, arrow_screen_y), 2)
            
        except Exception as e:
            self.logger.error(f"❌ Robot footprint draw error: {e}")
    
    def draw_occupancy_grid_centered(self, surface: pygame.Surface, map_data: dict):
        """Draw occupancy grid with robot at center"""
        try:
            if not map_data or 'grid' not in map_data:
                return
            
            grid = map_data['grid']
            resolution = map_data['resolution']
            pixels_per_meter = 50
            
            # Robot position in screen coordinates (center)
            robot_screen_x = self.lidar_overlay_size[0] // 2
            robot_screen_y = self.lidar_overlay_size[1] // 2
            
            # Current robot position in world coordinates
            robot_world_x = self.mapper.robot_x
            robot_world_y = self.mapper.robot_y
            
            # Draw grid cells around robot
            for gy in range(0, map_data['height'], 2):  # Sample every 2nd cell for performance
                for gx in range(0, map_data['width'], 2):
                    occupancy = grid[gy, gx]
                    
                    if occupancy > 0.3:  # Only draw occupied cells
                        # Convert grid coordinates to world coordinates
                        world_x, world_y = self.mapper.grid_to_world(gx, gy)
                        
                        # Calculate relative position to robot
                        rel_x = world_x - robot_world_x
                        rel_y = world_y - robot_world_y
                        
                        # Convert to screen coordinates
                        screen_x = int(robot_screen_x + rel_x * pixels_per_meter)
                        screen_y = int(robot_screen_y - rel_y * pixels_per_meter)  # Flip Y axis
                        
                        # Only draw if within display bounds
                        if (0 <= screen_x < self.lidar_overlay_size[0] and 
                            0 <= screen_y < self.lidar_overlay_size[1]):
                            
                            # Color based on occupancy probability
                            intensity = int(occupancy * 255)
                            color = (intensity, intensity, intensity)
                            
                            # Draw small rectangle for grid cell
                            cell_size = max(1, int(resolution * pixels_per_meter))
                            pygame.draw.rect(surface, color, 
                                           (screen_x - cell_size//2, screen_y - cell_size//2, 
                                            cell_size, cell_size))
                            
        except Exception as e:
            self.logger.error(f"❌ Occupancy grid draw error: {e}")
    
    def draw_lidar_points_centered(self, surface: pygame.Surface, scan):
        """Draw LIDAR points with robot at center"""
        try:
            if not scan or not scan.points:
                return
            
            pixels_per_meter = 50
            robot_screen_x = self.lidar_overlay_size[0] // 2
            robot_screen_y = self.lidar_overlay_size[1] // 2
            
            for point in scan.points:
                if 0.05 <= point.distance <= 6.0:  # Valid range
                    # Convert polar to cartesian (relative to robot)
                    rel_x = point.distance * math.cos(point.angle)
                    rel_y = point.distance * math.sin(point.angle)
                    
                    # Convert to screen coordinates
                    screen_x = int(robot_screen_x + rel_x * pixels_per_meter)
                    screen_y = int(robot_screen_y - rel_y * pixels_per_meter)  # Flip Y axis
                    
                    # Only draw if within display bounds
                    if (0 <= screen_x < self.lidar_overlay_size[0] and 
                        0 <= screen_y < self.lidar_overlay_size[1]):
                        
                        # Color based on distance
                        if point.distance < 0.5:
                            color = (255, 0, 0)  # Red for close obstacles
                        else:
                            intensity = max(100, 255 - int(point.distance * 30))
                            color = (0, intensity, 255)  # Blue gradient
                        
                        pygame.draw.circle(surface, color, (screen_x, screen_y), 2)
                        
        except Exception as e:
            self.logger.error(f"❌ LIDAR points draw error: {e}")
    
    def draw_lidar_overlay(self, screen: pygame.Surface):
        """Draw LIDAR mapping overlay on main screen"""
        if not (self.show_lidar_overlay and self.enable_lidar_mapping and 
                self.lidar_plot and self.mapper):
            return
        
        try:
            # Create overlay surface
            overlay_surface = pygame.Surface(self.lidar_overlay_size)
            overlay_surface.fill((20, 20, 30))  # Dark background
            
            # Draw LIDAR plot on overlay
            self.lidar_plot.draw_grid(overlay_surface, self.lidar_display_config)
            
            # Draw occupancy grid centered on robot
            map_data = self.mapper.get_map_data_for_display()
            self.draw_occupancy_grid_centered(overlay_surface, map_data)
            
            # Draw current LIDAR points relative to robot center
            scan = self.lidar.get_latest_scan() if self.lidar else None
            if scan:
                self.draw_lidar_points_centered(overlay_surface, scan)
            
            # Draw robot with footprint and clearance zones
            self.draw_robot_footprint(
                overlay_surface,
                self.mapper.robot_x,
                self.mapper.robot_y,
                self.mapper.robot_theta
            )
            
            # Draw obstacle warning indicators
            if self.obstacle_detected:
                # Flash red border when obstacle detected
                flash_color = (255, 0, 0) if int(time.time() * 4) % 2 else (255, 100, 100)
                pygame.draw.rect(overlay_surface, flash_color, overlay_surface.get_rect(), 3)
                
                # Show obstacle direction
                font = pygame.font.Font(None, 14)
                if self.obstacle_direction:
                    warning_text = font.render(f"⚠ {self.obstacle_direction.upper()}", True, (255, 255, 0))
                    overlay_surface.blit(warning_text, (5, 20))
            else:
                # Normal border
                pygame.draw.rect(overlay_surface, (100, 100, 100), overlay_surface.get_rect(), 2)
            
            # Add title with status
            font = pygame.font.Font(None, 16)
            status_text = "OBSTACLE!" if self.obstacle_detected else "CLEAR"
            title_color = (255, 0, 0) if self.obstacle_detected else (255, 255, 255)
            title_text = font.render(f"LIDAR - {status_text}", True, title_color)
            overlay_surface.blit(title_text, (5, 5))
            
            # Blit overlay to main screen
            screen.blit(overlay_surface, self.lidar_display_rect)
            
        except Exception as e:
            self.logger.error(f"❌ LIDAR overlay draw error: {e}")
    
    def draw_lidar_info(self, screen: pygame.Surface):
        """Draw LIDAR information in the main UI"""
        if not self.enable_lidar_mapping:
            return
        
        try:
            # Get statistics
            lidar_stats = self.lidar.get_statistics() if self.lidar else {}
            mapper_stats = self.mapper.get_statistics() if self.mapper else {}
            clearance = self.check_robot_clearance()
            
            # Prepare info text
            font = self.base_controller.small_font
            y_pos = self.base_controller.height - 100  # Above existing UI
            
            # LIDAR status
            if lidar_stats.get('connected', False):
                status_text = f"LIDAR: {lidar_stats.get('scan_frequency', 0):.1f}Hz"
                status_color = (0, 255, 0)
            else:
                status_text = "LIDAR: DISCONNECTED"
                status_color = (255, 0, 0)
            
            text_surface = font.render(status_text, True, status_color)
            screen.blit(text_surface, (5, y_pos))
            
            # Obstacle detection status
            if self.obstacle_detected:
                obstacle_text = f"OBSTACLE: {self.obstacle_direction or 'DETECTED'}"
                obstacle_color = (255, 0, 0)
            else:
                obstacle_text = "CLEARANCE: OK"
                obstacle_color = (0, 255, 0)
            
            obstacle_surface = font.render(obstacle_text, True, obstacle_color)
            screen.blit(obstacle_surface, (5, y_pos + 12))
            
            # Clearance distances
            if clearance.get('obstacle_distances'):
                distances = clearance['obstacle_distances']
                clearance_text = f"F:{distances['front']:.2f}m L:{distances['left']:.2f}m R:{distances['right']:.2f}m B:{distances['back']:.2f}m"
                clearance_surface = font.render(clearance_text, True, (200, 200, 200))
                screen.blit(clearance_surface, (5, y_pos + 24))
            
            # Avoidance status
            if self.avoidance_active:
                avoidance_text = "AVOIDANCE: ACTIVE"
                avoidance_color = (255, 255, 0)
            else:
                avoidance_text = "AVOIDANCE: STANDBY"
                avoidance_color = (100, 100, 100)
            
            avoidance_surface = font.render(avoidance_text, True, avoidance_color)
            screen.blit(avoidance_surface, (5, y_pos + 36))
            
            # Mapping info
            if mapper_stats:
                map_text = f"MAP: {mapper_stats.get('total_scans_processed', 0)} scans"
                map_surface = font.render(map_text, True, (200, 200, 200))
                screen.blit(map_surface, (5, y_pos + 48))
            
        except Exception as e:
            self.logger.error(f"❌ LIDAR info draw error: {e}")
    
    def get_enhanced_navigation_info(self) -> Dict[str, Any]:
        """Get enhanced navigation information including LIDAR data"""
        base_info = self.base_controller.navigator.get_navigation_info()
        
        if self.enable_lidar_mapping and self.lidar and self.mapper:
            lidar_stats = self.lidar.get_statistics()
            mapper_stats = self.mapper.get_statistics()
            
            base_info.update({
                'lidar_connected': lidar_stats.get('connected', False),
                'lidar_scanning': lidar_stats.get('is_scanning', False),
                'lidar_frequency': lidar_stats.get('scan_frequency', 0.0),
                'map_scans_processed': mapper_stats.get('total_scans_processed', 0),
                'lidar_obstacles_detected': len(self.get_lidar_obstacles())
            })
        
        return base_info
    
    def cleanup(self):
        """Cleanup LIDAR resources"""
        self.logger.info("🧹 Cleaning up LIDAR integration...")
        self.stop_lidar_mapping()


def integrate_lidar_with_pathfinding(pathfinding_controller, 
                                   lidar_port: str = '/dev/ttyUSB1',
                                   enable_mapping: bool = True) -> LidarEnhancedPathfindingController:
    """
    Factory function to integrate LIDAR mapping with existing pathfinding controller
    
    Args:
        pathfinding_controller: Existing PathfindingRobotController instance
        lidar_port: Serial port for LIDAR connection
        enable_mapping: Enable LIDAR mapping functionality
        
    Returns:
        LidarEnhancedPathfindingController instance
    """
    enhanced_controller = LidarEnhancedPathfindingController(
        pathfinding_controller=pathfinding_controller,
        lidar_port=lidar_port,
        enable_lidar_mapping=enable_mapping
    )
    
    if enable_mapping:
        enhanced_controller.start_lidar_mapping()
    
    return enhanced_controller


# Example integration with existing pathfinding controller
if __name__ == "__main__":
    import sys
    import os
    
    # Add parent directory to path to import pathfinding controller
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    try:
        from pathfinding_robot_controller import PathfindingRobotController
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        
        # Create base pathfinding controller
        base_controller = PathfindingRobotController(
            port='/dev/ttyUSB0',
            baudrate=115200,
            enable_collision_avoidance=False
        )
        
        # Enhance with LIDAR mapping
        enhanced_controller = integrate_lidar_with_pathfinding(
            pathfinding_controller=base_controller,
            lidar_port='/dev/ttyUSB1',
            enable_mapping=True
        )
        
        print("🚀 LIDAR-enhanced pathfinding controller ready!")
        print("Press Ctrl+C to stop...")
        
        # Run enhanced controller (this would need modification to the base controller)
        # base_controller.run()
        
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("Make sure pathfinding_robot_controller.py is available")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

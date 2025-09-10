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
        
        # LIDAR components
        self.lidar: Optional[YDLidarX2Optimized] = None
        self.mapper: Optional[LidarMapper] = None
        self.lidar_plot: Optional[CartesianPlot] = None
        
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
            
            # Draw occupancy grid
            map_data = self.mapper.get_map_data_for_display()
            self.lidar_plot.draw_occupancy_grid(
                overlay_surface,
                map_data['grid'],
                map_data['width'],
                map_data['height'],
                map_data['origin_x'],
                map_data['origin_y'],
                map_data['resolution'],
                self.lidar_display_config
            )
            
            # Draw current LIDAR points
            scan = self.lidar.get_latest_scan() if self.lidar else None
            if scan:
                points = self.mapper.get_cartesian_points(scan)
                self.lidar_plot.draw_lidar_points(overlay_surface, points, self.lidar_display_config)
            
            # Draw robot
            self.lidar_plot.draw_robot(
                overlay_surface,
                self.mapper.robot_x,
                self.mapper.robot_y,
                self.mapper.robot_theta,
                self.lidar_display_config
            )
            
            # Draw border
            pygame.draw.rect(overlay_surface, (100, 100, 100), 
                           overlay_surface.get_rect(), 2)
            
            # Add title
            font = pygame.font.Font(None, 16)
            title_text = font.render("LIDAR MAP", True, (255, 255, 255))
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
            
            # Prepare info text
            font = self.base_controller.small_font
            y_pos = self.base_controller.height - 60  # Above existing UI
            
            # LIDAR status
            if lidar_stats.get('connected', False):
                status_text = f"LIDAR: {lidar_stats.get('scan_frequency', 0):.1f}Hz"
                status_color = (0, 255, 0)
            else:
                status_text = "LIDAR: DISCONNECTED"
                status_color = (255, 0, 0)
            
            text_surface = font.render(status_text, True, status_color)
            screen.blit(text_surface, (5, y_pos))
            
            # Mapping info
            if mapper_stats:
                map_text = f"MAP: {mapper_stats.get('total_scans_processed', 0)} scans"
                map_surface = font.render(map_text, True, (200, 200, 200))
                screen.blit(map_surface, (5, y_pos + 12))
            
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

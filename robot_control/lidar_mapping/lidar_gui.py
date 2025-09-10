#!/usr/bin/env python3
"""
LIDAR Mapping GUI Module
========================

This module provides a graphical user interface for displaying LIDAR mapping data
with a Cartesian plot. Designed for 480x320 display resolution.
"""

import pygame
import math
import time
import threading
import numpy as np
import logging
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass

# Handle both relative and absolute imports
try:
    from .ydlidar_x2_optimized import YDLidarX2Optimized, LidarScan, LidarConfig
    from .lidar_mapper import LidarMapper
except ImportError:
    # Fallback for direct execution
    from ydlidar_x2_optimized import YDLidarX2Optimized, LidarScan, LidarConfig
    from lidar_mapper import LidarMapper


@dataclass
class DisplayConfig:
    """Display configuration for the LIDAR GUI"""
    width: int = 480
    height: int = 320
    fps: int = 30
    background_color: Tuple[int, int, int] = (20, 20, 30)
    grid_color: Tuple[int, int, int] = (40, 40, 60)
    lidar_point_color: Tuple[int, int, int] = (0, 255, 0)
    robot_color: Tuple[int, int, int] = (255, 100, 100)
    obstacle_color: Tuple[int, int, int] = (255, 255, 255)
    free_space_color: Tuple[int, int, int] = (100, 100, 100)


class CartesianPlot:
    """
    Cartesian coordinate plotting system for LIDAR data
    
    Handles coordinate transformations and rendering of LIDAR points
    on a Cartesian grid within the display area.
    """
    
    def __init__(self, 
                 display_width: int,
                 display_height: int,
                 world_width: float = 8.0,
                 world_height: float = 8.0,
                 plot_margin: int = 40):
        """
        Initialize Cartesian plot
        
        Args:
            display_width: Display width in pixels
            display_height: Display height in pixels  
            world_width: World coordinate width in meters
            world_height: World coordinate height in meters
            plot_margin: Margin around plot area in pixels
        """
        self.display_width = display_width
        self.display_height = display_height
        self.world_width = world_width
        self.world_height = world_height
        self.plot_margin = plot_margin
        
        # Calculate plot area
        self.plot_width = display_width - 2 * plot_margin
        self.plot_height = display_height - 2 * plot_margin
        self.plot_x = plot_margin
        self.plot_y = plot_margin
        
        # Calculate scale factors
        self.scale_x = self.plot_width / world_width
        self.scale_y = self.plot_height / world_height
        
        # Use uniform scaling to maintain aspect ratio
        self.scale = min(self.scale_x, self.scale_y)
        
        # Center the plot
        actual_plot_width = world_width * self.scale
        actual_plot_height = world_height * self.scale
        self.plot_x = (display_width - actual_plot_width) // 2
        self.plot_y = (display_height - actual_plot_height) // 2
        
        self.logger = logging.getLogger('CartesianPlot')
        self.logger.info(f"📊 Cartesian plot initialized:")
        self.logger.info(f"   Display: {display_width}x{display_height}")
        self.logger.info(f"   World: {world_width}x{world_height}m")
        self.logger.info(f"   Scale: {self.scale:.1f} pixels/meter")
    
    def world_to_screen(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to screen coordinates
        
        Args:
            world_x: X coordinate in meters
            world_y: Y coordinate in meters
            
        Returns:
            Tuple of (screen_x, screen_y) coordinates
        """
        # Center world coordinates (0,0 at center of world)
        centered_x = world_x + self.world_width / 2
        centered_y = world_y + self.world_height / 2
        
        # Convert to screen coordinates
        screen_x = int(self.plot_x + centered_x * self.scale)
        screen_y = int(self.plot_y + (self.world_height - centered_y) * self.scale)
        
        return screen_x, screen_y
    
    def screen_to_world(self, screen_x: int, screen_y: int) -> Tuple[float, float]:
        """
        Convert screen coordinates to world coordinates
        
        Args:
            screen_x: Screen X coordinate
            screen_y: Screen Y coordinate
            
        Returns:
            Tuple of (world_x, world_y) coordinates
        """
        # Convert to plot-relative coordinates
        plot_x = (screen_x - self.plot_x) / self.scale
        plot_y = (screen_y - self.plot_y) / self.scale
        
        # Convert to world coordinates (centered)
        world_x = plot_x - self.world_width / 2
        world_y = (self.world_height - plot_y) - self.world_height / 2
        
        return world_x, world_y
    
    def is_point_in_plot(self, screen_x: int, screen_y: int) -> bool:
        """Check if screen coordinates are within plot area"""
        return (self.plot_x <= screen_x <= self.plot_x + self.world_width * self.scale and
                self.plot_y <= screen_y <= self.plot_y + self.world_height * self.scale)
    
    def draw_grid(self, surface: pygame.Surface, config: DisplayConfig):
        """Draw coordinate grid on the plot"""
        # Draw plot background
        plot_rect = pygame.Rect(
            self.plot_x, 
            self.plot_y,
            int(self.world_width * self.scale),
            int(self.world_height * self.scale)
        )
        pygame.draw.rect(surface, (10, 10, 20), plot_rect)
        pygame.draw.rect(surface, config.grid_color, plot_rect, 2)
        
        # Draw grid lines every meter
        grid_spacing = 1.0  # 1 meter grid
        
        # Vertical lines
        for x in range(int(-self.world_width/2), int(self.world_width/2) + 1):
            if x == 0:
                color = (100, 100, 150)  # Highlight center lines
                width = 2
            else:
                color = config.grid_color
                width = 1
            
            screen_x, _ = self.world_to_screen(float(x), 0)
            pygame.draw.line(surface, color,
                           (screen_x, self.plot_y),
                           (screen_x, self.plot_y + int(self.world_height * self.scale)),
                           width)
        
        # Horizontal lines
        for y in range(int(-self.world_height/2), int(self.world_height/2) + 1):
            if y == 0:
                color = (100, 100, 150)  # Highlight center lines
                width = 2
            else:
                color = config.grid_color
                width = 1
            
            _, screen_y = self.world_to_screen(0, float(y))
            pygame.draw.line(surface, color,
                           (self.plot_x, screen_y),
                           (self.plot_x + int(self.world_width * self.scale), screen_y),
                           width)
    
    def draw_lidar_points(self, surface: pygame.Surface, points: List[Tuple[float, float]], 
                         config: DisplayConfig):
        """Draw LIDAR points on the plot"""
        for world_x, world_y in points:
            screen_x, screen_y = self.world_to_screen(world_x, world_y)
            
            # Only draw if point is within plot area
            if self.is_point_in_plot(screen_x, screen_y):
                pygame.draw.circle(surface, config.lidar_point_color, 
                                 (screen_x, screen_y), 2)
    
    def draw_robot(self, surface: pygame.Surface, robot_x: float, robot_y: float, 
                   robot_theta: float, config: DisplayConfig):
        """Draw robot position and orientation"""
        screen_x, screen_y = self.world_to_screen(robot_x, robot_y)
        
        if self.is_point_in_plot(screen_x, screen_y):
            # Draw robot body
            pygame.draw.circle(surface, config.robot_color, (screen_x, screen_y), 8)
            pygame.draw.circle(surface, (255, 255, 255), (screen_x, screen_y), 8, 2)
            
            # Draw orientation arrow
            arrow_length = 15
            end_x = screen_x + int(arrow_length * math.cos(robot_theta))
            end_y = screen_y - int(arrow_length * math.sin(robot_theta))  # Negative for screen coordinates
            pygame.draw.line(surface, (255, 255, 0), (screen_x, screen_y), (end_x, end_y), 3)
    
    def draw_occupancy_grid(self, surface: pygame.Surface, grid_data: np.ndarray,
                           grid_width: int, grid_height: int, 
                           origin_x: float, origin_y: float, resolution: float,
                           config: DisplayConfig):
        """Draw occupancy grid as background"""
        for gy in range(0, grid_height, 2):  # Skip every other cell for performance
            for gx in range(0, grid_width, 2):
                # Convert grid to world coordinates
                world_x = origin_x + (gx + 0.5) * resolution
                world_y = origin_y + (gy + 0.5) * resolution
                
                # Get occupancy probability
                prob = grid_data[gy, gx]
                
                # Only draw if significantly occupied or free
                if prob > 0.7:  # Occupied
                    screen_x, screen_y = self.world_to_screen(world_x, world_y)
                    if self.is_point_in_plot(screen_x, screen_y):
                        intensity = int(255 * prob)
                        color = (intensity, intensity, intensity)
                        pygame.draw.rect(surface, color,
                                       (screen_x-1, screen_y-1, 2, 2))
                elif prob < 0.3:  # Free space
                    screen_x, screen_y = self.world_to_screen(world_x, world_y)
                    if self.is_point_in_plot(screen_x, screen_y):
                        intensity = int(100 * (1.0 - prob))
                        color = (0, intensity//3, 0)
                        pygame.draw.rect(surface, color,
                                       (screen_x-1, screen_y-1, 2, 2))


class LidarMappingGUI:
    """
    Main GUI class for LIDAR mapping visualization
    
    Provides real-time display of LIDAR data and occupancy mapping
    on a 480x320 resolution display with Cartesian plotting.
    """
    
    def __init__(self, 
                 lidar_port: str = '/dev/ttyUSB1',
                 display_width: int = 480,
                 display_height: int = 320):
        """
        Initialize LIDAR mapping GUI
        
        Args:
            lidar_port: Serial port for LIDAR connection
            display_width: Display width in pixels
            display_height: Display height in pixels
        """
        self.display_width = display_width
        self.display_height = display_height
        
        # Display configuration
        self.config = DisplayConfig(width=display_width, height=display_height)
        
        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((display_width, display_height))
        pygame.display.set_caption("LIDAR Mapping - YDLIDAR X2")
        self.clock = pygame.time.Clock()
        
        # Initialize fonts
        self.font_small = pygame.font.Font(None, 16)
        self.font_medium = pygame.font.Font(None, 20)
        
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
        
        # Initialize Cartesian plot
        self.plot = CartesianPlot(
            display_width=display_width,
            display_height=display_height,
            world_width=8.0,
            world_height=8.0,
            plot_margin=30
        )
        
        # GUI state
        self.running = False
        self.show_grid = True
        self.show_lidar_points = True
        self.show_occupancy_grid = True
        self.show_robot = True
        self.show_info = True
        
        # Performance tracking
        self.fps = 0
        self.last_frame_time = time.time()
        
        # Setup logging
        self.logger = logging.getLogger('LidarMappingGUI')
        self.logger.setLevel(logging.INFO)
        
        self.logger.info(f"🖥️  LIDAR Mapping GUI initialized ({display_width}x{display_height})")
    
    def connect_lidar(self) -> bool:
        """Connect to LIDAR sensor"""
        if self.lidar.connect():
            if self.lidar.start_scanning():
                self.logger.info("✅ LIDAR connected and scanning")
                return True
            else:
                self.logger.error("❌ Failed to start LIDAR scanning")
                return False
        else:
            self.logger.error("❌ Failed to connect to LIDAR")
            return False
    
    def disconnect_lidar(self):
        """Disconnect from LIDAR sensor"""
        self.lidar.cleanup()
        self.logger.info("🔌 LIDAR disconnected")
    
    def handle_events(self) -> bool:
        """
        Handle pygame events
        
        Returns:
            False if should quit, True otherwise
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    return False
                elif event.key == pygame.K_g:
                    self.show_grid = not self.show_grid
                elif event.key == pygame.K_l:
                    self.show_lidar_points = not self.show_lidar_points
                elif event.key == pygame.K_o:
                    self.show_occupancy_grid = not self.show_occupancy_grid
                elif event.key == pygame.K_r:
                    self.show_robot = not self.show_robot
                elif event.key == pygame.K_i:
                    self.show_info = not self.show_info
                elif event.key == pygame.K_c:
                    self.mapper.clear_map()
                elif event.key == pygame.K_s:
                    # Save map
                    timestamp = int(time.time())
                    filename = f"lidar_map_{timestamp}.npy"
                    self.mapper.save_map(filename)
                    self.logger.info(f"💾 Map saved as {filename}")
                elif event.key == pygame.K_SPACE:
                    # Toggle LIDAR scanning
                    if self.lidar.is_scanning:
                        self.lidar.stop_scanning()
                        self.logger.info("⏸️  LIDAR scanning paused")
                    else:
                        self.lidar.start_scanning()
                        self.logger.info("▶️  LIDAR scanning resumed")
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    # Convert mouse position to world coordinates
                    world_x, world_y = self.plot.screen_to_world(*event.pos)
                    self.logger.info(f"🖱️  Clicked: ({world_x:.2f}, {world_y:.2f})")
        
        return True
    
    def update(self):
        """Update system state"""
        # Get latest LIDAR scan
        scan = self.lidar.get_latest_scan()
        
        if scan:
            # Update mapper with new scan data
            self.mapper.update_map(scan)
        
        # Update performance tracking
        current_time = time.time()
        if current_time - self.last_frame_time > 0:
            self.fps = 1.0 / (current_time - self.last_frame_time)
        self.last_frame_time = current_time
    
    def draw(self):
        """Draw everything on screen"""
        # Clear screen
        self.screen.fill(self.config.background_color)
        
        # Draw coordinate grid
        if self.show_grid:
            self.plot.draw_grid(self.screen, self.config)
        
        # Draw occupancy grid
        if self.show_occupancy_grid:
            map_data = self.mapper.get_map_data_for_display()
            self.plot.draw_occupancy_grid(
                self.screen,
                map_data['grid'],
                map_data['width'],
                map_data['height'],
                map_data['origin_x'],
                map_data['origin_y'],
                map_data['resolution'],
                self.config
            )
        
        # Draw LIDAR points
        if self.show_lidar_points:
            scan = self.lidar.get_latest_scan()
            if scan:
                points = self.mapper.get_cartesian_points(scan)
                self.plot.draw_lidar_points(self.screen, points, self.config)
        
        # Draw robot
        if self.show_robot:
            self.plot.draw_robot(
                self.screen,
                self.mapper.robot_x,
                self.mapper.robot_y,
                self.mapper.robot_theta,
                self.config
            )
        
        # Draw information panel
        if self.show_info:
            self.draw_info_panel()
        
        # Update display
        pygame.display.flip()
    
    def draw_info_panel(self):
        """Draw information panel with status and controls"""
        # Semi-transparent background
        info_height = 80
        info_rect = pygame.Rect(0, self.display_height - info_height, 
                               self.display_width, info_height)
        info_surface = pygame.Surface((self.display_width, info_height))
        info_surface.set_alpha(200)
        info_surface.fill((0, 0, 0))
        self.screen.blit(info_surface, (0, self.display_height - info_height))
        
        # Draw border
        pygame.draw.rect(self.screen, (100, 100, 100), info_rect, 1)
        
        # Get statistics
        lidar_stats = self.lidar.get_statistics()
        mapper_stats = self.mapper.get_statistics()
        
        # Prepare info text
        y_offset = self.display_height - info_height + 5
        
        # Line 1: Connection and scanning status
        status_text = "CONNECTED" if lidar_stats['connected'] else "DISCONNECTED"
        status_color = (0, 255, 0) if lidar_stats['connected'] else (255, 0, 0)
        
        scanning_text = "SCANNING" if lidar_stats['is_scanning'] else "STOPPED"
        scanning_color = (0, 255, 0) if lidar_stats['is_scanning'] else (255, 255, 0)
        
        line1 = f"LIDAR: {status_text} | {scanning_text} | {lidar_stats['scan_frequency']:.1f} Hz"
        text1 = self.font_small.render(line1, True, status_color)
        self.screen.blit(text1, (5, y_offset))
        y_offset += 16
        
        # Line 2: Mapping statistics
        line2 = f"SCANS: {mapper_stats['total_scans_processed']} | ROBOT: ({mapper_stats['robot_pose'][0]:.1f}, {mapper_stats['robot_pose'][1]:.1f})"
        text2 = self.font_small.render(line2, True, (200, 200, 200))
        self.screen.blit(text2, (5, y_offset))
        y_offset += 16
        
        # Line 3: Controls
        line3 = f"FPS: {self.fps:.1f} | G:Grid L:Points O:Map R:Robot I:Info C:Clear S:Save SPACE:Pause ESC:Exit"
        text3 = self.font_small.render(line3, True, (150, 150, 150))
        self.screen.blit(text3, (5, y_offset))
        y_offset += 16
        
        # Line 4: Toggle states
        toggles = []
        if self.show_grid: toggles.append("Grid")
        if self.show_lidar_points: toggles.append("Points") 
        if self.show_occupancy_grid: toggles.append("Map")
        if self.show_robot: toggles.append("Robot")
        
        line4 = f"SHOWING: {' | '.join(toggles)}"
        text4 = self.font_small.render(line4, True, (100, 200, 100))
        self.screen.blit(text4, (5, y_offset))
    
    def run(self):
        """Main application loop"""
        self.logger.info("🚀 Starting LIDAR Mapping GUI...")
        
        # Connect to LIDAR
        if not self.connect_lidar():
            self.logger.error("❌ Failed to connect to LIDAR. Running in demo mode.")
        
        self.running = True
        
        try:
            while self.running:
                # Handle events
                if not self.handle_events():
                    break
                
                # Update system
                self.update()
                
                # Draw everything
                self.draw()
                
                # Control frame rate
                self.clock.tick(self.config.fps)
        
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
        
        self.disconnect_lidar()
        pygame.quit()
        
        self.logger.info("👋 LIDAR Mapping GUI stopped")


# Main execution
if __name__ == "__main__":
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create and run GUI
    gui = LidarMappingGUI(
        lidar_port='/dev/ttyUSB1',
        display_width=480,
        display_height=320
    )
    
    gui.run()

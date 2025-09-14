#!/usr/bin/env python3
"""
SLAM Navigator with GUI
- First scans environment to build map
- Displays map in GUI
- Click-to-navigate functionality
- Uses pathfinding on generated map
"""

import pygame
import sys
import time
import math
import numpy as np
from typing import Optional, Tuple, List
from autonomous_navigator import AutonomousNavigator, Waypoint
from simple_slam import SimpleSLAM

class SLAMNavigatorGUI:
    def __init__(self, lidar_port='/dev/ttyUSB1', robot_port='/dev/ttyUSB0'):
        # Initialize pygame
        pygame.init()
        
        # Display setup
        self.width = 1200
        self.height = 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("🗺️ SLAM Navigator - Map & Navigate")
        
        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.GRAY = (128, 128, 128)
        self.LIGHT_GRAY = (200, 200, 200)
        self.DARK_GRAY = (64, 64, 64)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 100, 255)
        self.YELLOW = (255, 255, 0)
        self.CYAN = (0, 255, 255)
        self.ORANGE = (255, 165, 0)
        
        # Fonts
        self.font = pygame.font.Font(None, 24)
        self.large_font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 18)
        
        # Initialize navigator
        self.navigator = AutonomousNavigator(lidar_port, robot_port)
        self.slam = self.navigator.slam
        
        # GUI state
        self.mode = "MAPPING"  # MAPPING, NAVIGATION
        self.mapping_progress = 0
        self.scan_count = 0
        self.target_scans = 50  # Fewer scans needed for smaller map
        
        # Map display parameters
        self.map_display_x = 50
        self.map_display_y = 50
        self.map_display_size = 600
        self.map_scale = self.map_display_size / self.slam.map_size_m
        
        # Navigation
        self.selected_target = None
        self.current_path = []
        
        print("🗺️ SLAM Navigator GUI initialized")
        print(f"Map size: {self.slam.map_size_m}m x {self.slam.map_size_m}m")
        print(f"Resolution: {self.slam.resolution}m per cell")
        
    def world_to_screen(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates for map display"""
        screen_x = int(self.map_display_x + world_x * self.map_scale)
        screen_y = int(self.map_display_y + (self.slam.map_size_m - world_y) * self.map_scale)
        return screen_x, screen_y
    
    def screen_to_world(self, screen_x: int, screen_y: int) -> Tuple[float, float]:
        """Convert screen coordinates to world coordinates"""
        world_x = (screen_x - self.map_display_x) / self.map_scale
        world_y = self.slam.map_size_m - (screen_y - self.map_display_y) / self.map_scale
        return world_x, world_y
    
    def is_click_in_map(self, screen_x: int, screen_y: int) -> bool:
        """Check if click is within map display area"""
        return (self.map_display_x <= screen_x <= self.map_display_x + self.map_display_size and
                self.map_display_y <= screen_y <= self.map_display_y + self.map_display_size)
    
    def perform_mapping_scan(self):
        """Perform one mapping scan"""
        if not self.navigator.connect_lidar():
            print("❌ Failed to connect to LIDAR for mapping")
            return False
        
        # Get LIDAR points
        points = self.navigator.get_lidar_points()
        
        if points:
            # Update SLAM map
            self.slam.update_map(points)
            self.scan_count += 1
            self.mapping_progress = min(100, (self.scan_count / self.target_scans) * 100)
            
            print(f"📊 Mapping scan {self.scan_count}/{self.target_scans} - {len(points)} points")
            
            # Rotate robot slightly for better coverage
            if self.scan_count % 5 == 0:  # Every 5 scans for smaller map
                self.navigator.send_motor_commands(0, 0.15)  # Smaller rotation
                time.sleep(0.3)
                self.navigator.send_motor_commands(0, 0)
        
        return True
    
    def find_path_astar(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Simple A* pathfinding on SLAM map"""
        start_grid = self.slam.world_to_grid(*start)
        goal_grid = self.slam.world_to_grid(*goal)
        
        # Simple A* implementation
        from heapq import heappush, heappop
        
        open_set = [(0, start_grid)]
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}
        closed_set = set()
        
        while open_set:
            current = heappop(open_set)[1]
            
            if current == goal_grid:
                # Reconstruct path
                path = []
                while current in came_from:
                    world_pos = self.slam.grid_to_world(*current)
                    path.append(world_pos)
                    current = came_from[current]
                path.reverse()
                return path
            
            closed_set.add(current)
            
            # Check neighbors
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Check bounds
                if (neighbor[0] < 0 or neighbor[0] >= self.slam.grid_size or
                    neighbor[1] < 0 or neighbor[1] >= self.slam.grid_size):
                    continue
                
                # Check if occupied
                if self.slam.grid[neighbor[1], neighbor[0]] > 0.7:  # Occupied
                    continue
                
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + (1.414 if dx != 0 and dy != 0 else 1)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_grid)
                    heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path found
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic for A*"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def draw_map(self):
        """Draw the SLAM-generated map"""
        # Map background
        map_rect = pygame.Rect(self.map_display_x, self.map_display_y, 
                              self.map_display_size, self.map_display_size)
        pygame.draw.rect(self.screen, self.WHITE, map_rect)
        pygame.draw.rect(self.screen, self.BLACK, map_rect, 2)
        
        # Draw occupancy grid
        cell_size = self.map_scale * self.slam.resolution
        
        for i in range(self.slam.grid_size):
            for j in range(self.slam.grid_size):
                probability = self.slam.grid[j, i]  # Note: j,i for correct orientation
                
                if probability > 0.7:  # Occupied
                    color = self.BLACK
                elif probability < 0.3:  # Free
                    color = self.WHITE
                else:  # Unknown
                    gray_value = int(255 * (1 - probability))
                    color = (gray_value, gray_value, gray_value)
                
                if color != self.WHITE:  # Don't draw white cells (optimization)
                    cell_x = self.map_display_x + i * cell_size
                    cell_y = self.map_display_y + j * cell_size
                    cell_rect = pygame.Rect(cell_x, cell_y, max(1, int(cell_size)), max(1, int(cell_size)))
                    pygame.draw.rect(self.screen, color, cell_rect)
        
        # Draw robot position
        robot_screen = self.world_to_screen(self.slam.robot_x, self.slam.robot_y)
        pygame.draw.circle(self.screen, self.BLUE, robot_screen, 8)
        
        # Draw robot orientation
        robot_x, robot_y = robot_screen
        arrow_length = 15
        arrow_end_x = robot_x + arrow_length * math.cos(self.slam.robot_theta)
        arrow_end_y = robot_y - arrow_length * math.sin(self.slam.robot_theta)
        pygame.draw.line(self.screen, self.YELLOW, (robot_x, robot_y), 
                        (arrow_end_x, arrow_end_y), 3)
        
        # Draw target if selected
        if self.selected_target:
            target_screen = self.world_to_screen(*self.selected_target)
            pygame.draw.circle(self.screen, self.RED, target_screen, 6)
            pygame.draw.circle(self.screen, self.WHITE, target_screen, 3)
        
        # Draw path if available
        if self.current_path:
            path_points = [self.world_to_screen(x, y) for x, y in self.current_path]
            if len(path_points) > 1:
                pygame.draw.lines(self.screen, self.GREEN, False, path_points, 3)
    
    def draw_ui(self):
        """Draw user interface elements"""
        # Title
        title_text = self.large_font.render("🗺️ SLAM Navigator", True, self.BLACK)
        self.screen.blit(title_text, (10, 10))
        
        # Mode indicator
        mode_color = self.ORANGE if self.mode == "MAPPING" else self.GREEN
        mode_text = self.font.render(f"Mode: {self.mode}", True, mode_color)
        self.screen.blit(mode_text, (700, 60))
        
        # Status panel
        status_y = 100
        
        if self.mode == "MAPPING":
            # Mapping progress
            progress_text = self.font.render(f"Mapping Progress: {self.mapping_progress:.1f}%", True, self.BLACK)
            self.screen.blit(progress_text, (700, status_y))
            
            # Progress bar
            bar_rect = pygame.Rect(700, status_y + 25, 200, 20)
            pygame.draw.rect(self.screen, self.LIGHT_GRAY, bar_rect)
            progress_width = int(200 * self.mapping_progress / 100)
            if progress_width > 0:
                progress_rect = pygame.Rect(700, status_y + 25, progress_width, 20)
                pygame.draw.rect(self.screen, self.GREEN, progress_rect)
            pygame.draw.rect(self.screen, self.BLACK, bar_rect, 2)
            
            scan_text = self.small_font.render(f"Scans: {self.scan_count}/{self.target_scans}", True, self.BLACK)
            self.screen.blit(scan_text, (700, status_y + 50))
        
        else:  # NAVIGATION mode
            nav_text = self.font.render("Click on map to navigate", True, self.BLACK)
            self.screen.blit(nav_text, (700, status_y))
            
            if self.selected_target:
                target_text = self.small_font.render(f"Target: ({self.selected_target[0]:.2f}, {self.selected_target[1]:.2f})", True, self.BLACK)
                self.screen.blit(target_text, (700, status_y + 25))
        
        # Robot info
        robot_info_y = 200
        robot_texts = [
            f"Robot Position: ({self.slam.robot_x:.2f}, {self.slam.robot_y:.2f})",
            f"Robot Heading: {math.degrees(self.slam.robot_theta):.1f}°",
            f"Map Size: {self.slam.map_size_m}m x {self.slam.map_size_m}m",
            f"Resolution: {self.slam.resolution*100:.1f}cm/cell"
        ]
        
        for i, text in enumerate(robot_texts):
            rendered = self.small_font.render(text, True, self.BLACK)
            self.screen.blit(rendered, (700, robot_info_y + i * 20))
        
        # Controls
        controls_y = 300
        control_texts = [
            "Controls:",
            "SPACE - Switch to Navigation mode",
            "M - Continue mapping",
            "R - Reset map",
            "ESC - Exit"
        ]
        
        for i, text in enumerate(control_texts):
            color = self.BLACK if i == 0 else self.DARK_GRAY
            rendered = self.small_font.render(text, True, color)
            self.screen.blit(rendered, (700, controls_y + i * 18))
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_SPACE:
                    if self.mode == "MAPPING" and self.mapping_progress >= 50:
                        self.mode = "NAVIGATION"
                        print("🎯 Switched to NAVIGATION mode")
                    elif self.mode == "NAVIGATION":
                        self.mode = "MAPPING"
                        print("🗺️ Switched to MAPPING mode")
                elif event.key == pygame.K_m:
                    self.mode = "MAPPING"
                    print("🗺️ Continuing mapping...")
                elif event.key == pygame.K_r:
                    # Reset map
                    self.slam = SimpleSLAM(map_size_m=10.0, resolution=0.05)
                    self.navigator.slam = self.slam
                    self.scan_count = 0
                    self.mapping_progress = 0
                    self.selected_target = None
                    self.current_path = []
                    print("🔄 Map reset")
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1 and self.mode == "NAVIGATION":  # Left click
                    mouse_x, mouse_y = event.pos
                    if self.is_click_in_map(mouse_x, mouse_y):
                        # Convert to world coordinates
                        world_x, world_y = self.screen_to_world(mouse_x, mouse_y)
                        
                        # Check if target is in free space
                        if self.slam.is_cell_free(world_x, world_y):
                            self.selected_target = (world_x, world_y)
                            
                            # Find path
                            start = (self.slam.robot_x, self.slam.robot_y)
                            self.current_path = self.find_path_astar(start, self.selected_target)
                            
                            if self.current_path:
                                print(f"🎯 Path planned to ({world_x:.2f}, {world_y:.2f}) - {len(self.current_path)} waypoints")
                                # Start navigation
                                if self.current_path:
                                    first_waypoint = Waypoint(self.current_path[0][0], self.current_path[0][1], tolerance=0.1)
                                    self.navigator.navigate_to_waypoint(first_waypoint)
                            else:
                                print("❌ No path found to target")
                        else:
                            print("❌ Target is in occupied space")
        
        return True
    
    def update(self):
        """Update system state"""
        if self.mode == "MAPPING" and self.mapping_progress < 100:
            # Perform mapping
            if self.scan_count < self.target_scans:
                self.perform_mapping_scan()
                time.sleep(0.1)  # Small delay between scans
        
        elif self.mode == "NAVIGATION":
            # Update navigation if active
            if hasattr(self.navigator, 'target_waypoint') and self.navigator.target_waypoint:
                # Run navigation step
                waypoint_reached = self.navigator.run_navigation_step()
                
                if waypoint_reached and self.current_path:
                    # Move to next waypoint
                    self.current_path.pop(0)
                    if self.current_path:
                        next_waypoint = Waypoint(self.current_path[0][0], self.current_path[0][1], tolerance=0.1)
                        self.navigator.navigate_to_waypoint(next_waypoint)
                    else:
                        print("✅ Navigation completed!")
                        self.selected_target = None
    
    def run(self):
        """Main application loop"""
        clock = pygame.time.Clock()
        
        print("🚀 Starting SLAM Navigator GUI")
        print("📋 Instructions:")
        print("   1. Wait for mapping to complete (or press SPACE when >50%)")
        print("   2. Click on the map to navigate to that location")
        print("   3. Use M to continue mapping, R to reset")
        
        try:
            while True:
                # Handle events
                if not self.handle_events():
                    break
                
                # Update system
                self.update()
                
                # Draw everything
                self.screen.fill(self.WHITE)
                self.draw_map()
                self.draw_ui()
                
                # Update display
                pygame.display.flip()
                clock.tick(30)  # 30 FPS
        
        except KeyboardInterrupt:
            print("\n👋 Interrupted by user")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        print("🧹 Cleaning up...")
        self.navigator.cleanup()
        pygame.quit()
        print("👋 SLAM Navigator GUI stopped")

def main():
    gui = SLAMNavigatorGUI()
    gui.run()

if __name__ == "__main__":
    main()

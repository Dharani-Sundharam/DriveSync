#!/usr/bin/env python3
"""
Simple SLAM Implementation for LIDAR-based Navigation
- Occupancy grid mapping
- Simple localization using scan matching
- Path planning on generated map
"""

import numpy as np
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass

@dataclass
class GridMap:
    """Occupancy grid map representation"""
    width: int
    height: int
    resolution: float  # meters per cell
    origin_x: float    # world coordinates of grid origin
    origin_y: float
    data: np.ndarray   # occupancy probabilities (0-1)

class SimpleSLAM:
    def __init__(self, map_size_m: float = 3.0, resolution: float = 0.03):
        """
        Initialize simple SLAM system
        
        Args:
            map_size_m: Size of map in meters (square map)
            resolution: Grid resolution in meters per cell
        """
        self.resolution = resolution
        self.map_size_m = map_size_m
        self.grid_size = int(map_size_m / resolution)
        
        # Initialize occupancy grid (0.5 = unknown, 0 = free, 1 = occupied)
        self.grid = np.full((self.grid_size, self.grid_size), 0.5)
        
        # Robot pose in world coordinates
        self.robot_x = map_size_m / 2  # Start at center (0.75m from origin)
        self.robot_y = map_size_m / 2
        self.robot_theta = 0.0
        
        # SLAM parameters
        self.occupied_threshold = 0.7
        self.free_threshold = 0.3
        self.hit_probability = 0.8
        self.miss_probability = 0.2
        
        print(f"🗺️  Simple SLAM initialized: {self.grid_size}x{self.grid_size} grid, {resolution}m resolution")
        print(f"📏 Map covers {map_size_m}m x {map_size_m}m real-world area")
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x) / self.resolution)
        grid_y = int((y) / self.resolution)
        
        # Clamp to grid bounds
        grid_x = max(0, min(self.grid_size - 1, grid_x))
        grid_y = max(0, min(self.grid_size - 1, grid_y))
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        x = grid_x * self.resolution
        y = grid_y * self.resolution
        return x, y
    
    def update_map(self, lidar_points: List[Tuple[float, float]]):
        """
        Update occupancy grid with LIDAR scan
        
        Args:
            lidar_points: List of (x, y) points in robot frame
        """
        if not lidar_points:
            return
        
        # Robot position in grid coordinates
        robot_gx, robot_gy = self.world_to_grid(self.robot_x, self.robot_y)
        
        for rel_x, rel_y in lidar_points:
            # Transform point from robot frame to world frame
            cos_theta = math.cos(self.robot_theta)
            sin_theta = math.sin(self.robot_theta)
            
            world_x = self.robot_x + rel_x * cos_theta - rel_y * sin_theta
            world_y = self.robot_y + rel_x * sin_theta + rel_y * cos_theta
            
            # Convert to grid coordinates
            point_gx, point_gy = self.world_to_grid(world_x, world_y)
            
            # Ray tracing from robot to obstacle
            self._update_ray(robot_gx, robot_gy, point_gx, point_gy)
    
    def _update_ray(self, x0: int, y0: int, x1: int, y1: int):
        """Update grid cells along ray using Bresenham's line algorithm"""
        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # Mark cells as free (except the endpoint)
            if (x, y) != (x1, y1) and 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                # Update free space probability
                current_prob = self.grid[y, x]
                self.grid[y, x] = self._update_probability(current_prob, self.miss_probability)
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        # Mark endpoint as occupied
        if 0 <= x1 < self.grid_size and 0 <= y1 < self.grid_size:
            current_prob = self.grid[y1, x1]
            self.grid[y1, x1] = self._update_probability(current_prob, self.hit_probability)
    
    def _update_probability(self, prior: float, measurement: float) -> float:
        """Update probability using log-odds"""
        # Convert to log odds
        prior_odds = prior / (1 - prior + 1e-6)
        measurement_odds = measurement / (1 - measurement + 1e-6)
        
        # Update in log space
        posterior_odds = prior_odds * measurement_odds
        
        # Convert back to probability
        posterior = posterior_odds / (1 + posterior_odds)
        
        # Clamp to reasonable bounds
        return max(0.01, min(0.99, posterior))
    
    def update_robot_pose(self, delta_x: float, delta_y: float, delta_theta: float):
        """Update robot pose based on odometry"""
        self.robot_x += delta_x
        self.robot_y += delta_y
        self.robot_theta += delta_theta
        
        # Normalize angle
        while self.robot_theta > math.pi:
            self.robot_theta -= 2 * math.pi
        while self.robot_theta < -math.pi:
            self.robot_theta += 2 * math.pi
        
        # Keep robot within map bounds
        self.robot_x = max(0, min(self.map_size_m, self.robot_x))
        self.robot_y = max(0, min(self.map_size_m, self.robot_y))
    
    def is_cell_free(self, x: float, y: float) -> bool:
        """Check if a world coordinate is in free space"""
        grid_x, grid_y = self.world_to_grid(x, y)
        return self.grid[grid_y, grid_x] < self.free_threshold
    
    def is_cell_occupied(self, x: float, y: float) -> bool:
        """Check if a world coordinate is occupied"""
        grid_x, grid_y = self.world_to_grid(x, y)
        return self.grid[grid_y, grid_x] > self.occupied_threshold
    
    def find_free_path_direction(self, target_angle: float = 0.0, search_range: float = 1.0) -> Optional[float]:
        """
        Find a free direction to move towards
        
        Args:
            target_angle: Preferred direction (radians)
            search_range: How far to look ahead (meters)
            
        Returns:
            Best direction angle in radians, or None if no free path
        """
        best_angle = None
        best_score = -1
        
        # Search angles around the target
        for angle_offset in np.arange(-math.pi/2, math.pi/2, 0.1):  # ±90° search
            test_angle = target_angle + angle_offset
            
            # Check if path is clear
            clear_distance = self._get_clear_distance(test_angle, search_range)
            
            if clear_distance > 0.3:  # At least 30cm clearance
                # Score based on clearance and proximity to target
                clearance_score = min(clear_distance / search_range, 1.0)
                angle_score = 1.0 - abs(angle_offset) / (math.pi/2)
                total_score = clearance_score * 0.7 + angle_score * 0.3
                
                if total_score > best_score:
                    best_score = total_score
                    best_angle = test_angle
        
        return best_angle
    
    def _get_clear_distance(self, angle: float, max_distance: float) -> float:
        """Get the clear distance in a given direction"""
        step_size = self.resolution
        distance = 0
        
        while distance < max_distance:
            # Calculate test position
            test_x = self.robot_x + distance * math.cos(angle)
            test_y = self.robot_y + distance * math.sin(angle)
            
            # Check if position is occupied or out of bounds
            if (test_x < 0 or test_x >= self.map_size_m or 
                test_y < 0 or test_y >= self.map_size_m or
                self.is_cell_occupied(test_x, test_y)):
                break
            
            distance += step_size
        
        return distance
    
    def get_map_data(self) -> GridMap:
        """Get current map as GridMap object"""
        return GridMap(
            width=self.grid_size,
            height=self.grid_size,
            resolution=self.resolution,
            origin_x=0,
            origin_y=0,
            data=self.grid.copy()
        )
    
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """Get current robot pose"""
        return self.robot_x, self.robot_y, self.robot_theta

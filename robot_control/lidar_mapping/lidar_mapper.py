#!/usr/bin/env python3
"""
LIDAR Mapper Module
===================

This module processes LIDAR scan data to create and update maps.
Converts polar coordinates to Cartesian and manages occupancy grids.
"""

import math
import numpy as np
import time
import logging
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field
from collections import defaultdict

# Handle both relative and absolute imports
try:
    from .ydlidar_x2_optimized import LidarScan, LidarPoint
except ImportError:
    from ydlidar_x2_optimized import LidarScan, LidarPoint


@dataclass
class MapPoint:
    """Represents a point in the occupancy map"""
    x: float
    y: float
    occupancy_probability: float  # 0.0 = free, 1.0 = occupied
    last_updated: float
    hit_count: int = 0
    miss_count: int = 0


@dataclass
class OccupancyGrid:
    """Occupancy grid map representation"""
    width: int          # Grid width in cells
    height: int         # Grid height in cells
    resolution: float   # Meters per cell
    origin_x: float     # World coordinate of grid origin
    origin_y: float     # World coordinate of grid origin
    grid: np.ndarray = field(init=False)
    
    def __post_init__(self):
        """Initialize the grid after creation"""
        self.grid = np.full((self.height, self.width), 0.5, dtype=np.float32)  # Unknown = 0.5


class LidarMapper:
    """
    LIDAR-based mapping system
    
    Processes LIDAR scans to build and maintain an occupancy grid map.
    Handles coordinate transformations and map updates.
    """
    
    def __init__(self, 
                 map_width: float = 10.0,     # Map width in meters
                 map_height: float = 10.0,    # Map height in meters
                 resolution: float = 0.05,    # Resolution in meters/cell
                 max_range: float = 5.0):     # Maximum LIDAR range to consider
        """
        Initialize LIDAR mapper
        
        Args:
            map_width: Width of map area in meters
            map_height: Height of map area in meters  
            resolution: Map resolution in meters per cell
            max_range: Maximum LIDAR range to process
        """
        self.map_width = map_width
        self.map_height = map_height
        self.resolution = resolution
        self.max_range = max_range
        
        # Calculate grid dimensions
        self.grid_width = int(map_width / resolution)
        self.grid_height = int(map_height / resolution)
        
        # Center the map origin
        self.origin_x = -map_width / 2.0
        self.origin_y = -map_height / 2.0
        
        # Create occupancy grid
        self.occupancy_grid = OccupancyGrid(
            width=self.grid_width,
            height=self.grid_height,
            resolution=resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y
        )
        
        # Mapping parameters
        self.hit_probability = 0.7      # Probability increase for occupied cells
        self.miss_probability = 0.3     # Probability decrease for free cells
        self.min_probability = 0.1      # Minimum occupancy probability
        self.max_probability = 0.9      # Maximum occupancy probability
        
        # Robot pose (assumed to be at origin initially)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Statistics
        self.total_scans_processed = 0
        self.last_update_time = 0.0
        
        # Setup logging
        self.logger = logging.getLogger('LidarMapper')
        self.logger.setLevel(logging.INFO)
        
        self.logger.info(f"🗺️  LIDAR Mapper initialized:")
        self.logger.info(f"   Map size: {map_width}x{map_height}m")
        self.logger.info(f"   Grid size: {self.grid_width}x{self.grid_height} cells")
        self.logger.info(f"   Resolution: {resolution}m/cell")
        self.logger.info(f"   Max range: {max_range}m")
    
    def update_robot_pose(self, x: float, y: float, theta: float):
        """
        Update robot pose for coordinate transformations
        
        Args:
            x: Robot X position in meters
            y: Robot Y position in meters
            theta: Robot orientation in radians
        """
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
    
    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates
        
        Args:
            world_x: X coordinate in meters
            world_y: Y coordinate in meters
            
        Returns:
            Tuple of (grid_x, grid_y) coordinates
        """
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Convert grid coordinates to world coordinates
        
        Args:
            grid_x: Grid X coordinate
            grid_y: Grid Y coordinate
            
        Returns:
            Tuple of (world_x, world_y) coordinates
        """
        world_x = self.origin_x + (grid_x + 0.5) * self.resolution
        world_y = self.origin_y + (grid_y + 0.5) * self.resolution
        return world_x, world_y
    
    def is_valid_grid_point(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid coordinates are within bounds"""
        return (0 <= grid_x < self.grid_width and 
                0 <= grid_y < self.grid_height)
    
    def polar_to_cartesian(self, distance: float, angle_deg: float) -> Tuple[float, float]:
        """
        Convert polar coordinates (from LIDAR) to Cartesian coordinates
        
        Args:
            distance: Distance in meters
            angle_deg: Angle in degrees (0-360)
            
        Returns:
            Tuple of (x, y) coordinates relative to robot
        """
        angle_rad = math.radians(angle_deg)
        
        # Convert to robot-relative coordinates
        x_rel = distance * math.cos(angle_rad)
        y_rel = distance * math.sin(angle_rad)
        
        # Transform to world coordinates considering robot pose
        cos_theta = math.cos(self.robot_theta)
        sin_theta = math.sin(self.robot_theta)
        
        x_world = self.robot_x + (x_rel * cos_theta - y_rel * sin_theta)
        y_world = self.robot_y + (x_rel * sin_theta + y_rel * cos_theta)
        
        return x_world, y_world
    
    def bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """
        Bresenham's line algorithm for ray tracing
        
        Returns list of grid points along the line from (x0,y0) to (x1,y1)
        """
        points = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        x_step = 1 if x0 < x1 else -1
        y_step = 1 if y0 < y1 else -1
        
        error = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            error2 = 2 * error
            
            if error2 > -dy:
                error -= dy
                x += x_step
            
            if error2 < dx:
                error += dx
                y += y_step
        
        return points
    
    def update_map(self, scan: LidarScan):
        """
        Update occupancy grid with new LIDAR scan
        
        Args:
            scan: LidarScan object containing scan data
        """
        if not scan or not scan.is_complete:
            return
        
        start_time = time.time()
        
        # Robot position in grid coordinates
        robot_grid_x, robot_grid_y = self.world_to_grid(self.robot_x, self.robot_y)
        
        if not self.is_valid_grid_point(robot_grid_x, robot_grid_y):
            self.logger.warning("⚠️  Robot position outside map bounds")
            return
        
        points_processed = 0
        
        for point in scan.points:
            # Skip invalid or out-of-range points
            if (point.distance <= 0.1 or 
                point.distance > self.max_range or
                point.quality < 50):  # Quality threshold
                continue
            
            # Convert to world coordinates
            world_x, world_y = self.polar_to_cartesian(point.distance, point.angle)
            
            # Convert to grid coordinates
            end_grid_x, end_grid_y = self.world_to_grid(world_x, world_y)
            
            if not self.is_valid_grid_point(end_grid_x, end_grid_y):
                continue
            
            # Trace ray from robot to detected point
            ray_points = self.bresenham_line(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y)
            
            # Update free space along the ray (except the last point)
            for i, (gx, gy) in enumerate(ray_points[:-1]):
                if self.is_valid_grid_point(gx, gy):
                    current_prob = self.occupancy_grid.grid[gy, gx]
                    # Decrease probability for free space
                    new_prob = max(self.min_probability, 
                                 current_prob * self.miss_probability)
                    self.occupancy_grid.grid[gy, gx] = new_prob
            
            # Update occupied space at the endpoint
            if len(ray_points) > 0:
                end_gx, end_gy = ray_points[-1]
                if self.is_valid_grid_point(end_gx, end_gy):
                    current_prob = self.occupancy_grid.grid[end_gy, end_gx]
                    # Increase probability for occupied space
                    new_prob = min(self.max_probability,
                                 current_prob + (1.0 - current_prob) * self.hit_probability)
                    self.occupancy_grid.grid[end_gy, end_gx] = new_prob
            
            points_processed += 1
        
        # Update statistics
        self.total_scans_processed += 1
        self.last_update_time = time.time()
        processing_time = self.last_update_time - start_time
        
        self.logger.debug(f"📡 Processed scan: {points_processed} points in {processing_time:.3f}s")
    
    def get_occupancy_grid(self) -> OccupancyGrid:
        """Get the current occupancy grid"""
        return self.occupancy_grid
    
    def get_map_data_for_display(self) -> Dict[str, Any]:
        """
        Get map data formatted for display
        
        Returns:
            Dictionary containing map data and metadata
        """
        return {
            'grid': self.occupancy_grid.grid.copy(),
            'width': self.grid_width,
            'height': self.grid_height,
            'resolution': self.resolution,
            'origin_x': self.origin_x,
            'origin_y': self.origin_y,
            'robot_x': self.robot_x,
            'robot_y': self.robot_y,
            'robot_theta': self.robot_theta,
            'total_scans': self.total_scans_processed,
            'last_update': self.last_update_time
        }
    
    def get_cartesian_points(self, scan: LidarScan) -> List[Tuple[float, float]]:
        """
        Convert LIDAR scan to Cartesian points for visualization
        
        Args:
            scan: LidarScan object
            
        Returns:
            List of (x, y) points in world coordinates
        """
        points = []
        
        if not scan or not scan.points:
            return points
        
        for point in scan.points:
            if (point.distance > 0.1 and 
                point.distance <= self.max_range and
                point.quality >= 50):
                
                x, y = self.polar_to_cartesian(point.distance, point.angle)
                points.append((x, y))
        
        return points
    
    def clear_map(self):
        """Clear the occupancy grid (reset to unknown)"""
        self.occupancy_grid.grid.fill(0.5)  # Unknown state
        self.total_scans_processed = 0
        self.logger.info("🗺️  Map cleared")
    
    def save_map(self, filename: str):
        """Save occupancy grid to file"""
        try:
            np.save(filename, self.occupancy_grid.grid)
            self.logger.info(f"💾 Map saved to {filename}")
        except Exception as e:
            self.logger.error(f"❌ Failed to save map: {e}")
    
    def load_map(self, filename: str):
        """Load occupancy grid from file"""
        try:
            loaded_grid = np.load(filename)
            if loaded_grid.shape == (self.grid_height, self.grid_width):
                self.occupancy_grid.grid = loaded_grid
                self.logger.info(f"📂 Map loaded from {filename}")
            else:
                self.logger.error(f"❌ Map size mismatch: {loaded_grid.shape}")
        except Exception as e:
            self.logger.error(f"❌ Failed to load map: {e}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get mapping statistics"""
        return {
            'total_scans_processed': self.total_scans_processed,
            'last_update_time': self.last_update_time,
            'map_width': self.map_width,
            'map_height': self.map_height,
            'resolution': self.resolution,
            'grid_size': (self.grid_width, self.grid_height),
            'robot_pose': (self.robot_x, self.robot_y, self.robot_theta)
        }
    
    def get_cartesian_points(self, scan: LidarScan) -> List[Tuple[float, float]]:
        """
        Convert LIDAR scan points from polar to Cartesian coordinates
        
        Args:
            scan: LidarScan object containing polar coordinate points
            
        Returns:
            List of (x, y) tuples in Cartesian coordinates
        """
        cartesian_points = []
        
        for point in scan.points:
            # Convert from polar to Cartesian coordinates
            # point.angle is in radians, point.distance is in meters
            x = point.distance * math.cos(point.angle)
            y = point.distance * math.sin(point.angle)
            
            # Transform to robot's coordinate system (if needed)
            # For now, assume LIDAR is at robot center
            world_x = self.robot_x + x
            world_y = self.robot_y + y
            
            cartesian_points.append((world_x, world_y))
        
        return cartesian_points


# Example usage
if __name__ == "__main__":
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    # Create mapper
    mapper = LidarMapper(map_width=8.0, map_height=8.0, resolution=0.05)
    
    # Simulate some LIDAR data processing
    print("🗺️  LIDAR Mapper test")
    print(f"Grid size: {mapper.grid_width}x{mapper.grid_height}")
    print(f"Resolution: {mapper.resolution}m/cell")
    
    # Test coordinate conversion
    test_points = [(1.0, 1.0), (-1.0, -1.0), (0.0, 2.0)]
    for wx, wy in test_points:
        gx, gy = mapper.world_to_grid(wx, wy)
        wx2, wy2 = mapper.grid_to_world(gx, gy)
        print(f"World ({wx}, {wy}) -> Grid ({gx}, {gy}) -> World ({wx2:.2f}, {wy2:.2f})")

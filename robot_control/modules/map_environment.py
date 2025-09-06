#!/usr/bin/env python3
"""
Map Environment Module
Handles road boundaries, obstacles, and map structure for pathfinding
"""

import pygame
import math
from typing import List, Tuple, Set

class MapEnvironment:
    """Manages the map environment with roads, boundaries, and obstacles"""
    
    def __init__(self, width: int, height: int, scale: float = 100.0):
        self.width = width
        self.height = height
        self.scale = scale  # pixels per meter
        
        # Colors - IMPROVED VISIBILITY
        self.ROAD_COLOR = (120, 120, 120)     # Lighter gray roads (more visible)
        self.BOUNDARY_COLOR = (139, 69, 19)   # Brown boundaries
        self.OBSTACLE_COLOR = (255, 0, 0)     # Red obstacles
        self.GRASS_COLOR = (0, 100, 0)        # Darker green grass (better contrast)
        self.PATH_COLOR = (255, 255, 0)       # Yellow path
        self.WAYPOINT_COLOR = (0, 255, 255)   # Cyan waypoints
        
        # Map elements
        self.roads = []
        self.boundaries = []
        self.obstacles = []
        self.grid_size = 0.2  # 20cm grid resolution (larger for better pathfinding)
        
        self._create_sample_map()
    
    def _create_sample_map(self):
        """Create a sample road network centered around (0,0)"""
        
        # Calculate world size (visible area in meters)
        world_width = self.width / self.scale    # e.g., 1920/150 = 12.8m
        world_height = self.height / self.scale  # e.g., 1080/150 = 7.2m
        
        # Create roads centered around (0,0)
        # Main horizontal road through center (y=0)
        self.roads.append({
            'type': 'horizontal',
            'y': 0.0,  # CENTER LINE (y=0)
            'x_start': -world_width/2 + 1.0,  # Start 1m from left edge
            'x_end': world_width/2 - 1.0,     # End 1m from right edge  
            'width': 1.0  # 100cm wide main road
        })
        
        # Main vertical road through center (x=0)
        self.roads.append({
            'type': 'vertical', 
            'x': 0.0,  # CENTER LINE (x=0)
            'y_start': -world_height/2 + 1.0,  # Start 1m from bottom
            'y_end': world_height/2 - 1.0,     # End 1m from top
            'width': 1.0  # 100cm wide main road
        })
        
        # Secondary horizontal roads
        self.roads.append({
            'type': 'horizontal',
            'y': 2.0,  # 2m above center
            'x_start': -world_width/3,
            'x_end': world_width/3,
            'width': 0.6  # 60cm wide road
        })
        
        self.roads.append({
            'type': 'horizontal',
            'y': -2.0,  # 2m below center
            'x_start': -world_width/3,
            'x_end': world_width/3,
            'width': 0.6  # 60cm wide road
        })
        
        # Secondary vertical roads
        self.roads.append({
            'type': 'vertical',
            'x': 3.0,  # 3m right of center
            'y_start': -world_height/3,
            'y_end': world_height/3,
            'width': 0.6  # 60cm wide road
        })
        
        self.roads.append({
            'type': 'vertical',
            'x': -3.0,  # 3m left of center
            'y_start': -world_height/3,
            'y_end': world_height/3,
            'width': 0.6  # 60cm wide road
        })
        
        # Create boundaries along roads
        for road in self.roads:
            self._create_road_boundaries(road)
        
        # Add some obstacles (positioned in world coordinates)
        self.obstacles.extend([
            {'x': 1.5, 'y': 1.0, 'radius': 0.3},   # Right side, upper
            {'x': -2.0, 'y': -1.5, 'radius': 0.25}, # Left side, lower
            {'x': 0.5, 'y': -3.0, 'radius': 0.2},   # Center-right, bottom
            {'x': -1.0, 'y': 2.5, 'radius': 0.3},   # Left side, top
        ])
    
    def _create_road_boundaries(self, road):
        """Create boundary walls along a road"""
        half_width = road['width'] / 2
        
        if road['type'] == 'horizontal':
            # Top boundary
            self.boundaries.append({
                'start': (road['x_start'], road['y'] - half_width),
                'end': (road['x_end'], road['y'] - half_width)
            })
            # Bottom boundary
            self.boundaries.append({
                'start': (road['x_start'], road['y'] + half_width),
                'end': (road['x_end'], road['y'] + half_width)
            })
        
        elif road['type'] == 'vertical':
            # Left boundary
            self.boundaries.append({
                'start': (road['x'] - half_width, road['y_start']),
                'end': (road['x'] - half_width, road['y_end'])
            })
            # Right boundary
            self.boundaries.append({
                'start': (road['x'] + half_width, road['y_start']),
                'end': (road['x'] + half_width, road['y_end'])
            })
    
    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (meters) to screen coordinates (pixels)"""
        # Center (0,0) should be at screen center
        screen_x = int(self.width // 2 + x * self.scale)
        screen_y = int(self.height // 2 - y * self.scale)  # Y axis flipped for screen
        return screen_x, screen_y
    
    def screen_to_world(self, screen_x: int, screen_y: int) -> Tuple[float, float]:
        """Convert screen coordinates (pixels) to world coordinates (meters)"""
        # Screen center should be (0,0) in world coordinates
        world_x = (screen_x - self.width // 2) / self.scale
        world_y = (self.height // 2 - screen_y) / self.scale
        return world_x, world_y
    
    def is_point_on_road(self, x: float, y: float) -> bool:
        """Check if a point is on a valid road"""
        for road in self.roads:
            if self._point_in_road(x, y, road):
                return True
        return False
    
    def _point_in_road(self, x: float, y: float, road) -> bool:
        """Check if point is within a specific road"""
        half_width = road['width'] / 2
        
        if road['type'] == 'horizontal':
            return (road['x_start'] <= x <= road['x_end'] and 
                    road['y'] - half_width <= y <= road['y'] + half_width)
        
        elif road['type'] == 'vertical':
            return (road['y_start'] <= y <= road['y_end'] and
                    road['x'] - half_width <= x <= road['x'] + half_width)
        
        return False
    
    def is_point_blocked(self, x: float, y: float) -> bool:
        """Check if a point is blocked by obstacles or boundaries"""
        # Check obstacles
        for obstacle in self.obstacles:
            dist = math.sqrt((x - obstacle['x'])**2 + (y - obstacle['y'])**2)
            if dist < obstacle['radius']:
                return True
        
        # Check if point is on road (not blocked by boundaries)
        return not self.is_point_on_road(x, y)
    
    def get_neighbors(self, x: float, y: float) -> List[Tuple[float, float]]:
        """Get valid neighboring points for pathfinding"""
        neighbors = []
        
        # 8-directional movement
        directions = [
            (-self.grid_size, 0), (self.grid_size, 0),  # Left, Right
            (0, -self.grid_size), (0, self.grid_size),  # Down, Up
            (-self.grid_size, -self.grid_size), (self.grid_size, -self.grid_size),  # Diagonals
            (-self.grid_size, self.grid_size), (self.grid_size, self.grid_size)
        ]
        
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            
            # Check bounds (world coordinates can be negative)
            world_width = self.width / self.scale
            world_height = self.height / self.scale
            
            if (-world_width/2 <= new_x <= world_width/2 and 
                -world_height/2 <= new_y <= world_height/2 and
                not self.is_point_blocked(new_x, new_y)):
                neighbors.append((new_x, new_y))
        
        return neighbors
    
    def draw(self, screen):
        """Draw the map environment"""
        # Fill background with grass
        screen.fill(self.GRASS_COLOR)
        
        # Draw roads
        for road in self.roads:
            self._draw_road(screen, road)
        
        # Draw boundaries
        for boundary in self.boundaries:
            start_screen = self.world_to_screen(*boundary['start'])
            end_screen = self.world_to_screen(*boundary['end'])
            pygame.draw.line(screen, self.BOUNDARY_COLOR, start_screen, end_screen, 5)
        
        # Draw obstacles
        for obstacle in self.obstacles:
            center_screen = self.world_to_screen(obstacle['x'], obstacle['y'])
            radius_pixels = int(obstacle['radius'] * self.scale)
            pygame.draw.circle(screen, self.OBSTACLE_COLOR, center_screen, radius_pixels)
    
    def _draw_road(self, screen, road):
        """Draw a single road with enhanced visibility"""
        if road['type'] == 'horizontal':
            start_screen = self.world_to_screen(road['x_start'], road['y'])
            end_screen = self.world_to_screen(road['x_end'], road['y'])
            width_pixels = int(road['width'] * self.scale)
            
            # Draw road as thick line with border
            pygame.draw.line(screen, (80, 80, 80), start_screen, end_screen, width_pixels + 4)  # Dark border
            pygame.draw.line(screen, self.ROAD_COLOR, start_screen, end_screen, width_pixels)  # Main road
            
        elif road['type'] == 'vertical':
            start_screen = self.world_to_screen(road['x'], road['y_start'])
            end_screen = self.world_to_screen(road['x'], road['y_end'])
            width_pixels = int(road['width'] * self.scale)
            
            # Draw road as thick line with border
            pygame.draw.line(screen, (80, 80, 80), start_screen, end_screen, width_pixels + 4)  # Dark border
            pygame.draw.line(screen, self.ROAD_COLOR, start_screen, end_screen, width_pixels)  # Main road
    
    def draw_path(self, screen, path: List[Tuple[float, float]]):
        """Draw a path on the map"""
        if len(path) < 2:
            return
        
        screen_points = []
        for x, y in path:
            screen_x, screen_y = self.world_to_screen(x, y)
            screen_points.append((screen_x, screen_y))
        
        # Draw path line
        if len(screen_points) > 1:
            pygame.draw.lines(screen, self.PATH_COLOR, False, screen_points, 4)
        
        # Draw waypoints
        for point in screen_points:
            pygame.draw.circle(screen, self.WAYPOINT_COLOR, point, 6)
    
    def draw_target(self, screen, target_x: float, target_y: float):
        """Draw the target location"""
        target_screen = self.world_to_screen(target_x, target_y)
        pygame.draw.circle(screen, (255, 0, 255), target_screen, 12)  # Magenta target
        pygame.draw.circle(screen, (255, 255, 255), target_screen, 8)  # White center

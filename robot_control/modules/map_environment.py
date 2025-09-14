#!/usr/bin/env python3
"""
Enhanced Map Environment for Robot Navigation
Provides a visually appealing map with roads, junctions, obstacles, and boundaries.
"""

import pygame
import math
import time
from typing import List, Tuple, Set, Optional

class MapEnvironment:
    """Manages the map environment with roads, boundaries, and obstacles"""
    
    def __init__(self, width: int, height: int, scale: float = 100.0, workspace_size: float = 3.0):
        self.width = width
        self.height = height
        self.scale = scale  # pixels per meter
        self.workspace_size = workspace_size  # workspace size in meters
        
        # LIDAR overlay settings
        self.show_lidar_overlay = False
        self.lidar_points = []  # Store LIDAR scan points
        self.obstacle_detected = False  # Obstacle detection status
        self.robot_x = 0.0  # Robot position for safety zones
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Colors - IMPROVED VISIBILITY AND APPEAL
        self.ROAD_COLOR = (80, 80, 80)        # Darker asphalt gray
        self.ROAD_STRIPE_COLOR = (255, 255, 255)  # White road stripes
        self.BOUNDARY_COLOR = (139, 69, 19)   # Brown boundaries
        self.OBSTACLE_COLOR = (220, 20, 20)   # Darker red obstacles
        self.GRASS_COLOR = (34, 139, 34)      # Forest green grass
        self.PATH_COLOR = (255, 215, 0)       # Gold path
        self.WAYPOINT_COLOR = (0, 191, 255)   # Deep sky blue waypoints
        self.JUNCTION_COLOR = (60, 60, 60)    # Darker junction color
        self.SIDEWALK_COLOR = (169, 169, 169) # Light gray sidewalks
        
        # Map elements
        self.roads = []
        self.boundaries = []
        self.obstacles = []
        self.grid_size = 0.1  # 10cm grid resolution for small workspace
        
        self._create_sample_map()
    
    def _create_sample_map(self):
        """Create a complex road network for 3m x 3m workspace"""
        
        # Use workspace size for bigger map with minimal margins
        half_workspace = self.workspace_size / 2  # 1.5m from center
        margin = 0.05  # Minimal 5cm margin
        
        # Main horizontal road through center (y=0) - full width
        self.roads.append({
            'type': 'horizontal',
            'y': 0.0,  # CENTER LINE (y=0)
            'x_start': -half_workspace + margin,  # Start 5cm from edge
            'x_end': half_workspace - margin,     # End 5cm from edge  
            'width': 0.5  # 50cm wide main road
        })
        
        # Main vertical road through center (x=0) - full height
        self.roads.append({
            'type': 'vertical', 
            'x': 0.0,  # CENTER LINE (x=0)
            'y_start': -half_workspace + margin,  # Start 5cm from edge
            'y_end': half_workspace - margin,     # End 5cm from edge
            'width': 0.5  # 50cm wide main road
        })
        
        # Secondary horizontal roads - full width
        self.roads.append({
            'type': 'horizontal',
            'y': 0.9,  # 90cm above center
            'x_start': -half_workspace + margin,
            'x_end': half_workspace - margin,
            'width': 0.4  # 40cm wide road
        })
        
        self.roads.append({
            'type': 'horizontal',
            'y': -0.9,  # 90cm below center
            'x_start': -half_workspace + margin,
            'x_end': half_workspace - margin,
            'width': 0.4  # 40cm wide road
        })
        
        # Secondary vertical roads - full height
        self.roads.append({
            'type': 'vertical',
            'x': 0.9,  # 90cm right of center
            'y_start': -half_workspace + margin,
            'y_end': half_workspace - margin,
            'width': 0.4  # 40cm wide road
        })
        
        self.roads.append({
            'type': 'vertical',
            'x': -0.9,  # 90cm left of center
            'y_start': -half_workspace + margin,
            'y_end': half_workspace - margin,
            'width': 0.4  # 40cm wide road
        })
        
        # Create boundaries along roads (avoiding intersections)
        self._create_smart_boundaries()
        
        # No static obstacles - will use LIDAR for real-time obstacle detection
    
    def _create_smart_boundaries(self):
        """Create road boundaries that avoid intersections"""
        # First, find all intersection points
        intersections = self._find_intersections()
        
        # Create boundaries for each road, avoiding intersections
        for road in self.roads:
            self._create_road_boundaries_with_gaps(road, intersections)
    
    def _find_intersections(self) -> List[Tuple[float, float]]:
        """Find all road intersection points"""
        intersections = []
        
        for i, road1 in enumerate(self.roads):
            for j, road2 in enumerate(self.roads):
                if i >= j:  # Avoid duplicate checks
                    continue
                
                intersection = self._find_road_intersection(road1, road2)
                if intersection:
                    intersections.append(intersection)
        
        return intersections
    
    def _find_road_intersection(self, road1, road2) -> Optional[Tuple[float, float]]:
        """Find intersection point between two roads"""
        if road1['type'] == road2['type']:
            return None  # Parallel roads don't intersect
        
        if road1['type'] == 'horizontal' and road2['type'] == 'vertical':
            # Check if they actually intersect
            if (road1['x_start'] <= road2['x'] <= road1['x_end'] and
                road2['y_start'] <= road1['y'] <= road2['y_end']):
                return (road2['x'], road1['y'])
                
        elif road1['type'] == 'vertical' and road2['type'] == 'horizontal':
            # Check if they actually intersect
            if (road2['x_start'] <= road1['x'] <= road2['x_end'] and
                road1['y_start'] <= road2['y'] <= road1['y_end']):
                return (road1['x'], road2['y'])
        
        return None
    
    def _create_road_boundaries_with_gaps(self, road, intersections: List[Tuple[float, float]]):
        """Create boundaries for a road with gaps at intersections"""
        half_width = road['width'] / 2
        gap_size = 1.0  # 1 meter gap at intersections
        
        if road['type'] == 'horizontal':
            # Find intersections on this road
            road_intersections = []
            for ix, iy in intersections:
                if (road['x_start'] <= ix <= road['x_end'] and 
                    abs(iy - road['y']) < 0.1):  # On this horizontal road
                    road_intersections.append(ix)
            
            road_intersections.sort()  # Sort by x coordinate
            
            # Create boundary segments between intersections
            current_x = road['x_start']
            
            for intersection_x in road_intersections:
                # Create boundary before intersection
                if current_x < intersection_x - gap_size/2:
                    # Top boundary
                    self.boundaries.append({
                        'start': (current_x, road['y'] + half_width),
                        'end': (intersection_x - gap_size/2, road['y'] + half_width)
                    })
                    # Bottom boundary
                    self.boundaries.append({
                        'start': (current_x, road['y'] - half_width),
                        'end': (intersection_x - gap_size/2, road['y'] - half_width)
                    })
                
                current_x = intersection_x + gap_size/2
            
            # Final segment after last intersection
            if current_x < road['x_end']:
                # Top boundary
                self.boundaries.append({
                    'start': (current_x, road['y'] + half_width),
                    'end': (road['x_end'], road['y'] + half_width)
                })
                # Bottom boundary
                self.boundaries.append({
                    'start': (current_x, road['y'] - half_width),
                    'end': (road['x_end'], road['y'] - half_width)
                })
        
        elif road['type'] == 'vertical':
            # Similar logic for vertical roads
            road_intersections = []
            for ix, iy in intersections:
                if (road['y_start'] <= iy <= road['y_end'] and 
                    abs(ix - road['x']) < 0.1):  # On this vertical road
                    road_intersections.append(iy)
            
            road_intersections.sort()  # Sort by y coordinate
            
            # Create boundary segments between intersections
            current_y = road['y_start']
            
            for intersection_y in road_intersections:
                # Create boundary before intersection
                if current_y < intersection_y - gap_size/2:
                    # Left boundary
                    self.boundaries.append({
                        'start': (road['x'] - half_width, current_y),
                        'end': (road['x'] - half_width, intersection_y - gap_size/2)
                    })
                    # Right boundary
                    self.boundaries.append({
                        'start': (road['x'] + half_width, current_y),
                        'end': (road['x'] + half_width, intersection_y - gap_size/2)
                    })
                
                current_y = intersection_y + gap_size/2
            
            # Final segment after last intersection
            if current_y < road['y_end']:
                # Left boundary
                self.boundaries.append({
                    'start': (road['x'] - half_width, current_y),
                    'end': (road['x'] - half_width, road['y_end'])
                })
                # Right boundary
                self.boundaries.append({
                    'start': (road['x'] + half_width, current_y),
                    'end': (road['x'] + half_width, road['y_end'])
                })
    
    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates"""
        screen_x = int(x * self.scale + self.width // 2)
        screen_y = int(-y * self.scale + self.height // 2)  # Flip Y axis
        return (screen_x, screen_y)
    
    def screen_to_world(self, screen_x: int, screen_y: int) -> Tuple[float, float]:
        """Convert screen coordinates to world coordinates"""
        world_x = (screen_x - self.width // 2) / self.scale
        world_y = -(screen_y - self.height // 2) / self.scale  # Flip Y axis
        return (world_x, world_y)
    
    def is_point_on_road(self, x: float, y: float) -> bool:
        """Check if a point is on any road"""
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
        """Get valid neighboring points for pathfinding with angular constraints"""
        neighbors = []
        
        # ANGULAR-CONSTRAINED MOVEMENT: Only 90°, 45°, 180°, and 360° turns
        # Use larger steps to keep robot in middle of roads and reduce jitter
        step_size = self.grid_size * 2  # Double grid size for smoother paths
        
        # Define allowed movement directions with their angles
        directions = [
            # 90-degree movements (cardinal directions)
            (step_size, 0),      # 0° - Right (East)
            (0, step_size),      # 90° - Up (North) 
            (-step_size, 0),     # 180° - Left (West)
            (0, -step_size),     # 270° - Down (South)
            
            # 45-degree movements (diagonal directions)  
            (step_size, step_size),     # 45° - Northeast
            (-step_size, step_size),    # 135° - Northwest
            (-step_size, -step_size),   # 225° - Southwest
            (step_size, -step_size),    # 315° - Southeast
        ]
        
        for dx, dy in directions:
            new_x = x + dx
            new_y = y + dy
            
            # Check if new position is valid
            if not self.is_point_blocked(new_x, new_y):
                
                # MIDDLE-OF-ROAD CONSTRAINT: Only allow points on road centerlines
                if self._is_point_on_road_center(new_x, new_y):
                    neighbors.append((new_x, new_y))
        
        return neighbors
    
    def _is_point_on_road_center(self, x: float, y: float) -> bool:
        """Check if point is near the center of a road (not at edges)"""
        for road in self.roads:
            if self._point_near_road_center(x, y, road):
                return True
        return False
    
    def _point_near_road_center(self, x: float, y: float, road) -> bool:
        """Check if point is near the center of a specific road"""
        # Tolerance for being "near center" (25% of road width from centerline)
        center_tolerance = road['width'] * 0.25
        
        if road['type'] == 'horizontal':
            # Check if within road bounds
            if not (road['x_start'] <= x <= road['x_end']):
                return False
            # Check if near horizontal centerline
            return abs(y - road['y']) <= center_tolerance
            
        elif road['type'] == 'vertical':
            # Check if within road bounds  
            if not (road['y_start'] <= y <= road['y_end']):
                return False
            # Check if near vertical centerline
            return abs(x - road['x']) <= center_tolerance
            
        return False
    
    def draw(self, screen):
        """Draw the enhanced map environment"""
        # Fill background with grass
        screen.fill(self.GRASS_COLOR)
        
        # Draw roads with junctions
        self._draw_roads_with_junctions(screen)
        
        # Draw road markings (center lines)
        self._draw_road_markings(screen)
        
        # Draw boundaries with sidewalks
        self._draw_boundaries_enhanced(screen)
        
        # Draw obstacles with shadows (only if any exist)
        if self.obstacles:
            self._draw_obstacles_enhanced(screen)
        
        # Draw LIDAR overlay if enabled
        if self.show_lidar_overlay:
            self._draw_lidar_overlay(screen)
    
    def _draw_roads_with_junctions(self, screen):
        """Draw roads with proper junction handling"""
        # First find all intersections
        intersections = self._find_intersections()
        
        # Draw roads
        for road in self.roads:
            self._draw_single_road_enhanced(screen, road)
        
        # Draw junction overlays to clean up intersections
        for intersection in intersections:
            self._draw_junction(screen, intersection)
    
    def _draw_single_road_enhanced(self, screen, road):
        """Draw a single road with enhanced visuals"""
        if road['type'] == 'horizontal':
            start_screen = self.world_to_screen(road['x_start'], road['y'])
            end_screen = self.world_to_screen(road['x_end'], road['y'])
            width_pixels = int(road['width'] * self.scale)
            
            # Draw road with borders
            pygame.draw.line(screen, (40, 40, 40), start_screen, end_screen, width_pixels + 4)  # Dark border
            pygame.draw.line(screen, self.ROAD_COLOR, start_screen, end_screen, width_pixels)  # Main road
            
        elif road['type'] == 'vertical':
            start_screen = self.world_to_screen(road['x'], road['y_start'])
            end_screen = self.world_to_screen(road['x'], road['y_end'])
            width_pixels = int(road['width'] * self.scale)
            
            # Draw road with borders
            pygame.draw.line(screen, (40, 40, 40), start_screen, end_screen, width_pixels + 4)  # Dark border
            pygame.draw.line(screen, self.ROAD_COLOR, start_screen, end_screen, width_pixels)  # Main road
    
    def _draw_junction(self, screen, intersection):
        """Draw a proper junction at intersection point"""
        junction_screen = self.world_to_screen(intersection[0], intersection[1])
        
        # Determine junction size based on intersecting roads
        max_width = 0
        for road in self.roads:
            if road['type'] == 'horizontal' and abs(road['y'] - intersection[1]) < 0.01:
                max_width = max(max_width, road['width'])
            elif road['type'] == 'vertical' and abs(road['x'] - intersection[0]) < 0.01:
                max_width = max(max_width, road['width'])
        
        junction_size = int(max_width * self.scale * 0.8)
        
        # Draw junction square
        junction_rect = pygame.Rect(
            junction_screen[0] - junction_size//2,
            junction_screen[1] - junction_size//2,
            junction_size,
            junction_size
        )
        pygame.draw.rect(screen, self.JUNCTION_COLOR, junction_rect)
        
        # Add junction corner markings
        corner_size = junction_size // 8
        for dx, dy in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
            corner_x = junction_screen[0] + dx * junction_size//4
            corner_y = junction_screen[1] + dy * junction_size//4
            pygame.draw.circle(screen, self.ROAD_STRIPE_COLOR, (corner_x, corner_y), corner_size, 1)
    
    def _draw_road_markings(self, screen):
        """Draw road center lines and lane markings"""
        for road in self.roads:
            if road['type'] == 'horizontal':
                start_screen = self.world_to_screen(road['x_start'], road['y'])
                end_screen = self.world_to_screen(road['x_end'], road['y'])
                
                # Dashed center line
                dash_length = 8
                gap_length = 6
                current_x = start_screen[0]
                while current_x < end_screen[0]:
                    dash_end = min(current_x + dash_length, end_screen[0])
                    pygame.draw.line(screen, self.ROAD_STRIPE_COLOR, 
                                   (current_x, start_screen[1]), (dash_end, start_screen[1]), 2)
                    current_x = dash_end + gap_length
                    
            elif road['type'] == 'vertical':
                start_screen = self.world_to_screen(road['x'], road['y_start'])
                end_screen = self.world_to_screen(road['x'], road['y_end'])
                
                # Dashed center line
                dash_length = 8
                gap_length = 6
                current_y = start_screen[1]
                while current_y < end_screen[1]:
                    dash_end = min(current_y + dash_length, end_screen[1])
                    pygame.draw.line(screen, self.ROAD_STRIPE_COLOR, 
                                   (start_screen[0], current_y), (start_screen[0], dash_end), 2)
                    current_y = dash_end + gap_length
    
    def _draw_boundaries_enhanced(self, screen):
        """Draw road boundaries with sidewalks"""
        for boundary in self.boundaries:
            start_screen = self.world_to_screen(*boundary['start'])
            end_screen = self.world_to_screen(*boundary['end'])
            
            # Draw sidewalk (wider, lighter line)
            pygame.draw.line(screen, self.SIDEWALK_COLOR, start_screen, end_screen, 6)
            # Draw curb (boundary)
            pygame.draw.line(screen, self.BOUNDARY_COLOR, start_screen, end_screen, 2)
    
    def _draw_obstacles_enhanced(self, screen):
        """Draw obstacles with shadows for better visual appeal"""
        for obstacle in self.obstacles:
            center_screen = self.world_to_screen(obstacle['x'], obstacle['y'])
            radius_pixels = int(obstacle['radius'] * self.scale)
            
            if radius_pixels > 0:
                # Draw shadow (offset)
                shadow_offset = 2
                pygame.draw.circle(screen, (50, 50, 50), 
                                 (center_screen[0] + shadow_offset, center_screen[1] + shadow_offset), 
                                 radius_pixels)
                
                # Draw main obstacle with gradient effect
                pygame.draw.circle(screen, self.OBSTACLE_COLOR, center_screen, radius_pixels)
                pygame.draw.circle(screen, (255, 100, 100), 
                                 (center_screen[0] - radius_pixels//3, center_screen[1] - radius_pixels//3), 
                                 radius_pixels//3)
                pygame.draw.circle(screen, (150, 0, 0), center_screen, radius_pixels, 2)
    
    def draw_path(self, screen, path: List[Tuple[float, float]]):
        """Draw an enhanced path on the map"""
        if len(path) < 2:
            return
        
        screen_points = []
        for x, y in path:
            screen_x, screen_y = self.world_to_screen(x, y)
            screen_points.append((screen_x, screen_y))
        
        if len(screen_points) > 1:
            # Draw path shadow
            shadow_points = [(x+1, y+1) for x, y in screen_points]
            pygame.draw.lines(screen, (100, 100, 0), False, shadow_points, 5)
            # Draw main path
            pygame.draw.lines(screen, self.PATH_COLOR, False, screen_points, 3)
            # Draw path highlights
            pygame.draw.lines(screen, (255, 255, 150), False, screen_points, 1)
        
        # Draw enhanced waypoints
        waypoint_size = max(3, int(min(self.width, self.height) / 80))
        for point in screen_points:
            pygame.draw.circle(screen, (50, 50, 50), (point[0]+1, point[1]+1), waypoint_size+1)  # Shadow
            pygame.draw.circle(screen, self.WAYPOINT_COLOR, point, waypoint_size)
    
    def draw_target(self, screen, target_x: float, target_y: float):
        """Draw an enhanced target marker with pulsing effect"""
        target_screen = self.world_to_screen(target_x, target_y)
        
        # Animated pulsing effect
        pulse = int(abs(math.sin(time.time() * 3)) * 3)
        
        target_size = max(6, int(min(self.width, self.height) / 50))
        
        # Draw target with pulse effect and shadow
        pygame.draw.circle(screen, (50, 50, 50), 
                          (target_screen[0]+1, target_screen[1]+1), target_size + pulse + 1)  # Shadow
        pygame.draw.circle(screen, self.WAYPOINT_COLOR, target_screen, target_size + pulse)
        pygame.draw.circle(screen, (255, 255, 255), target_screen, target_size - 2)
        pygame.draw.circle(screen, self.WAYPOINT_COLOR, target_screen, 2)
        
        # Draw enhanced crosshairs
        pygame.draw.line(screen, (255, 255, 255), 
                        (target_screen[0] - target_size - 3, target_screen[1]), 
                        (target_screen[0] + target_size + 3, target_screen[1]), 2)
        pygame.draw.line(screen, (255, 255, 255), 
                        (target_screen[0], target_screen[1] - target_size - 3), 
                        (target_screen[0], target_screen[1] + target_size + 3), 2)
    
    def toggle_lidar_overlay(self):
        """Toggle LIDAR overlay on/off"""
        self.show_lidar_overlay = not self.show_lidar_overlay
        print(f"🔍 LIDAR overlay: {'ON' if self.show_lidar_overlay else 'OFF'}")
    
    def update_lidar_data(self, lidar_points, robot_x=0.0, robot_y=0.0, robot_theta=0.0):
        """Update LIDAR points for overlay display and obstacle detection"""
        self.lidar_points = []
        self.obstacle_detected = False
        
        # Safety distances from lidar_cartesian_plot.py
        front_distance = 0.30   # 30cm front
        side_distance = 0.25    # 25cm left/right
        back_distance = 0.20    # 20cm back
        
        # Convert LIDAR points to world coordinates and check for obstacles
        for point in lidar_points:
            if hasattr(point, 'distance') and hasattr(point, 'angle'):
                # Convert polar to cartesian relative to robot (match lidar_cartesian_plot.py)
                local_x = point.distance * math.cos(point.angle)
                local_y = -point.distance * math.sin(point.angle)  # Flip Y to correct left/right
                
                # Check if point is in safety zone (same logic as lidar_cartesian_plot.py)
                if self._is_point_in_safety_zone(local_x, local_y, front_distance, side_distance, back_distance):
                    self.obstacle_detected = True
                
                # Transform to world coordinates
                cos_theta = math.cos(robot_theta)
                sin_theta = math.sin(robot_theta)
                world_x = robot_x + (local_x * cos_theta - local_y * sin_theta)
                world_y = robot_y + (local_x * sin_theta + local_y * cos_theta)
                
                self.lidar_points.append((world_x, world_y, point.distance, local_x, local_y))
    
    def _is_point_in_safety_zone(self, x, y, front_distance, side_distance, back_distance):
        """Check if point is within any directional safety zone (from lidar_cartesian_plot.py)"""
        # Front zone (x > 0, within side distance)
        if x > 0 and x <= front_distance and abs(y) <= side_distance:
            return True
        # Back zone (x < 0, within side distance)  
        if x < 0 and x >= -back_distance and abs(y) <= side_distance:
            return True
        # Left zone (y > 0, within front/back range)
        if y > 0 and y <= side_distance and -back_distance <= x <= front_distance:
            return True
        # Right zone (y < 0, within front/back range)
        if y < 0 and y >= -side_distance and -back_distance <= x <= front_distance:
            return True
        return False
    
    def _draw_lidar_overlay(self, screen):
        """Draw LIDAR scan overlay with safety zones"""
        if not self.lidar_points:
            return
        
        # Draw safety zones around robot (if we have robot position)
        if hasattr(self, 'robot_x') and hasattr(self, 'robot_y'):
            self._draw_safety_zones(screen, self.robot_x, self.robot_y, getattr(self, 'robot_theta', 0.0))
        
        # Draw LIDAR points with distance-based coloring and safety highlighting
        for data in self.lidar_points:
            if len(data) >= 5:
                world_x, world_y, distance, local_x, local_y = data[:5]
            else:
                world_x, world_y, distance = data[:3]
                local_x, local_y = 0, 0
            
            screen_pos = self.world_to_screen(world_x, world_y)
            
            # Check if point is in safety zone for highlighting
            front_distance, side_distance, back_distance = 0.30, 0.25, 0.20
            in_safety_zone = self._is_point_in_safety_zone(local_x, local_y, front_distance, side_distance, back_distance)
            
            # Color based on safety zone first, then distance
            if in_safety_zone:
                color = (255, 50, 50)  # Bright red for danger
                size = 6
            elif distance < 0.5:  # Very close - red
                color = (255, 100, 100)
                size = 4
            elif distance < 1.0:  # Medium distance - yellow
                color = (255, 255, 100)
                size = 3
            elif distance < 2.0:  # Far - green
                color = (100, 255, 100)
                size = 2
            else:  # Very far - blue
                color = (100, 100, 255)
                size = 2
            
            # Draw LIDAR point
            pygame.draw.circle(screen, color, screen_pos, size)
            pygame.draw.circle(screen, (255, 255, 255), screen_pos, size, 1)  # White outline
        
        # Draw obstacle warning overlay
        if self.obstacle_detected:
            self._draw_obstacle_warning(screen)
    
    def _draw_safety_zones(self, screen, robot_x, robot_y, robot_theta):
        """Draw safety zones around robot (from lidar_cartesian_plot.py)"""
        front_distance, side_distance, back_distance = 0.30, 0.25, 0.20
        
        # Transform safety zone corners to world coordinates
        cos_theta = math.cos(robot_theta)
        sin_theta = math.sin(robot_theta)
        
        # Front zone
        front_corners = [
            (0, -side_distance/2), (front_distance, -side_distance/2),
            (front_distance, side_distance/2), (0, side_distance/2)
        ]
        front_world = []
        for lx, ly in front_corners:
            wx = robot_x + (lx * cos_theta - ly * sin_theta)
            wy = robot_y + (lx * sin_theta + ly * cos_theta)
            front_world.append(self.world_to_screen(wx, wy))
        
        # Draw safety zones
        zone_color = (255, 0, 0) if self.obstacle_detected else (0, 255, 0)
        pygame.draw.polygon(screen, zone_color, front_world, 2)
    
    def _draw_obstacle_warning(self, screen):
        """Draw obstacle warning overlay - just red overlay, no text"""
        # Semi-transparent red overlay
        warning_surface = pygame.Surface((self.width, self.height))
        warning_surface.set_alpha(50)
        warning_surface.fill((255, 0, 0))
        screen.blit(warning_surface, (0, 0))
    
    def handle_key_press(self, key):
        """Handle keyboard input for map controls"""
        if key == pygame.K_l:  # 'L' key toggles LIDAR overlay
            self.toggle_lidar_overlay()
            return True
        return False
    
    def update_robot_position(self, x, y, theta):
        """Update robot position for safety zone display"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
    
    def is_obstacle_detected(self):
        """Check if obstacle is detected in safety zones"""
        return self.obstacle_detected
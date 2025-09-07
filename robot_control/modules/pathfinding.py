#!/usr/bin/env python3
"""
Pathfinding Module
Implements A* and other pathfinding algorithms for robot navigation
"""

import heapq
import math
from typing import List, Tuple, Optional, Set, Dict

class AStarPathfinder:
    """A* pathfinding algorithm implementation"""
    
    def __init__(self, map_environment):
        self.map_env = map_environment
        self.grid_size = map_environment.grid_size * 2  # Use larger grid for angular paths
        
        # Logging
        import logging
        self.logger = logging.getLogger('AStarPathfinder')
    
    def heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Euclidean distance heuristic"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Calculate actual distance between two points"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        
        # Diagonal movement cost
        if dx > 0 and dy > 0:
            return math.sqrt(dx**2 + dy**2)
        else:
            return dx + dy
    
    def snap_to_grid(self, x: float, y: float) -> Tuple[float, float]:
        """Snap coordinates to grid"""
        grid_x = round(x / self.grid_size) * self.grid_size
        grid_y = round(y / self.grid_size) * self.grid_size
        return grid_x, grid_y
    
    def find_nearest_road_point(self, x: float, y: float) -> Tuple[float, float]:
        """Find the nearest road centerline point to given coordinates"""
        best_point = (x, y)
        min_distance = float('inf')
        
        # FIRST: Try to snap directly to road centerlines
        for road in self.map_env.roads:
            centerline_point = self._snap_to_road_centerline(x, y, road)
            if centerline_point:
                distance = math.sqrt((centerline_point[0] - x)**2 + (centerline_point[1] - y)**2)
                if distance < min_distance:
                    min_distance = distance
                    best_point = centerline_point
        
        # If direct snap worked, return it
        if min_distance < float('inf'):
            return self.snap_to_grid(*best_point)
        
        # FALLBACK: Search in expanding radius for any road centerline
        search_radius = 0.1   # Start with 10cm radius
        max_radius = 3.0      # Maximum 3m radius
        
        while search_radius <= max_radius:
            # Check points in a circle around the target
            for angle in range(0, 360, 15):  # Every 15 degrees
                test_x = x + search_radius * math.cos(math.radians(angle))
                test_y = y + search_radius * math.sin(math.radians(angle))
                
                # Check if point is near road center (not just on road)
                if self.map_env._is_point_on_road_center(test_x, test_y):
                    distance = math.sqrt((test_x - x)**2 + (test_y - y)**2)
                    if distance < min_distance:
                        min_distance = distance
                        best_point = (test_x, test_y)
            
            if min_distance < float('inf'):
                break
                
            search_radius += 0.1  # Expand search by 10cm
        
        return self.snap_to_grid(*best_point)
    
    def _snap_to_road_centerline(self, x: float, y: float, road) -> Optional[Tuple[float, float]]:
        """Snap a point to the centerline of a specific road"""
        if road['type'] == 'horizontal':
            # Check if point is within road's X range
            if road['x_start'] <= x <= road['x_end']:
                # Snap to horizontal centerline
                return (x, road['y'])
        
        elif road['type'] == 'vertical':
            # Check if point is within road's Y range
            if road['y_start'] <= y <= road['y_end']:
                # Snap to vertical centerline  
                return (road['x'], y)
        
        return None
    
    def _add_junction_waypoints(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Add junction center waypoints to ensure robot goes to junction centers first"""
        if len(path) < 3:
            return path
        
        enhanced_path = [path[0]]  # Start with first point
        
        for i in range(1, len(path) - 1):
            prev_point = path[i - 1]
            current_point = path[i]
            next_point = path[i + 1]
            
            # Check if this is a direction change (potential junction)
            if self._is_direction_change(prev_point, current_point, next_point):
                # Find the actual junction center
                junction_center = self._find_junction_center(current_point)
                if junction_center and junction_center != current_point:
                    # Add waypoint to go to junction center first
                    enhanced_path.append(junction_center)
                    self.logger.info(f"🛣️  Added junction center waypoint: {junction_center}")
            
            enhanced_path.append(current_point)
        
        enhanced_path.append(path[-1])  # Add final goal
        return enhanced_path
    
    def _is_direction_change(self, prev_point: Tuple[float, float], 
                            current_point: Tuple[float, float], 
                            next_point: Tuple[float, float]) -> bool:
        """Check if there's a significant direction change at this point"""
        # Vector from prev to current
        v1_x = current_point[0] - prev_point[0]
        v1_y = current_point[1] - prev_point[1]
        
        # Vector from current to next
        v2_x = next_point[0] - current_point[0]
        v2_y = next_point[1] - current_point[1]
        
        # Normalize vectors
        v1_len = math.sqrt(v1_x**2 + v1_y**2)
        v2_len = math.sqrt(v2_x**2 + v2_y**2)
        
        if v1_len < 0.001 or v2_len < 0.001:
            return False
        
        v1_x /= v1_len
        v1_y /= v1_len
        v2_x /= v2_len
        v2_y /= v2_len
        
        # Dot product to check if direction changed
        dot_product = v1_x * v2_x + v1_y * v2_y
        
        # If dot product < 0.5, there's a significant direction change (>60 degrees)
        return dot_product < 0.5
    
    def _find_junction_center(self, point: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """Find the center of the junction nearest to this point"""
        # Get all intersections from map environment
        intersections = self.map_env._find_intersections()
        
        if not intersections:
            return None
        
        # Find the closest intersection
        closest_intersection = None
        min_distance = float('inf')
        
        for intersection in intersections:
            distance = math.sqrt((intersection[0] - point[0])**2 + (intersection[1] - point[1])**2)
            if distance < min_distance and distance < 1.0:  # Within 1 meter
                min_distance = distance
                closest_intersection = intersection
        
        return closest_intersection
    
    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Find path from start to goal using A* algorithm
        Returns list of waypoints or None if no path found
        """
        # Snap coordinates to grid and ensure they're on roads
        start = self.find_nearest_road_point(*start)
        goal = self.find_nearest_road_point(*goal)
        
        if start == goal:
            return [start, goal]
        
        # A* algorithm implementation
        open_set = [(0, start)]
        came_from: Dict[Tuple[float, float], Tuple[float, float]] = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        closed_set: Set[Tuple[float, float]] = set()
        
        max_iterations = 5000  # Prevent infinite loops
        iterations = 0
        
        # Verify start and goal are valid
        if not self.map_env.is_point_on_road(*start) or not self.map_env.is_point_on_road(*goal):
            print(f"❌ Invalid start/goal: Start on road: {self.map_env.is_point_on_road(*start)}, Goal on road: {self.map_env.is_point_on_road(*goal)}")
            return None
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            # Get point with lowest f_score
            current_f, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Check if we reached the goal
            if self.get_distance(current, goal) < self.grid_size * 0.5:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                path.append(goal)  # Add final goal
                
                # Return simple path - let human-like navigation handle it
                return path
            
            # Explore neighbors
            for neighbor in self.map_env.get_neighbors(*current):
                if neighbor in closed_set:
                    continue
                
                # Calculate tentative g_score
                tentative_g = g_score[current] + self.get_distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # This path to neighbor is better
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    
                    # Add to open set if not already there
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        return None
    
    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path while preserving angular constraints and turn points"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]  # Always keep start point
        
        i = 0
        while i < len(path) - 1:
            current_point = path[i]
            
            # Look for direction changes (turn points) that must be preserved
            farthest = i + 1
            last_direction = None
            
            for j in range(i + 1, len(path)):
                # Calculate direction from current to this point
                dx = path[j][0] - current_point[0]
                dy = path[j][1] - current_point[1]
                
                # Normalize direction to detect angle changes
                if abs(dx) > 0.001 or abs(dy) > 0.001:
                    direction = (1 if dx > 0 else (-1 if dx < 0 else 0),
                               1 if dy > 0 else (-1 if dy < 0 else 0))
                    
                    # If direction changed, this is a turn point - must keep previous point
                    if last_direction and direction != last_direction:
                        break
                    
                    # Check if line is clear for angular movement
                    if self.is_line_clear(current_point, path[j]):
                        farthest = j
                        last_direction = direction
                    else:
                        break
            
            # Add the farthest reachable point (preserving turn points)
            if farthest < len(path):
                smoothed.append(path[farthest])
            
            i = farthest
        
        # Ensure goal is included
        if smoothed[-1] != path[-1]:
            smoothed.append(path[-1])
        
        return smoothed
    
    def is_line_clear(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        """Check if a straight line between two points is clear of obstacles"""
        distance = self.get_distance(start, end)
        
        if distance == 0:
            return True
        
        # Check points along the line
        num_checks = max(int(distance / (self.grid_size * 0.5)), 2)
        
        for i in range(num_checks + 1):
            t = i / num_checks
            check_x = start[0] + t * (end[0] - start[0])
            check_y = start[1] + t * (end[1] - start[1])
            
            if self.map_env.is_point_blocked(check_x, check_y):
                return False
        
        return True


class RRTPathfinder:
    """Rapidly-exploring Random Tree (RRT) pathfinding algorithm"""
    
    def __init__(self, map_environment):
        self.map_env = map_environment
        self.max_extend_distance = 0.2  # 20cm max extension
        self.goal_sample_rate = 0.1     # 10% chance to sample goal
    
    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float], 
                  max_iterations: int = 5000) -> Optional[List[Tuple[float, float]]]:
        """
        Find path using RRT algorithm
        Returns list of waypoints or None if no path found
        """
        # Ensure start and goal are on roads
        astar = AStarPathfinder(self.map_env)
        start = astar.find_nearest_road_point(*start)
        goal = astar.find_nearest_road_point(*goal)
        
        # Initialize tree with start node
        tree = {start: None}  # node -> parent
        
        map_width_m = self.map_env.width / self.map_env.scale
        map_height_m = self.map_env.height / self.map_env.scale
        
        for _ in range(max_iterations):
            # Sample random point (or goal)
            if random.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = (
                    random.uniform(0, map_width_m),
                    random.uniform(0, map_height_m)
                )
            
            # Find nearest node in tree
            nearest = min(tree.keys(), key=lambda n: self._distance(n, sample))
            
            # Extend towards sample
            new_node = self._extend(nearest, sample)
            
            if new_node and not self.map_env.is_point_blocked(*new_node):
                tree[new_node] = nearest
                
                # Check if we reached goal
                if self._distance(new_node, goal) < 0.1:  # Within 10cm
                    # Reconstruct path
                    path = []
                    current = new_node
                    while current is not None:
                        path.append(current)
                        current = tree[current]
                    path.reverse()
                    path.append(goal)
                    return path
        
        return None  # No path found
    
    def _distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Calculate Euclidean distance"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def _extend(self, from_node: Tuple[float, float], to_point: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """Extend from node towards point by max_extend_distance"""
        distance = self._distance(from_node, to_point)
        
        if distance <= self.max_extend_distance:
            return to_point
        
        # Extend by max distance
        ratio = self.max_extend_distance / distance
        new_x = from_node[0] + ratio * (to_point[0] - from_node[0])
        new_y = from_node[1] + ratio * (to_point[1] - from_node[1])
        
        return (new_x, new_y)


# Import random for RRT
import random

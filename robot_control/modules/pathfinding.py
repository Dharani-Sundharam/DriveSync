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
        self.grid_size = map_environment.grid_size
    
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
        """Find the nearest valid road point to given coordinates"""
        best_point = (x, y)
        min_distance = float('inf')
        
        # Search in expanding radius
        search_radius = 0.05  # Start with 5cm radius
        max_radius = 2.0      # Maximum 2m radius
        
        while search_radius <= max_radius:
            # Check points in a circle around the target
            for angle in range(0, 360, 10):  # Every 10 degrees
                test_x = x + search_radius * math.cos(math.radians(angle))
                test_y = y + search_radius * math.sin(math.radians(angle))
                
                if self.map_env.is_point_on_road(test_x, test_y):
                    distance = math.sqrt((test_x - x)**2 + (test_y - y)**2)
                    if distance < min_distance:
                        min_distance = distance
                        best_point = (test_x, test_y)
            
            if min_distance < float('inf'):
                break
                
            search_radius += 0.05  # Expand search by 5cm
        
        return self.snap_to_grid(*best_point)
    
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
                
                # Smooth the path
                return self.smooth_path(path)
            
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
        """Smooth the path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]  # Always keep start point
        
        i = 0
        while i < len(path) - 1:
            # Look ahead to find the farthest point we can reach directly
            farthest = i + 1
            
            for j in range(i + 2, len(path)):
                if self.is_line_clear(path[i], path[j]):
                    farthest = j
                else:
                    break
            
            # Add the farthest reachable point
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

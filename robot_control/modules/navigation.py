#!/usr/bin/env python3
"""
Navigation Module
Handles path following and robot navigation control
"""

import math
import time
from typing import List, Tuple, Optional
from enum import Enum

class NavigationState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    REACHED_GOAL = "reached_goal"
    PATH_BLOCKED = "path_blocked"
    TURNING = "turning"

class NavigationController:
    """Controls robot navigation along planned paths"""
    
    def __init__(self, robot_controller, map_environment):
        self.robot = robot_controller
        self.map_env = map_environment
        
        # Navigation parameters
        self.waypoint_tolerance = 0.05  # 5cm tolerance to reach waypoint
        self.goal_tolerance = 0.08      # 8cm tolerance to reach final goal
        self.max_speed = 120            # Maximum motor speed
        self.turn_speed = 80            # Speed for turning
        self.approach_speed = 60        # Speed when approaching waypoint
        
        # PID controller parameters for stable path following
        self.kp_angular = 150.0   # Proportional gain for angular control
        self.ki_angular = 0.0     # Integral gain (start with 0)
        self.kd_angular = 20.0    # Derivative gain for stability
        
        # PID state variables
        self.previous_angular_error = 0.0
        self.angular_error_integral = 0.0
        self.last_time = time.time()
        
        # Navigation state
        self.state = NavigationState.IDLE
        self.current_path: List[Tuple[float, float]] = []
        self.current_waypoint_index = 0
        self.target_goal: Optional[Tuple[float, float]] = None
        
        # Timing
        self.last_navigation_update = time.time()
        self.navigation_frequency = 20  # 20 Hz navigation updates
        
        # Logging
        import logging
        self.logger = logging.getLogger('Navigation')
    
    def set_target(self, target_x: float, target_y: float, pathfinder):
        """Set a new navigation target and plan path"""
        self.target_goal = (target_x, target_y)
        
        # Get current robot position
        current_pos = (self.robot.x, self.robot.y)
        
        # Plan path using pathfinder
        self.logger.info(f"🎯 Planning path from {current_pos} to {self.target_goal}")
        path = pathfinder.find_path(current_pos, self.target_goal)
        
        if path and len(path) > 1:
            self.current_path = path
            self.current_waypoint_index = 0
            self.state = NavigationState.NAVIGATING
            
            # Reset PID controller for fresh start
            self.previous_angular_error = 0.0
            self.angular_error_integral = 0.0
            self.last_time = time.time()
            
            self.logger.info(f"✅ Path planned with {len(path)} waypoints")
            
            # Log the path
            for i, waypoint in enumerate(path):
                self.logger.debug(f"   Waypoint {i}: ({waypoint[0]:.3f}, {waypoint[1]:.3f})")
        else:
            self.logger.warning("❌ No path found to target")
            self.state = NavigationState.PATH_BLOCKED
            self.current_path = []
            
            # Try direct movement if close enough
            distance_to_target = math.sqrt((self.target_goal[0] - self.robot.x)**2 + (self.target_goal[1] - self.robot.y)**2)
            if distance_to_target < 0.5:  # Within 50cm
                self.logger.info("🔄 Attempting direct movement to nearby target")
                self.current_path = [current_pos, self.target_goal]
                self.current_waypoint_index = 0
                self.state = NavigationState.NAVIGATING
    
    def update(self):
        """Update navigation controller (call this regularly)"""
        current_time = time.time()
        
        # Check if it's time for navigation update
        if current_time - self.last_navigation_update < (1.0 / self.navigation_frequency):
            return
        
        self.last_navigation_update = current_time
        
        if self.state == NavigationState.NAVIGATING:
            self._navigate_to_current_waypoint()
        elif self.state == NavigationState.TURNING:
            self._handle_turning()
    
    def _navigate_to_current_waypoint(self):
        """Navigate towards the current waypoint"""
        if not self.current_path or self.current_waypoint_index >= len(self.current_path):
            self.state = NavigationState.REACHED_GOAL
            self.robot.send_motor_command(0, 0)  # Stop
            self.logger.info("🏁 Reached final goal!")
            return
        
        # Get current waypoint
        waypoint = self.current_path[self.current_waypoint_index]
        current_pos = (self.robot.x, self.robot.y)
        
        # Calculate distance to waypoint
        distance_to_waypoint = math.sqrt(
            (waypoint[0] - current_pos[0])**2 + 
            (waypoint[1] - current_pos[1])**2
        )
        
        # Check if we reached the current waypoint
        tolerance = self.goal_tolerance if self.current_waypoint_index == len(self.current_path) - 1 else self.waypoint_tolerance
        
        if distance_to_waypoint < tolerance:
            self.current_waypoint_index += 1
            self.logger.info(f"📍 Reached waypoint {self.current_waypoint_index}/{len(self.current_path)}")
            
            if self.current_waypoint_index >= len(self.current_path):
                self.state = NavigationState.REACHED_GOAL
                self.robot.send_motor_command(0, 0)
                self.logger.info("🎉 Navigation completed!")
                return
            
            # Get next waypoint
            waypoint = self.current_path[self.current_waypoint_index]
        
        # Calculate control commands
        left_speed, right_speed = self._calculate_motor_speeds(current_pos, waypoint)
        
        # Send motor commands
        try:
            self.robot.send_motor_command(left_speed, right_speed)
        except Exception as e:
            self.logger.warning(f"⚠️  Motor command failed: {e}")
            # Continue navigation despite communication error
    
    def _calculate_motor_speeds(self, current_pos: Tuple[float, float], 
                               target_pos: Tuple[float, float]) -> Tuple[int, int]:
        """HUMAN-LIKE DRIVING: Look where you want to go, turn towards it, drive straight"""
        
        # Where do I want to go?
        dx = target_pos[0] - current_pos[0] 
        dy = target_pos[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.05:  # Very close - stop
            self.logger.info("✅ ARRIVED: At waypoint")
            return 0, 0
        
        # Which way am I facing?
        robot_facing_x = math.cos(self.robot.theta)
        robot_facing_y = math.sin(self.robot.theta)
        
        # Which way should I be facing to reach the target?
        target_direction_x = dx / distance
        target_direction_y = dy / distance
        
        # Am I facing the right way?
        # Cross product tells me if I need to turn left or right
        cross_product = target_direction_x * robot_facing_y - target_direction_y * robot_facing_x
        
        # Dot product tells me if I'm facing roughly the right direction
        dot_product = target_direction_x * robot_facing_x + target_direction_y * robot_facing_y
        
        self.logger.info(f"🎯 Target: ({target_pos[0]:.2f}, {target_pos[1]:.2f}), "
                         f"Dist: {distance:.2f}m, Need turn: {abs(cross_product):.2f}")
        
        # HUMAN LOGIC: If I'm not facing the right way, turn first
        if abs(cross_product) > 0.2:  # Need to turn (about 11 degrees)
            # Turn towards target
            if cross_product > 0:
                left_speed = -80   # Turn left
                right_speed = 80
                self.logger.info("🔄 TURN LEFT: Face target")
            else:
                left_speed = 80    # Turn right
                right_speed = -80
                self.logger.info("🔄 TURN RIGHT: Face target")
        
        else:
            # I'm facing the right way - drive straight!
            
            # Choose speed based on distance
            if distance < 0.1:
                speed = 50  # Slow when close
                self.logger.info("🐌 SLOW: Close to target")
            else:
                speed = 100  # Normal speed
                self.logger.info("🚗 DRIVE: Straight to target")
            
            # Go straight (equal motor speeds)
            left_speed = right_speed = speed
        
        self.logger.info(f"🎮 Motors: L={left_speed}, R={right_speed}")
        return left_speed, right_speed
    
    def _get_path_direction(self, current_pos: Tuple[float, float], 
                           target_pos: Tuple[float, float]) -> Tuple[float, float]:
        """Calculate the direction of the yellow line (path segment)"""
        
        # If we have multiple waypoints, use the direction between consecutive waypoints
        if (len(self.current_path) > 1 and 
            self.current_waypoint_index < len(self.current_path) - 1):
            
            # Get current and next waypoint to determine path direction
            current_waypoint = self.current_path[self.current_waypoint_index]
            next_waypoint = self.current_path[self.current_waypoint_index + 1]
            
            # Direction vector from current to next waypoint (yellow line direction)
            dx = next_waypoint[0] - current_waypoint[0]
            dy = next_waypoint[1] - current_waypoint[1]
            
        else:
            # Only one waypoint or last waypoint - use direction to target
            dx = target_pos[0] - current_pos[0]
            dy = target_pos[1] - current_pos[1]
        
        # Normalize to unit vector
        length = math.sqrt(dx**2 + dy**2)
        if length > 0.001:  # Avoid division by zero
            return dx / length, dy / length
        else:
            return 1.0, 0.0  # Default to pointing right
    
    def _handle_turning(self):
        """Handle pure turning state"""
        # This could be implemented for more sophisticated turning behavior
        pass
    
    def stop_navigation(self):
        """Stop current navigation"""
        self.state = NavigationState.IDLE
        self.current_path = []
        self.current_waypoint_index = 0
        self.target_goal = None
        self.robot.send_motor_command(0, 0)
        self.logger.info("⏹️  Navigation stopped")
    
    def is_navigating(self) -> bool:
        """Check if robot is currently navigating"""
        return self.state in [NavigationState.NAVIGATING, NavigationState.TURNING]
    
    def get_current_waypoint(self) -> Optional[Tuple[float, float]]:
        """Get the current target waypoint"""
        if (self.current_path and 
            0 <= self.current_waypoint_index < len(self.current_path)):
            return self.current_path[self.current_waypoint_index]
        return None
    
    def get_navigation_info(self) -> dict:
        """Get current navigation status information"""
        info = {
            'state': self.state.value,
            'has_path': len(self.current_path) > 0,
            'waypoint_index': self.current_waypoint_index,
            'total_waypoints': len(self.current_path),
            'target_goal': self.target_goal,
            'current_waypoint': self.get_current_waypoint()
        }
        
        # Calculate progress percentage
        if self.current_path:
            progress = (self.current_waypoint_index / len(self.current_path)) * 100
            info['progress_percent'] = min(progress, 100)
        else:
            info['progress_percent'] = 0
        
        return info

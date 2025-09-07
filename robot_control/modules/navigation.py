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
        self.max_speed = 150            # Maximum motor speed
        self.turn_speed = 100           # Speed for turning
        self.approach_speed = 80        # Speed when approaching waypoint
        
        # PID controller parameters for path following
        self.kp_linear = 200.0    # Proportional gain for linear velocity
        self.kp_angular = 300.0   # Proportional gain for angular velocity
        self.max_angular_error = math.pi / 4  # 45 degrees max correction
        
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
        self.robot.send_motor_command(left_speed, right_speed)
    
    def _calculate_motor_speeds(self, current_pos: Tuple[float, float], 
                               target_pos: Tuple[float, float]) -> Tuple[int, int]:
        """Calculate motor speeds for differential drive navigation"""
        
        # Calculate target angle
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        target_angle = math.atan2(dy, dx)
        
        # Calculate angular error
        angular_error = target_angle - self.robot.theta
        
        # Normalize angular error to [-π, π]
        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi
        
        # Calculate distance to target
        distance = math.sqrt(dx**2 + dy**2)
        
        # If we need to turn significantly, prioritize turning
        if abs(angular_error) > math.pi / 6:  # More than 30 degrees
            # Pure rotation
            angular_velocity = self.kp_angular * angular_error
            angular_velocity = max(min(angular_velocity, self.turn_speed), -self.turn_speed)
            
            left_speed = int(angular_velocity)   # Fixed to match corrected motor mapping
            right_speed = -int(angular_velocity) # Fixed to match corrected motor mapping
            
            self.logger.debug(f"🔄 Turning: error={math.degrees(angular_error):.1f}°, "
                            f"L={left_speed}, R={right_speed}")
        else:
            # Forward motion with steering
            base_speed = self.max_speed
            
            # Reduce speed when approaching waypoint
            if distance < 0.3:  # Within 30cm
                base_speed = self.approach_speed
            
            # Calculate steering correction
            steering = self.kp_angular * angular_error
            steering = max(min(steering, base_speed), -base_speed)
            
            # Apply differential steering
            left_speed = int(base_speed - steering)
            right_speed = int(base_speed + steering)
            
            # Ensure speeds are within limits
            left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
            right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
            
            self.logger.debug(f"➡️  Forward: dist={distance:.3f}m, "
                            f"angle_err={math.degrees(angular_error):.1f}°, "
                            f"L={left_speed}, R={right_speed}")
        
        return left_speed, right_speed
    
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

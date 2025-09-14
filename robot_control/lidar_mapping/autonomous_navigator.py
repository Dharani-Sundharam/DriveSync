#!/usr/bin/env python3
"""
Autonomous Navigator with LIDAR Obstacle Avoidance and Localization

Mathematical Concepts Used:
1. Pose Estimation: Using LIDAR scan matching to determine robot's position and orientation
2. Obstacle Avoidance: Vector field navigation around detected obstacles
3. Path Planning: Simple waypoint navigation with dynamic replanning
4. Coordinate Transformations: Converting between robot frame and world frame
5. Angular Velocity Calculation: Using landmark tracking for rotation estimation
"""

import math
import time
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
from simple_slam import SimpleSLAM
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from smooth_robot_controller import OptimizedRobotController

@dataclass
class Pose:
    """Robot pose in 2D space"""
    x: float = 0.0      # X position (meters)
    y: float = 0.0      # Y position (meters)
    theta: float = 0.0  # Orientation (radians)

@dataclass
class Waypoint:
    """Target waypoint for navigation"""
    x: float
    y: float
    tolerance: float = 0.1  # Arrival tolerance (meters)

class AutonomousNavigator:
    def __init__(self, lidar_port='/dev/ttyUSB1', robot_port='/dev/ttyUSB0'):
        # Initialize LIDAR
        config = LidarConfig(port=lidar_port, baudrate=115200)
        self.lidar = YDLidarX2Optimized(config)
        
        # Initialize robot controller
        self.robot = OptimizedRobotController(port=robot_port, baudrate=115200)
        
        # Initialize SLAM system - 3m x 3m map with more space
        self.slam = SimpleSLAM(map_size_m=3.0, resolution=0.03)
        
        # Robot state
        self.pose = Pose()
        self.previous_pose = Pose()
        self.target_waypoint = None
        self.start_position = (1.5, 1.5)  # Start at center of 3m map
        self.max_travel_distance = 2.0   # Larger travel distance for bigger map
        
        # Initialize robot pose in SLAM
        self.slam.robot_x = self.start_position[0]
        self.slam.robot_y = self.start_position[1]
        
        # Safety distances - adjusted for bigger workspace
        self.front_distance = 0.25   # 25cm
        self.side_distance = 0.20    # 20cm  
        self.back_distance = 0.15    # 15cm
        
        # Navigation parameters
        self.max_linear_speed = 0.25  # m/s - increase speed
        self.max_angular_speed = 0.8  # rad/s - faster turns
        self.obstacle_avoidance_gain = 3.0  # stronger avoidance
        self.goal_attraction_gain = 1.0
        self.obstacle_slow_speed = 0.15  # Faster when obstacles detected
        
        # Motor control parameters
        self.min_turn_pwm = 120  # Minimum PWM for effective turning
        self.curve_speed = 0.1   # Speed when curving around obstacles
        
        # Gap detection parameters - adjusted for bigger workspace
        self.min_gap_width = 0.3  # Minimum gap width to pass through (meters)
        self.gap_detection_range = 1.5  # Range to look for gaps (meters)
        
        # Angle tracking for LIDAR feedback
        self.target_heading = 0.0
        self.heading_tolerance = 0.1  # radians (~5.7 degrees)
        
        # Localization parameters
        self.landmark_memory = []  # Store previous scan landmarks
        self.pose_history = []     # Store pose history for smoothing
        
        print("🤖 Autonomous Navigator Initialized")
        print(f"Safety zones - Front: {self.front_distance}m, Sides: {self.side_distance}m, Back: {self.back_distance}m")
        
        # Start robot controller threads
        if self.robot.serial_port:
            self.robot.start_threads()
            time.sleep(1)  # Let threads initialize
            print("✅ Robot controller threads started")

    def connect_lidar(self):
        """Connect to LIDAR sensor"""
        if self.lidar.connect():
            if self.lidar.start_scanning():
                print("✅ LIDAR connected and scanning")
                return True
            else:
                print("❌ Failed to start LIDAR scanning")
                return False
        else:
            print("❌ Failed to connect to LIDAR")
            return False

    def get_lidar_points(self) -> List[Tuple[float, float]]:
        """Get current LIDAR points in Cartesian coordinates"""
        scan = self.lidar.get_latest_scan()
        points = []
        
        if scan and scan.points:
            for point in scan.points:
                if 0.05 <= point.distance <= 6.0:
                    # Convert polar to cartesian (with Y-axis flip for correct orientation)
                    x = point.distance * math.cos(point.angle)
                    y = -point.distance * math.sin(point.angle)
                    points.append((x, y))
        
        return points

    def detect_obstacles_in_zones(self, points: List[Tuple[float, float]]) -> dict:
        """
        Detect obstacles in different safety zones
        
        Mathematical Concept: Zone-based collision detection
        - Divides space around robot into directional zones
        - Uses rectangular boundary checking for each zone
        """
        zones = {
            'front': [],
            'back': [],
            'left': [],
            'right': []
        }
        
        for x, y in points:
            # Front zone (x > 0, within side distance)
            if x > 0 and x <= self.front_distance and abs(y) <= self.side_distance:
                zones['front'].append((x, y))
            
            # Back zone (x < 0, within side distance)
            elif x < 0 and x >= -self.back_distance and abs(y) <= self.side_distance:
                zones['back'].append((x, y))
            
            # Left zone (y > 0, within front/back range)
            elif y > 0 and y <= self.side_distance and -self.back_distance <= x <= self.front_distance:
                zones['left'].append((x, y))
            
            # Right zone (y < 0, within front/back range)
            elif y < 0 and y >= -self.side_distance and -self.back_distance <= x <= self.front_distance:
                zones['right'].append((x, y))
        
        return zones

    def find_gaps(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float, float]]:
        """
        Find navigable gaps between obstacles
        Returns list of (start_angle, end_angle, width) for each gap
        """
        if not points:
            return []
        
        # Convert points to polar coordinates and sort by angle
        polar_points = []
        for x, y in points:
            angle = math.atan2(y, x)
            distance = math.sqrt(x**2 + y**2)
            if distance <= self.gap_detection_range:
                polar_points.append((angle, distance))
        
        if not polar_points:
            return []
        
        polar_points.sort(key=lambda p: p[0])  # Sort by angle
        
        gaps = []
        angle_resolution = 0.05  # 2.9 degrees resolution
        
        # Create angle bins for gap detection
        angle_bins = {}
        for angle, distance in polar_points:
            bin_angle = round(angle / angle_resolution) * angle_resolution
            if bin_angle not in angle_bins or distance < angle_bins[bin_angle]:
                angle_bins[bin_angle] = distance
        
        # Find gaps in front sector (-90° to +90°)
        front_angles = []
        for angle in np.arange(-math.pi/2, math.pi/2, angle_resolution):
            bin_angle = round(angle / angle_resolution) * angle_resolution
            distance = angle_bins.get(bin_angle, self.gap_detection_range + 1)
            front_angles.append((angle, distance))
        
        # Identify gaps
        gap_start = None
        for i, (angle, distance) in enumerate(front_angles):
            is_clear = distance > self.front_distance + 0.1  # Extra margin
            
            if is_clear and gap_start is None:
                gap_start = angle
            elif not is_clear and gap_start is not None:
                # End of gap
                gap_end = angle
                gap_width = (gap_end - gap_start) * (self.front_distance + 0.2)
                
                if gap_width >= self.min_gap_width:
                    gaps.append((gap_start, gap_end, gap_width))
                
                gap_start = None
        
        # Check if gap extends to the end
        if gap_start is not None:
            gap_end = math.pi/2
            gap_width = (gap_end - gap_start) * (self.front_distance + 0.2)
            if gap_width >= self.min_gap_width:
                gaps.append((gap_start, gap_end, gap_width))
        
        return gaps

    def select_best_gap(self, gaps: List[Tuple[float, float, float]], target_angle: float = 0.0) -> Optional[float]:
        """
        Select the best gap to navigate through
        Returns the target angle for the gap center
        """
        if not gaps:
            return None
        
        best_gap = None
        best_score = -1
        
        for start_angle, end_angle, width in gaps:
            gap_center = (start_angle + end_angle) / 2
            
            # Score based on: width, proximity to target, and forward preference
            width_score = min(width / self.min_gap_width, 2.0)  # Cap at 2x
            angle_score = 1.0 - abs(gap_center - target_angle) / (math.pi/2)
            forward_score = 1.0 - abs(gap_center) / (math.pi/2)  # Prefer forward
            
            total_score = width_score * 0.4 + angle_score * 0.3 + forward_score * 0.3
            
            if total_score > best_score:
                best_score = total_score
                best_gap = gap_center
        
        return best_gap

    def estimate_pose_change(self, current_points: List[Tuple[float, float]]) -> Tuple[float, float, float]:
        """
        Estimate robot pose change using scan matching
        
        Mathematical Concept: Iterative Closest Point (ICP) approximation
        - Compares current scan with previous scan
        - Finds transformation (dx, dy, dtheta) that best aligns the scans
        - Uses centroid matching for simplicity
        """
        if not self.landmark_memory:
            self.landmark_memory = current_points.copy()
            return 0.0, 0.0, 0.0
        
        if not current_points or not self.landmark_memory:
            return 0.0, 0.0, 0.0
        
        # Simple centroid-based matching (approximation of ICP)
        current_centroid = np.mean(current_points, axis=0) if current_points else np.array([0, 0])
        previous_centroid = np.mean(self.landmark_memory, axis=0) if self.landmark_memory else np.array([0, 0])
        
        # Translation estimate
        dx = current_centroid[0] - previous_centroid[0]
        dy = current_centroid[1] - previous_centroid[1]
        
        # Rotation estimate using principal component analysis
        dtheta = self.estimate_rotation_change(current_points, self.landmark_memory)
        
        # Update landmark memory
        self.landmark_memory = current_points.copy()
        
        return dx, dy, dtheta

    def estimate_rotation_change(self, current_points: List[Tuple[float, float]], 
                               previous_points: List[Tuple[float, float]]) -> float:
        """
        Estimate rotation change using principal component analysis
        
        Mathematical Concept: Principal Component Analysis (PCA)
        - Finds the main direction of point distribution
        - Compares main directions between scans to estimate rotation
        """
        if len(current_points) < 3 or len(previous_points) < 3:
            return 0.0
        
        def get_principal_angle(points):
            if not points:
                return 0.0
            points_array = np.array(points)
            # Center the points
            centered = points_array - np.mean(points_array, axis=0)
            # Compute covariance matrix
            cov_matrix = np.cov(centered.T)
            # Get eigenvalues and eigenvectors
            eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
            # Principal direction is the eigenvector with largest eigenvalue
            principal_vector = eigenvectors[:, np.argmax(eigenvalues)]
            return math.atan2(principal_vector[1], principal_vector[0])
        
        current_angle = get_principal_angle(current_points)
        previous_angle = get_principal_angle(previous_points)
        
        # Calculate angle difference (handle wraparound)
        dtheta = current_angle - previous_angle
        while dtheta > math.pi:
            dtheta -= 2 * math.pi
        while dtheta < -math.pi:
            dtheta += 2 * math.pi
        
        return dtheta

    def update_pose(self, dt: float):
        """
        Update robot pose using LIDAR-based localization
        
        Mathematical Concept: Dead reckoning with sensor correction
        - Integrates velocity commands over time
        - Corrects using LIDAR scan matching
        """
        # Get current LIDAR data
        current_points = self.get_lidar_points()
        
        # Estimate pose change from LIDAR
        dx, dy, dtheta = self.estimate_pose_change(current_points)
        
        # Update pose (transform from robot frame to world frame)
        # Mathematical transformation: rotation matrix application
        cos_theta = math.cos(self.pose.theta)
        sin_theta = math.sin(self.pose.theta)
        
        # Transform movement from robot frame to world frame
        world_dx = cos_theta * dx - sin_theta * dy
        world_dy = sin_theta * dx + cos_theta * dy
        
        # Update pose with rotation calibration
        self.previous_pose = Pose(self.pose.x, self.pose.y, self.pose.theta)
        self.pose.x += world_dx
        self.pose.y += world_dy
        self.pose.theta += dtheta * 0.95  # Apply rotation calibration factor
        
        # Normalize angle
        while self.pose.theta > math.pi:
            self.pose.theta -= 2 * math.pi
        while self.pose.theta < -math.pi:
            self.pose.theta += 2 * math.pi
        
        # Store in history for smoothing
        self.pose_history.append(Pose(self.pose.x, self.pose.y, self.pose.theta))
        if len(self.pose_history) > 10:
            self.pose_history.pop(0)

    def calculate_obstacle_avoidance_vector(self, obstacle_zones: dict) -> Tuple[float, float]:
        """
        Calculate repulsive force from obstacles
        
        Mathematical Concept: Artificial Potential Fields
        - Each obstacle creates a repulsive force inversely proportional to distance
        - Forces are summed vectorially to get total avoidance direction
        """
        avoidance_x, avoidance_y = 0.0, 0.0
        
        for zone, obstacles in obstacle_zones.items():
            for obs_x, obs_y in obstacles:
                # Distance to obstacle
                distance = math.sqrt(obs_x**2 + obs_y**2)
                if distance < 0.01:  # Avoid division by zero
                    distance = 0.01
                
                # Repulsive force (inversely proportional to distance squared)
                force_magnitude = self.obstacle_avoidance_gain / (distance**2)
                
                # Direction away from obstacle (unit vector)
                force_x = -obs_x / distance * force_magnitude
                force_y = -obs_y / distance * force_magnitude
                
                avoidance_x += force_x
                avoidance_y += force_y
        
        return avoidance_x, avoidance_y

    def calculate_goal_attraction_vector(self) -> Tuple[float, float]:
        """
        Calculate attractive force toward goal
        
        Mathematical Concept: Attractive Potential Field
        - Goal creates attractive force proportional to distance
        - Direction is straight line toward goal
        """
        if not self.target_waypoint:
            return 0.0, 0.0
        
        # Vector from robot to goal (in world coordinates)
        goal_x = self.target_waypoint.x - self.pose.x
        goal_y = self.target_waypoint.y - self.pose.y
        
        # Distance to goal
        distance = math.sqrt(goal_x**2 + goal_y**2)
        
        if distance < 0.01:
            return 0.0, 0.0
        
        # Attractive force (proportional to distance, capped)
        force_magnitude = min(self.goal_attraction_gain * distance, self.max_linear_speed)
        
        # Unit vector toward goal
        attraction_x = goal_x / distance * force_magnitude
        attraction_y = goal_y / distance * force_magnitude
        
        return attraction_x, attraction_y

    def calculate_slam_navigation(self, points: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Simple SLAM-based navigation - move straight and avoid obstacles
        
        Algorithm:
        1. Update SLAM map with current scan
        2. Find free direction using map
        3. Move straight in free direction
        4. Simple obstacle avoidance
        """
        # Update SLAM map with current LIDAR data
        self.slam.update_map(points)
        
        # Calculate desired direction (towards goal or forward)
        target_direction = 0.0  # Default: straight ahead
        
        if self.target_waypoint:
            # Calculate direction to goal in robot frame
            dx = self.target_waypoint.x - self.slam.robot_x
            dy = self.target_waypoint.y - self.slam.robot_y
            goal_world_angle = math.atan2(dy, dx)
            target_direction = goal_world_angle - self.slam.robot_theta
            
            # Normalize to [-pi, pi]
            while target_direction > math.pi:
                target_direction -= 2 * math.pi
            while target_direction < -math.pi:
                target_direction += 2 * math.pi
        
        # Find free direction using SLAM map - bigger search range
        free_direction = self.slam.find_free_path_direction(target_direction, search_range=1.2)
        
        if free_direction is not None:
            # Convert to robot frame
            angle_error = free_direction - self.slam.robot_theta
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
            
            print(f"🗺️  SLAM: Free direction at {math.degrees(free_direction):.1f}° (robot frame: {math.degrees(angle_error):.1f}°)")
            
            # Simple control: turn towards free direction, then move forward
            if abs(angle_error) > 0.3:  # Need to turn (>17 degrees)
                linear_velocity = 0.1  # Slow forward while turning
                angular_velocity = angle_error * 2.0 * 0.95  # Apply rotation calibration
            else:
                # Good direction, move forward faster
                linear_velocity = self.max_linear_speed
                angular_velocity = angle_error * 1.0 * 0.95  # Apply rotation calibration
        else:
            # No free direction found, turn in place to explore
            print("🔄 SLAM: No free path, exploring...")
            linear_velocity = 0.05  # Small forward motion while exploring
            angular_velocity = 0.5 * 0.95  # Apply rotation calibration to exploration
        
        # Limit velocities
        linear_velocity = max(0, min(self.max_linear_speed, linear_velocity))
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        
        return linear_velocity, angular_velocity

    def send_motor_commands(self, linear_vel: float, angular_vel: float):
        """
        Send motor commands to robot using OptimizedRobotController
        
        Mathematical Concept: Differential Drive Kinematics
        - Converts linear and angular velocities to left/right wheel speeds
        """
        # Differential drive kinematics - scaled down to match real movement
        wheel_base = 0.1  # meters (reduced from 0.2m to match real robot)
        wheel_radius = 0.0175  # meters (reduced from 0.05m to match real robot)
        
        # Convert to wheel speeds
        left_wheel_speed = (linear_vel - angular_vel * wheel_base / 2) / wheel_radius
        right_wheel_speed = (linear_vel + angular_vel * wheel_base / 2) / wheel_radius
        
        # Convert to PWM or motor commands - reduce scaling to match real movement
        left_pwm = int(left_wheel_speed * 75)  # Reduce scaling to match real robot movement
        right_pwm = int(right_wheel_speed * 75)
        
        # Ensure minimum PWM for effective turning
        if abs(left_pwm - right_pwm) > 20:  # Turning motion detected
            if left_pwm > right_pwm:  # Left turn
                if left_pwm > 0 and left_pwm < self.min_turn_pwm:
                    left_pwm = self.min_turn_pwm
                if right_pwm < 0 and right_pwm > -self.min_turn_pwm:
                    right_pwm = -self.min_turn_pwm
            else:  # Right turn
                if right_pwm > 0 and right_pwm < self.min_turn_pwm:
                    right_pwm = self.min_turn_pwm
                if left_pwm < 0 and left_pwm > -self.min_turn_pwm:
                    left_pwm = -self.min_turn_pwm
        
        # Limit PWM values
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))
        
        print(f"🚗 Motor Commands - Left: {left_pwm}, Right: {right_pwm} | Linear: {linear_vel:.2f}m/s, Angular: {angular_vel:.2f}rad/s")
        
        # Send commands to robot controller
        self.robot.send_motor_command(left_pwm, right_pwm)

    def navigate_to_waypoint(self, waypoint: Waypoint):
        """Set target waypoint for navigation with distance limit"""
        # Store starting position
        self.start_position = (self.pose.x, self.pose.y)
        
        # Limit waypoint distance to max travel distance
        dx = waypoint.x - self.pose.x
        dy = waypoint.y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance > self.max_travel_distance:
            # Scale down to max distance
            scale = self.max_travel_distance / distance
            limited_x = self.pose.x + dx * scale
            limited_y = self.pose.y + dy * scale
            self.target_waypoint = Waypoint(limited_x, limited_y, waypoint.tolerance)
            print(f"🎯 Limited waypoint to {self.max_travel_distance}m: ({limited_x:.2f}, {limited_y:.2f})")
        else:
            self.target_waypoint = waypoint
            print(f"🎯 Navigating to waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f})")

    def is_waypoint_reached(self) -> bool:
        """Check if current waypoint is reached"""
        if not self.target_waypoint:
            return False
        
        distance = math.sqrt(
            (self.target_waypoint.x - self.pose.x)**2 + 
            (self.target_waypoint.y - self.pose.y)**2
        )
        
        return distance < self.target_waypoint.tolerance

    def run_navigation_step(self, dt: float = 0.1):
        """Execute one navigation step with distance and obstacle checks"""
        # Update robot pose using LIDAR
        self.update_pose(dt)
        
        # Check if traveled too far from start
        travel_distance = math.sqrt(
            (self.pose.x - self.start_position[0])**2 + 
            (self.pose.y - self.start_position[1])**2
        )
        
        if travel_distance >= self.max_travel_distance:
            print(f"🛑 Maximum travel distance reached: {travel_distance:.2f}m")
            self.send_motor_commands(0, 0)  # Stop
            return True
        
        # Get current LIDAR points
        points = self.get_lidar_points()
        
        # Detect obstacles in safety zones
        obstacle_zones = self.detect_obstacles_in_zones(points)
        
        # Check for obstacles
        total_obstacles = sum(len(obstacles) for obstacles in obstacle_zones.values())
        front_obstacles = len(obstacle_zones.get('front', []))
        
        # Stop if too many front obstacles
        if front_obstacles > 5:
            print(f"🛑 Too many front obstacles: {front_obstacles}")
            self.send_motor_commands(0, 0)  # Stop
            return True
        
        # Calculate and send motor commands using SLAM navigation
        if self.target_waypoint and not self.is_waypoint_reached():
            linear_vel, angular_vel = self.calculate_slam_navigation(points)
            self.send_motor_commands(linear_vel, angular_vel)
            
            # Update SLAM with robot movement
            if hasattr(self, 'last_slam_pose'):
                dx = self.slam.robot_x - self.last_slam_pose[0]
                dy = self.slam.robot_y - self.last_slam_pose[1]
                dtheta = self.slam.robot_theta - self.last_slam_pose[2]
                if abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dtheta) > 0.05:
                    self.slam.update_robot_pose(dx, dy, dtheta)
            
            self.last_slam_pose = (self.slam.robot_x, self.slam.robot_y, self.slam.robot_theta)
            
            # Status display
            distance_to_goal = math.sqrt(
                (self.target_waypoint.x - self.pose.x)**2 + 
                (self.target_waypoint.y - self.pose.y)**2
            )
            
            print(f"📍 Pose: ({self.pose.x:.2f}, {self.pose.y:.2f}, {math.degrees(self.pose.theta):.1f}°)")
            print(f"🎯 Distance to goal: {distance_to_goal:.2f}m | Travel: {travel_distance:.2f}m | Obstacles: {total_obstacles}")
            
        elif self.is_waypoint_reached():
            print("✅ Waypoint reached!")
            self.send_motor_commands(0, 0)  # Stop
            return True
        else:
            # No target, stop
            self.send_motor_commands(0, 0)  # Stop
        
        return False

    def run_autonomous_navigation(self, waypoints: List[Waypoint]):
        """Run autonomous navigation through multiple waypoints"""
        if not self.connect_lidar():
            return
        
        print(f"🚀 Starting autonomous navigation through {len(waypoints)} waypoints")
        
        try:
            current_waypoint_idx = 0
            
            while current_waypoint_idx < len(waypoints):
                # Set current waypoint
                if not self.target_waypoint or self.is_waypoint_reached():
                    if current_waypoint_idx < len(waypoints):
                        self.navigate_to_waypoint(waypoints[current_waypoint_idx])
                        current_waypoint_idx += 1
                
                # Execute navigation step
                waypoint_reached = self.run_navigation_step()
                
                if waypoint_reached and current_waypoint_idx >= len(waypoints):
                    break
                
                time.sleep(0.1)  # 10Hz control loop
                
        except KeyboardInterrupt:
            print("\n🛑 Navigation stopped by user")
        finally:
            self.send_motor_commands(0, 0)  # Stop motors
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        self.lidar.cleanup()
        self.robot.cleanup()
        print("🧹 Navigator cleaned up")

def main():
    navigator = AutonomousNavigator()
    
    # Waypoints for bigger 3m map
    waypoints = [
        Waypoint(2.0, 1.5, tolerance=0.15),   # Move 0.5m forward from center (1.5,1.5)
        Waypoint(2.0, 2.0, tolerance=0.15),   # Move up
        Waypoint(1.0, 2.0, tolerance=0.15),   # Move left
        Waypoint(1.0, 1.0, tolerance=0.15),   # Move down
    ]
    
    print("🤖 Autonomous Navigator")
    print("Mathematical concepts demonstrated:")
    print("1. Scan Matching - for pose estimation using LIDAR")
    print("2. Principal Component Analysis - for rotation estimation")
    print("3. Potential Fields - for obstacle avoidance and goal attraction")
    print("4. Coordinate Transformations - between robot and world frames")
    print("5. Differential Drive Kinematics - for motor control")
    print()
    
    navigator.run_autonomous_navigation(waypoints)

if __name__ == "__main__":
    main()

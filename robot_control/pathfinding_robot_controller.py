#!/usr/bin/env python3
"""
Pathfinding Robot Controller
Main controller integrating map environment, pathfinding, and navigation
"""

import pygame
import sys
import time
import math
import logging
from typing import Optional, Tuple

# Import our modules
from modules.map_environment import MapEnvironment
from modules.pathfinding import AStarPathfinder, RRTPathfinder
from modules.navigation import NavigationController, NavigationState
from collision_avoidance_system import CollisionAvoidanceSystem, SafetyState

# Import LIDAR integration
try:
    from lidar_mapping.ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
    LIDAR_AVAILABLE = True
    print("✅ LIDAR direct integration available")
except ImportError:
    try:
        from lidar_mapping.pathfinding_integration import LidarEnhancedPathfindingController, integrate_lidar_with_pathfinding
        LIDAR_AVAILABLE = True
        print("✅ LIDAR enhanced integration available")
    except ImportError:
        print("⚠️  LIDAR integration not available - running without LIDAR obstacle avoidance")
        LIDAR_AVAILABLE = False

# Import the robot controller from smooth_robot_controller
try:
    from smooth_robot_controller import OptimizedRobotController
except ImportError:
    # Fallback dynamic import
    import importlib.util
    spec = importlib.util.spec_from_file_location("robot", "smooth_robot_controller.py")
    robot_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot_module)
    OptimizedRobotController = robot_module.OptimizedRobotController

class PathfindingRobotController:
    """Main controller for pathfinding robot system"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, enable_collision_avoidance=False, enable_lidar=True, lidar_port='/dev/ttyUSB1'):
        # Setup logging
        self.setup_logging()
        self.logger = logging.getLogger('PathfindingRobot')
        
        # Initialize robot controller
        self.robot = OptimizedRobotController(port, baudrate)
        
        # Start robot controller threads for Arduino communication
        if self.robot.serial_port:
            self.robot.start_threads()
            time.sleep(1)  # Let threads initialize
            self.logger.info("✅ Robot controller threads started")
        else:
            self.logger.error("❌ Failed to connect to Arduino")
        
        # Initialize collision avoidance system
        self.enable_collision_avoidance = enable_collision_avoidance
        self.collision_avoidance = None
        if enable_collision_avoidance:
            try:
                self.collision_avoidance = CollisionAvoidanceSystem(
                    camera_id=0,
                    confidence_threshold=0.4,
                    model_name='yolov8n.pt'
                )
                self.collision_avoidance.start()
                self.logger.info("✅ Collision avoidance system started")
            except Exception as e:
                self.logger.error(f"❌ Failed to start collision avoidance: {e}")
                self.collision_avoidance = None
        
        # Safety state tracking
        self.last_safety_check = time.time()
        self.safety_check_interval = 0.005  # Check every 50ms
        self.is_safety_stopped = False
        self.safety_stop_time = 0
        
        # Set robot to start at center of map (on road intersection)
        self.robot.x = 0.0  # Center X
        self.robot.y = 0.0  # Center Y  
        self.robot.theta = 0.0  # Facing right
        self.robot.path = [(0.0, 0.0)]  # Reset path
        
        # Initialize Pygame with Pi's current resolution
        pygame.init()
        
        # Auto-detect Pi's current display resolution
        display_info = pygame.display.Info()
        self.width = display_info.current_w
        self.height = display_info.current_h
        
        # Create display matching Pi's resolution
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption(f"🤖 Robot Controller ({self.width}x{self.height}) - Press ESC to exit")
        
        self.logger.info(f"🖥️  Using display resolution: {self.width}x{self.height}")
        
        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.BLUE = (0, 100, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)
        self.YELLOW = (255, 255, 0)
        self.CYAN = (0, 255, 255)
        self.MAGENTA = (255, 0, 255)
        self.LIGHT_GRAY = (200, 200, 200)
        
        # Initialize fonts (adaptive to screen resolution)
        # Base font sizes on screen height for better scaling
        base_font_size = max(10, int(self.height / 30))  # Minimum 10px, scale with height
        self.font = pygame.font.Font(None, base_font_size)
        self.small_font = pygame.font.Font(None, max(8, int(base_font_size * 0.7)))
        self.large_font = pygame.font.Font(None, int(base_font_size * 1.3))
        
        self.logger.info(f"🔤 Font sizes: large={int(base_font_size * 1.3)}, normal={base_font_size}, small={max(8, int(base_font_size * 0.7))}")
        
        # Safety colors
        self.SAFETY_GREEN = (0, 255, 0)      # Safe to move
        self.SAFETY_RED = (255, 0, 0)        # Danger - stopped
        self.SAFETY_YELLOW = (255, 255, 0)   # Checking path
        self.SAFETY_ORANGE = (255, 165, 0)   # Emergency
        
        # Calculate optimal scale based on display resolution for 3m workspace
        # Use smaller dimension to ensure map fills screen with minimal margins
        min_dimension = min(self.width, self.height)
        optimal_scale = min_dimension / 3.2  # 3m workspace fills most of screen (0.1m margin each side)
        self.logger.info(f"📏 Using map scale: {optimal_scale:.1f} pixels/meter for 3m workspace")
        
        # Initialize map environment with auto-calculated scale for bigger workspace
        self.map_env = MapEnvironment(self.width, self.height, scale=optimal_scale, workspace_size=3.0)
        
        # Initialize pathfinding algorithms
        self.astar = AStarPathfinder(self.map_env)
        self.rrt = RRTPathfinder(self.map_env)
        self.current_pathfinder = self.astar  # Default to A*
        
        # Initialize navigation controller
        self.navigator = NavigationController(self.robot, self.map_env)
        
        # Path continuation state (inspired by collision_avoidance_system.py)
        self.path_continuation_state = "safe"  # "safe", "paused", "checking"
        self.path_clear_check_count = 0
        self.required_clear_checks = 3  # Number of consecutive clear checks needed
        self.last_navigation_target = None  # Store target to resume navigation
        
        # Initialize LIDAR system directly
        self.lidar = None
        self.lidar_enhanced = None
        self.enable_lidar = enable_lidar
        if enable_lidar and LIDAR_AVAILABLE:
            try:
                # Try direct LIDAR initialization first
                if 'YDLidarX2Optimized' in globals():
                    config = LidarConfig(port=lidar_port, baudrate=115200)
                    self.lidar = YDLidarX2Optimized(config)
                    if self.lidar.connect():
                        if self.lidar.start_scanning():
                            self.logger.info("✅ LIDAR direct connection established")
                        else:
                            self.logger.error("❌ Failed to start LIDAR scanning")
                            self.lidar.cleanup()
                            self.lidar = None
                    else:
                        self.logger.error("❌ Failed to connect to LIDAR")
                        self.lidar = None
                
                # Fallback to enhanced integration
                if not self.lidar:
                    self.lidar_enhanced = integrate_lidar_with_pathfinding(
                        pathfinding_controller=self,
                        lidar_port=lidar_port,
                        enable_mapping=True
                    )
                    self.logger.info("✅ LIDAR enhanced integration initialized")
                    
            except Exception as e:
                self.logger.error(f"❌ Failed to initialize LIDAR system: {e}")
                self.lidar = None
                self.lidar_enhanced = None
                self.enable_lidar = False
        elif enable_lidar and not LIDAR_AVAILABLE:
            self.logger.warning("⚠️  LIDAR requested but not available")
            self.enable_lidar = False
        
        # LIDAR obstacle avoidance state
        self.lidar_override_active = False
        self.original_navigation_active = False
        self.last_lidar_check = time.time()
        self.lidar_check_interval = 0.05  # 20 Hz LIDAR checking
        
        # UI state
        self.selected_target: Optional[Tuple[float, float]] = None
        self.mouse_world_pos: Optional[Tuple[float, float]] = None
        self.show_help = False
        self.pathfinder_name = "A*"
        self.robot_placement_mode = False  # Mode for placing robot
        
        # Performance tracking
        self.last_frame_time = time.time()
        self.fps = 60
        
        self.logger.info("🚀 Pathfinding Robot Controller initialized")
    
    def setup_logging(self):
        """Setup logging configuration"""
        import os
        if not os.path.exists('logs'):
            os.makedirs('logs')
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('logs/pathfinding_robot.log'),
                logging.StreamHandler()
            ]
        )
    
    def check_collision_avoidance(self):
        """Check collision avoidance system and manage robot safety"""
        if not self.collision_avoidance:
            return True  # No collision avoidance - always safe
        
        try:
            current_time = time.time()
            
            # Check safety at regular intervals
            if current_time - self.last_safety_check < self.safety_check_interval:
                return not self.is_safety_stopped
            
            self.last_safety_check = current_time
            
            # Get latest safety status
            is_safe = self.collision_avoidance.is_safe_to_move()
            latest_detection = self.collision_avoidance.get_latest_detection()
            
            if not is_safe and not self.is_safety_stopped:
                # PAUSE for safety
                try:
                    self.robot.send_motor_command(0, 0)  # Stop motors
                except Exception as e:
                    self.logger.warning(f"⚠️  Motor stop failed: {e}")
                
                self.is_safety_stopped = True
                self.safety_stop_time = current_time
                
                # Pause navigation if active (don't stop completely)
                if self.navigator.is_navigating():
                    self.navigator.pause_navigation()
                
                # Log the safety pause
                if latest_detection and latest_detection.get('objects'):
                    objects = latest_detection['objects']
                    object_names = [obj['name'] for obj in objects]
                    self.logger.warning(f"🚨 SAFETY PAUSE: {', '.join(object_names)} detected!")
                else:
                    self.logger.warning(f"🚨 SAFETY PAUSE: Objects detected in path!")
                    
            elif is_safe and self.is_safety_stopped:
                # Path is clear - can resume
                self.is_safety_stopped = False
                stop_duration = current_time - self.safety_stop_time
                self.logger.info(f"✅ Path clear - resuming movement (paused for {stop_duration:.1f}s)")
                
                # Resume navigation if it was paused
                if self.navigator.is_paused():
                    self.navigator.resume_navigation()
            
            return not self.is_safety_stopped
            
        except Exception as e:
            self.logger.error(f"❌ Collision avoidance error: {e}")
            # On error, assume unsafe to be safe
            return False
    
    def apply_obstacle_avoidance(self, avoidance_command):
        """Apply LIDAR obstacle avoidance command - DISABLED FOR OVERLAY ONLY MODE"""
        left_speed, right_speed = avoidance_command
        
        # MOTOR CONTROL DISABLED - Only log the avoidance command for visualization
        # Convert to motor speeds (scale appropriately) - FOR DISPLAY ONLY
        left_motor = int(left_speed * 200)  # Scale to motor range
        right_motor = int(right_speed * 200)
        
        # Clamp to safe limits
        left_motor = max(-255, min(255, left_motor))
        right_motor = max(-255, min(255, right_motor))
        
        # Log avoidance action (NO MOTOR COMMANDS SENT)
        action_type = "BACKING" if (left_motor + right_motor) < 0 else ("TURNING" if abs(left_motor - right_motor) > 50 else "ADJUSTING")
        self.logger.info(f"🚨 LIDAR {action_type} (DISPLAY ONLY): L={left_motor}, R={right_motor}")
        
        # NOTE: Motor commands are NOT sent - this is overlay-only mode
    
    def check_lidar_obstacle_avoidance(self):
        """Check LIDAR for obstacles - OVERLAY ONLY MODE (no motor control)"""
        if not self.enable_lidar or not self.lidar_enhanced:
            return True  # No LIDAR - continue normal navigation
        
        try:
            current_time = time.time()
            
            # Check LIDAR at regular intervals
            if current_time - self.last_lidar_check < self.lidar_check_interval:
                return True  # Always allow navigation in overlay-only mode
            
            self.last_lidar_check = current_time
            
            # Check robot clearance using LIDAR (for display only)
            clearance = self.lidar_enhanced.check_robot_clearance()
            obstacle_detected = not clearance['safe']
            
            # Update obstacle state for overlay display
            if obstacle_detected:
                # Log the obstacle detection (for display only)
                closest = clearance.get('closest_obstacle')
                if closest:
                    direction = closest['direction']
                    distance = closest['distance']
                    self.logger.info(f"🚨 LIDAR OBSTACLE DETECTED: {direction.upper()} at {distance:.2f}m (DISPLAY ONLY)")
                else:
                    self.logger.info(f"🚨 LIDAR OBSTACLE DETECTED: Multiple obstacles (DISPLAY ONLY)")
            else:
                # Path is clear
                if self.lidar_enhanced.obstacle_detected:
                    self.logger.info("✅ LIDAR: Path clear (DISPLAY ONLY)")
            
            # Always allow navigation in overlay-only mode
            return True
            
        except Exception as e:
            self.logger.error(f"❌ LIDAR obstacle avoidance error: {e}")
            return True  # On error, allow normal navigation
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_h:
                    self.show_help = not self.show_help
                elif event.key == pygame.K_SPACE:
                    self.navigator.stop_navigation()
                elif event.key == pygame.K_1:
                    self.current_pathfinder = self.astar
                    self.pathfinder_name = "A*"
                    self.logger.info("🔄 Switched to A* pathfinder")
                elif event.key == pygame.K_2:
                    self.current_pathfinder = self.rrt
                    self.pathfinder_name = "RRT"
                    self.logger.info("🔄 Switched to RRT pathfinder")
                elif event.key == pygame.K_r:
                    # Reset robot position to center
                    self.robot.x = 0.0
                    self.robot.y = 0.0
                    self.robot.theta = 0.0
                    self.robot.path = [(0.0, 0.0)]
                    self.logger.info("🔄 Reset robot position")
                elif event.key == pygame.K_f:
                    # Toggle fullscreen (for testing)
                    pygame.display.toggle_fullscreen()
                elif event.key == pygame.K_p:
                    # Toggle robot placement mode
                    self.robot_placement_mode = not self.robot_placement_mode
                    if self.robot_placement_mode:
                        self.logger.info("🎯 Robot placement mode ON - Click to place robot")
                    else:
                        self.logger.info("🎯 Robot placement mode OFF")
                elif event.key == pygame.K_l:
                    # Toggle LIDAR overlay
                    if self.map_env.handle_key_press(event.key):
                        overlay_status = "ON" if self.map_env.show_lidar_overlay else "OFF"
                        self.logger.info(f"🔍 LIDAR overlay toggled: {overlay_status}")
                        print(f"🔍 LIDAR overlay is now: {overlay_status}")
                        print(f"🔍 LIDAR enabled: {self.enable_lidar}")
                        print(f"🔍 Direct LIDAR: {self.lidar is not None}")
                        print(f"🔍 Enhanced LIDAR: {self.lidar_enhanced is not None}")
                    elif self.lidar_enhanced:
                        self.lidar_enhanced.handle_enhanced_events(event)
                elif event.key == pygame.K_m:
                    # Clear LIDAR map (if available)
                    if self.lidar_enhanced:
                        self.lidar_enhanced.handle_enhanced_events(event)
                elif event.key == pygame.K_n:
                    # Save LIDAR map (if available)
                    if self.lidar_enhanced:
                        self.lidar_enhanced.handle_enhanced_events(event)
                elif event.key == pygame.K_x:
                    # Debug info
                    print(f"🐛 DEBUG INFO:")
                    print(f"   Robot position: ({self.robot.x:.3f}, {self.robot.y:.3f})")
                    print(f"   Robot on road: {self.map_env.is_point_on_road(self.robot.x, self.robot.y)}")
                    print(f"   Navigation state: {self.navigator.state}")
                    print(f"   Current path length: {len(self.navigator.current_path)}")
                    if self.mouse_world_pos:
                        print(f"   Mouse position: {self.mouse_world_pos}")
                        print(f"   Mouse on road: {self.map_env.is_point_on_road(*self.mouse_world_pos)}")
                    print(f"   Manual control active: {not self.navigator.is_navigating()}")
                    print(f"   Path continuation state: {self.path_continuation_state}")
                    print(f"   Last navigation target: {self.last_navigation_target}")
                    if self.enable_lidar:
                        print(f"   LIDAR overlay active: {getattr(self.map_env, 'show_lidar_overlay', False)}")
                        print(f"   LIDAR points: {len(getattr(self.map_env, 'lidar_points', []))}")
                        print(f"   LIDAR obstacle detected: {getattr(self.map_env, 'obstacle_detected', False)}")
                        if self.lidar:
                            print(f"   Direct LIDAR connected: True")
                        elif self.lidar_enhanced:
                            print(f"   Enhanced LIDAR connected: True")
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    self.handle_mouse_click(event.pos)
            
            elif event.type == pygame.MOUSEMOTION:
                # Update mouse world position for display
                self.mouse_world_pos = self.map_env.screen_to_world(*event.pos)
        
        # Handle keyboard input for manual control (when not navigating)
        # LIDAR override disabled in overlay-only mode
        if not self.navigator.is_navigating():
            self.handle_manual_control()
        else:
            # If navigating, check for manual override
            keys = pygame.key.get_pressed()
            if any([keys[pygame.K_w], keys[pygame.K_a], keys[pygame.K_s], keys[pygame.K_d]]):
                # Manual override - stop navigation
                self.navigator.stop_navigation()
                self.handle_manual_control()
        
        return True
    
    def handle_mouse_click(self, screen_pos):
        """Handle mouse click for target selection or robot placement"""
        world_x, world_y = self.map_env.screen_to_world(*screen_pos)
        
        if self.robot_placement_mode:
            # Robot placement mode
            if self.map_env.is_point_on_road(world_x, world_y):
                # Place robot on road
                self.robot.x = world_x
                self.robot.y = world_y
                self.robot.theta = 0.0  # Face right
                self.robot.path = [(world_x, world_y)]
                self.logger.info(f"🤖 Robot placed at: ({world_x:.3f}, {world_y:.3f})")
            else:
                # Snap to nearest road
                nearest = self.astar.find_nearest_road_point(world_x, world_y)
                self.robot.x = nearest[0]
                self.robot.y = nearest[1] 
                self.robot.theta = 0.0
                self.robot.path = [(nearest[0], nearest[1])]
                self.logger.info(f"🤖 Robot snapped to road: ({nearest[0]:.3f}, {nearest[1]:.3f})")
        else:
            # Normal navigation mode
            if self.map_env.is_point_on_road(world_x, world_y):
                self.selected_target = (world_x, world_y)
                self.logger.info(f"🎯 Target set: ({world_x:.3f}, {world_y:.3f})")
                
                # Start navigation and store target for continuation
                self.navigator.set_target(world_x, world_y, self.current_pathfinder)
                self.last_navigation_target = (world_x, world_y)  # Store for resumption
            else:
                # Find nearest road point
                nearest = self.astar.find_nearest_road_point(world_x, world_y)
                self.selected_target = nearest
                self.logger.info(f"🎯 Target snapped to road: ({nearest[0]:.3f}, {nearest[1]:.3f})")
                
                # Start navigation and store target for continuation
                self.navigator.set_target(nearest[0], nearest[1], self.current_pathfinder)
                self.last_navigation_target = nearest  # Store for resumption
    
    def handle_manual_control(self):
        """Handle manual keyboard control when not navigating - WITH COLLISION AVOIDANCE"""
        # SAFETY FIRST: Check collision avoidance
        if not self.check_collision_avoidance():
            # Safety stop active - no manual control allowed
            return
        
        # LIDAR SAFETY: Check for obstacles in safety zones
        if hasattr(self.map_env, 'is_obstacle_detected') and self.map_env.is_obstacle_detected():
            self.logger.warning("🛑 Manual control blocked: Obstacle in safety zone!")
            return  # Block manual control when obstacles detected
        
        keys = pygame.key.get_pressed()
        
        left_speed = 0
        right_speed = 0
        
        if keys[pygame.K_w]:  # Forward
            left_speed = right_speed = 150
            print("🎮 Manual: FORWARD")
        elif keys[pygame.K_s]:  # Backward
            left_speed = right_speed = -150
            print("🎮 Manual: BACKWARD")
        elif keys[pygame.K_a]:  # Turn left (SWAPPED - now turns right)
            left_speed, right_speed = 120, -120  # Swapped: was turn right command
            print("🎮 Manual: TURN RIGHT (A key)")
        elif keys[pygame.K_d]:  # Turn right (SWAPPED - now turns left) 
            left_speed, right_speed = -120, 120  # Swapped: was turn left command
            print("🎮 Manual: TURN LEFT (D key)")
        
        # Only send command if it changed (like smooth controller)
        current_command = (left_speed, right_speed)
        if not hasattr(self, 'last_manual_command') or current_command != self.last_manual_command:
            self.last_manual_command = current_command
            try:
                self.robot.send_motor_command(left_speed, right_speed)
                if left_speed != 0 or right_speed != 0:
                    print(f"🎮 Sent motor command: L={left_speed}, R={right_speed}")
            except Exception as e:
                print(f"❌ Motor command failed: {e}")
                # Don't spam errors - just skip this command
    
    def update(self):
        """Update all systems - WITH COLLISION AVOIDANCE AND LIDAR OVERLAY"""
        # SAFETY FIRST: Always check collision avoidance
        is_safe_to_move = self.check_collision_avoidance()
        
        # LIDAR OVERLAY: Update LIDAR display (no motor control)
        self.check_lidar_obstacle_avoidance()  # Just updates display
        
        # Update LIDAR data for map environment overlay
        if self.enable_lidar:
            try:
                scan = None
                
                # Try direct LIDAR first
                if self.lidar:
                    scan = self.lidar.get_latest_scan()
                # Fallback to enhanced LIDAR
                elif self.lidar_enhanced and hasattr(self.lidar_enhanced, 'lidar'):
                    scan = self.lidar_enhanced.lidar.get_latest_scan()
                
                if scan and scan.points:
                    # Update map environment with LIDAR data and robot position
                    self.map_env.update_robot_position(self.robot.x, self.robot.y, self.robot.theta)
                    self.map_env.update_lidar_data(scan.points, self.robot.x, self.robot.y, self.robot.theta)
                    
                    # Check for obstacles and handle path continuation
                    if self.map_env.is_obstacle_detected():
                        if self.path_continuation_state == "safe":
                            # First time detecting obstacle - pause navigation
                            self.logger.warning("🛑 OBSTACLE DETECTED: Pausing navigation")
                            self.robot.send_motor_command(0, 0)
                            
                            # Store current target for resumption
                            if self.navigator.is_navigating() and self.selected_target:
                                self.last_navigation_target = self.selected_target
                                self.navigator.stop_navigation()
                            
                            self.path_continuation_state = "paused"
                            self.path_clear_check_count = 0
                    else:
                        # No obstacles detected
                        if self.path_continuation_state == "paused":
                            # Start checking if path is clear
                            self.path_continuation_state = "checking"
                            self.path_clear_check_count = 1
                            self.logger.info(f"🔍 Path checking... ({self.path_clear_check_count}/{self.required_clear_checks})")
                            
                        elif self.path_continuation_state == "checking":
                            # Continue checking
                            self.path_clear_check_count += 1
                            self.logger.info(f"🔍 Path checking... ({self.path_clear_check_count}/{self.required_clear_checks})")
                            
                            if self.path_clear_check_count >= self.required_clear_checks:
                                # Path confirmed clear - resume navigation
                                self.path_continuation_state = "safe"
                                self.logger.info("✅ Path clear - resuming navigation")
                                
                                # Resume navigation to last target
                                if self.last_navigation_target and not self.navigator.is_navigating():
                                    self.logger.info(f"🔄 Resuming navigation to: {self.last_navigation_target}")
                                    self.navigator.set_target(
                                        self.last_navigation_target[0], 
                                        self.last_navigation_target[1], 
                                        self.current_pathfinder
                                    )
                else:
                    # No scan data - clear previous data
                    self.map_env.lidar_points = []
                    
            except Exception as e:
                self.logger.error(f"❌ LIDAR overlay update error: {e}")
                # Clear LIDAR data on error
                self.map_env.lidar_points = []
        
        # Robot controller runs in its own threads, no need to update manually
        
        # Update navigation based on safety and obstacle detection
        # Allow navigation only when both collision avoidance and LIDAR are clear
        obstacles_detected = self.map_env.is_obstacle_detected() if hasattr(self.map_env, 'is_obstacle_detected') else False
        
        if is_safe_to_move and not obstacles_detected and self.path_continuation_state == "safe":
            self.navigator.update()
        # Navigation is automatically paused when obstacles detected or collision avoidance active
        
        # Update performance tracking
        current_time = time.time()
        if current_time - self.last_frame_time > 0:
            self.fps = 1.0 / (current_time - self.last_frame_time)
        self.last_frame_time = current_time
    
    def draw(self):
        """Draw everything on screen"""
        # Clear screen
        self.screen.fill(self.BLACK)
        
        # Draw map environment (includes LIDAR overlay if enabled)
        self.map_env.draw(self.screen)
        
        # Draw robot path
        self.draw_robot_path()
        
        # Draw current planned path
        if self.navigator.current_path:
            self.map_env.draw_path(self.screen, self.navigator.current_path)
        
        # Draw robot
        self.draw_robot()
        
        # Draw target
        if self.selected_target:
            self.map_env.draw_target(self.screen, *self.selected_target)
        
        # Draw current waypoint (adaptive to screen resolution)
        current_waypoint = self.navigator.get_current_waypoint()
        if current_waypoint:
            waypoint_screen = self.map_env.world_to_screen(*current_waypoint)
            waypoint_size = max(4, int(min(self.width, self.height) / 60))  # Adaptive waypoint size
            pygame.draw.circle(self.screen, self.CYAN, waypoint_screen, waypoint_size)
            pygame.draw.circle(self.screen, self.WHITE, waypoint_screen, max(2, waypoint_size // 2))
        
        # Draw LIDAR overlay (if available) - now handled by map_env.draw()
        # The LIDAR overlay is automatically drawn by map_env.draw() when enabled
        
        # Draw LIDAR scan info panel (if overlay is active)
        if self.enable_lidar and hasattr(self.map_env, 'show_lidar_overlay') and self.map_env.show_lidar_overlay:
            self.draw_lidar_info_panel()
        
        # Draw UI
        self.draw_ui()
        
        # Draw help if enabled
        if self.show_help:
            self.draw_help()
        
        # Update display
        pygame.display.flip()
    
    def draw_robot_path(self):
        """Draw the robot's traveled path"""
        if len(self.robot.path) > 1:
            screen_points = []
            for x, y in self.robot.path:
                screen_x, screen_y = self.map_env.world_to_screen(x, y)
                if 0 <= screen_x <= self.width and 0 <= screen_y <= self.height:
                    screen_points.append((screen_x, screen_y))
            
            if len(screen_points) > 1:
                pygame.draw.lines(self.screen, self.GREEN, False, screen_points, 2)
    
    def draw_robot(self):
        """Draw the robot with enhanced visualization (adaptive to screen resolution)"""
        robot_x, robot_y = self.map_env.world_to_screen(self.robot.x, self.robot.y)
        
        # Robot body (adaptive sizing based on screen resolution)
        robot_size = max(10, int(min(self.width, self.height) / 25))  # Scale with screen size, min 10px
        cos_theta = math.cos(self.robot.theta)
        sin_theta = math.sin(self.robot.theta)
        
        # Robot corners
        corners = []
        for dx, dy in [(-robot_size//2, -robot_size//3), (robot_size//2, -robot_size//3),
                      (robot_size//2, robot_size//3), (-robot_size//2, robot_size//3)]:
            rotated_x = dx * cos_theta - dy * sin_theta
            rotated_y = dx * sin_theta + dy * cos_theta
            corners.append((robot_x + rotated_x, robot_y - rotated_y))
        
        pygame.draw.polygon(self.screen, self.BLUE, corners)
        
        # Direction arrow
        arrow_length = robot_size * 1.3
        arrow_end_x = robot_x + arrow_length * cos_theta
        arrow_end_y = robot_y - arrow_length * sin_theta
        pygame.draw.line(self.screen, self.YELLOW, (robot_x, robot_y), 
                        (arrow_end_x, arrow_end_y), 4)
        
        # Navigation status indicator
        nav_info = self.navigator.get_navigation_info()
        if nav_info['state'] == 'navigating':
            # Green ring when navigating
            pygame.draw.circle(self.screen, self.GREEN, (robot_x, robot_y), robot_size + 5, 3)
        elif nav_info['state'] == 'reached_goal':
            # Cyan ring when reached goal
            pygame.draw.circle(self.screen, self.CYAN, (robot_x, robot_y), robot_size + 5, 3)
        
        # Center dot
        pygame.draw.circle(self.screen, self.WHITE, (robot_x, robot_y), 3)
    
    def draw_ui(self):
        """Draw user interface elements (minimal to avoid road interference)"""
        # Much smaller UI panel - only essential info
        ui_width = min(80, int(self.width * 0.25))   # Much smaller: max 80px or 25% width
        ui_height = min(60, int(self.height * 0.15)) # Much smaller: max 60px or 15% height
        ui_rect = pygame.Rect(3, 3, ui_width, ui_height)
        pygame.draw.rect(self.screen, (0, 0, 0, 180), ui_rect)  # More transparent
        pygame.draw.rect(self.screen, self.WHITE, ui_rect, 1)
        
        y_offset = 5
        
        # Minimal robot status (very compact)
        status_lines = [
            f"({self.robot.x:.1f},{self.robot.y:.1f})",
            f"{math.degrees(self.robot.theta):.0f}°",
        ]
        
        for line in status_lines:
            text = self.small_font.render(line, True, self.WHITE)
            self.screen.blit(text, (5, y_offset))
            y_offset += 10
        
        # Navigation status (ultra compact)
        nav_info = self.navigator.get_navigation_info()
        if nav_info['state'] == 'navigating':
            nav_text = f"→{self.pathfinder_name}"
            color = self.GREEN
        else:
            nav_text = nav_info['state'][:4].upper()
            color = self.WHITE
            
        text = self.small_font.render(nav_text, True, color)
        self.screen.blit(text, (5, y_offset))
        
        # Robot placement mode indicator (if active)
        if self.robot_placement_mode:
            placement_text = self.small_font.render("🎯", True, self.MAGENTA)
            self.screen.blit(placement_text, (ui_width - 15, 5))
        
        # Mouse position - bottom left (very compact)
        if self.mouse_world_pos:
            mouse_text = f"({self.mouse_world_pos[0]:.1f},{self.mouse_world_pos[1]:.1f})"
            on_road = "✓" if self.map_env.is_point_on_road(*self.mouse_world_pos) else "✗"
            mouse_color = self.GREEN if "✓" in on_road else self.RED
            
            # Single line at bottom
            combined_text = f"{mouse_text} {on_road}"
            mouse_surface = self.small_font.render(combined_text, True, mouse_color)
            self.screen.blit(mouse_surface, (3, self.height - 12))
        
        # COLLISION AVOIDANCE STATUS - top right corner (minimal)
        status_x = self.width - 12
        if self.collision_avoidance:
            status = self.collision_avoidance.get_safety_status()
            state = status['state']
            
            if state == 'safe':
                safety_text = "●"
                safety_color = self.SAFETY_GREEN
            elif state == 'danger':
                safety_text = "●"
                safety_color = self.SAFETY_RED
            else:
                safety_text = "●"
                safety_color = self.SAFETY_YELLOW
            
            # Very small indicator in corner
            safety_surface = self.small_font.render(safety_text, True, safety_color)
            self.screen.blit(safety_surface, (status_x, 3))
            status_x -= 15  # Move next indicator left
        
        # LIDAR STATUS - next to collision avoidance indicator
        if self.enable_lidar:
            # Check if LIDAR overlay is active and working
            if hasattr(self.map_env, 'show_lidar_overlay') and self.map_env.show_lidar_overlay:
                if hasattr(self.map_env, 'is_obstacle_detected') and self.map_env.is_obstacle_detected():
                    lidar_text = "◆"  # Diamond for obstacle detected
                    lidar_color = self.SAFETY_RED  # Red for danger
                elif len(self.map_env.lidar_points) > 0:
                    lidar_text = "◆"  # Diamond for scanning
                    lidar_color = self.SAFETY_GREEN
                else:
                    lidar_text = "◆"  # Diamond for no data
                    lidar_color = self.SAFETY_YELLOW
            elif self.lidar_enhanced and hasattr(self.lidar_enhanced, 'lidar_running'):
                if self.lidar_enhanced.lidar_running:
                    lidar_text = "◇"  # Hollow diamond for LIDAR available but overlay off
                    lidar_color = self.LIGHT_GRAY
                else:
                    lidar_text = "◆"  # Diamond for disconnected
                    lidar_color = self.SAFETY_RED
            else:
                lidar_text = "◇"  # Hollow diamond for LIDAR disabled
                lidar_color = self.LIGHT_GRAY
            
            lidar_surface = self.small_font.render(lidar_text, True, lidar_color)
            self.screen.blit(lidar_surface, (status_x, 3))
        
        # Performance and help info - bottom right (minimal)
        fps_text = self.small_font.render(f"{self.fps:.0f}", True, self.WHITE)
        self.screen.blit(fps_text, (self.width - 20, self.height - 12))
        
        help_text = self.small_font.render("H", True, self.LIGHT_GRAY)
        self.screen.blit(help_text, (self.width - 30, self.height - 12))
    
    def draw_help(self):
        """Draw help overlay"""
        # Semi-transparent background
        help_surface = pygame.Surface((self.width, self.height))
        help_surface.set_alpha(200)
        help_surface.fill((0, 0, 0))
        self.screen.blit(help_surface, (0, 0))
        
        # Help content
        help_lines = [
            "🤖 PATHFINDING ROBOT CONTROLLER HELP",
            "",
            "MOUSE CONTROLS:",
            "  • Left Click: Set navigation target",
            "  • Click on roads or system will snap to nearest road",
            "",
            "KEYBOARD CONTROLS:",
            "  • W/A/S/D: Manual control (when not navigating)",
            "  • SPACE: Stop current navigation",
            "  • P: Toggle robot placement mode",
            "  • 1: Switch to A* pathfinding algorithm",
            "  • 2: Switch to RRT pathfinding algorithm", 
            "  • R: Reset robot position to center (0,0)",
            "  • F: Toggle fullscreen mode",
            "  • L: Toggle LIDAR overlay display with scan info",
            "  • M: Clear LIDAR map",
            "  • N: Save LIDAR map",
            "  • X: Show debug information",
            "  • H: Toggle this help screen",
            "  • ESC: Exit application",
            "",
            "NAVIGATION:",
            "  • Green ring around robot: Currently navigating",
            "  • Cyan ring: Reached target destination",
            "  • Yellow line: Planned path to target",
            "  • Green line: Robot's traveled path",
            "  • Cyan circles: Current waypoint",
            "",
            "MAP ELEMENTS:",
            "  • Dark gray: Roads (navigable areas)",
            "  • Brown lines: Road boundaries",
            "  • Red circles: Obstacles",
            "  • Green areas: Grass (non-navigable)",
            "",
            "LIDAR SYSTEM:",
            "  • ◆ Green: LIDAR scanning, path clear",
            "  • ◆ Red: Obstacle detected in safety zone",
            "  • ◆ Yellow: LIDAR running but no data",
            "  • ◇ Gray: LIDAR available but overlay disabled",
            "  • Safety zones: 30cm front, 25cm sides, 20cm back",
            "  • Real-time scan info panel shows point counts",
            "  • Emergency stop when obstacles enter safety zones",
            "",
            "Press 'H' again to close help"
        ]
        
        y_start = 50
        for i, line in enumerate(help_lines):
            if line.startswith("🤖"):
                color = self.CYAN
                font = self.large_font
            elif line.endswith(":"):
                color = self.YELLOW
                font = self.font
            elif line.startswith("  •"):
                color = self.WHITE
                font = self.small_font
            else:
                color = self.LIGHT_GRAY
                font = self.small_font
            
            text = font.render(line, True, color)
            self.screen.blit(text, (50, y_start + i * 22))
    
    def draw_lidar_info_panel(self):
        """Draw simplified LIDAR obstacle detection panel"""
        # Smaller panel dimensions (right side top)
        panel_width = 150
        panel_height = 60
        panel_x = self.width - panel_width - 10
        panel_y = 10
        
        # Semi-transparent background
        panel_surface = pygame.Surface((panel_width, panel_height))
        panel_surface.set_alpha(200)
        panel_surface.fill((0, 0, 0))
        self.screen.blit(panel_surface, (panel_x, panel_y))
        
        # Border
        pygame.draw.rect(self.screen, self.WHITE, (panel_x, panel_y, panel_width, panel_height), 2)
        
        y_offset = panel_y + 5
        
        # Simple obstacle detection status
        if hasattr(self.map_env, 'lidar_points') and self.map_env.lidar_points:
            if self.map_env.obstacle_detected:
                # Obstacle detected - find which side
                obstacle_side = self._get_obstacle_side()
                status_text = "OBSTACLE FOUND"
                side_text = f"Side: {obstacle_side}"
                status_color = self.SAFETY_RED
            else:
                status_text = "PATH CLEAR"
                side_text = "All sides safe"
                status_color = self.SAFETY_GREEN
            
            # Draw status
            status_surface = self.font.render(status_text, True, status_color)
            self.screen.blit(status_surface, (panel_x + 5, y_offset))
            y_offset += 20
            
            # Draw side info
            side_surface = self.small_font.render(side_text, True, self.WHITE)
            self.screen.blit(side_surface, (panel_x + 5, y_offset))
            
        else:
            # No LIDAR data
            no_data_text = self.small_font.render("No LIDAR data", True, self.SAFETY_RED)
            self.screen.blit(no_data_text, (panel_x + 5, y_offset))
    
    def _get_obstacle_side(self):
        """Determine which side has obstacles"""
        if not hasattr(self.map_env, 'lidar_points') or not self.map_env.lidar_points:
            return "Unknown"
        
        # Safety distances from lidar_cartesian_plot.py
        front_distance = 0.30   # 30cm front
        side_distance = 0.25    # 25cm left/right
        back_distance = 0.20    # 20cm back
        
        # Count obstacles in each zone
        front_obstacles = 0
        left_obstacles = 0
        right_obstacles = 0
        back_obstacles = 0
        
        for point_data in self.map_env.lidar_points:
            if len(point_data) >= 5:  # Has local_x, local_y data
                local_x, local_y = point_data[3], point_data[4]
                
                # Check which safety zone this point is in
                # Front zone (x > 0, within side distance)
                if local_x > 0 and local_x <= front_distance and abs(local_y) <= side_distance:
                    front_obstacles += 1
                # Back zone (x < 0, within side distance)  
                elif local_x < 0 and local_x >= -back_distance and abs(local_y) <= side_distance:
                    back_obstacles += 1
                # Left zone (y > 0, within front/back range)
                elif local_y > 0 and local_y <= side_distance and -back_distance <= local_x <= front_distance:
                    left_obstacles += 1
                # Right zone (y < 0, within front/back range)
                elif local_y < 0 and local_y >= -side_distance and -back_distance <= local_x <= front_distance:
                    right_obstacles += 1
        
        # Determine primary obstacle direction
        obstacles = {
            "Front": front_obstacles,
            "Left": left_obstacles, 
            "Right": right_obstacles,
            "Back": back_obstacles
        }
        
        if max(obstacles.values()) == 0:
            return "None"
        
        # Return the side with most obstacles
        primary_side = max(obstacles, key=obstacles.get)
        
        # If multiple sides, show combination
        active_sides = [side for side, count in obstacles.items() if count > 0]
        if len(active_sides) > 1:
            return f"{primary_side} + {len(active_sides)-1} more"
        else:
            return primary_side
    
    def run(self):
        """Main application loop"""
        clock = pygame.time.Clock()
        
        self.logger.info("🚀 Starting pathfinding robot controller...")
        
        try:
            while True:
                # Handle events
                if not self.handle_events():
                    break
                
                # Update systems
                self.update()
                
                # Draw everything
                self.draw()
                
                # Control frame rate
                clock.tick(60)
        
        except KeyboardInterrupt:
            self.logger.info("👋 Received keyboard interrupt")
        
        except Exception as e:
            self.logger.error(f"❌ Application error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.logger.info("🧹 Cleaning up...")
        
        # Stop LIDAR system
        if self.lidar:
            self.lidar.cleanup()
            self.logger.info("✅ Direct LIDAR system stopped")
        elif self.enable_lidar and self.lidar_enhanced:
            self.lidar_enhanced.cleanup()
            self.logger.info("✅ Enhanced LIDAR system stopped")
        
        # Stop collision avoidance system
        if self.collision_avoidance:
            self.collision_avoidance.stop()
            self.logger.info("✅ Collision avoidance system stopped")
        
        # Stop navigation
        self.navigator.stop_navigation()
        
        # Stop robot
        self.robot.cleanup()
        
        # Quit pygame
        pygame.quit()
        
        self.logger.info("👋 Pathfinding robot controller stopped")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Pathfinding Robot Controller')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB0', 
                       help='Arduino serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                       help='Serial baudrate (default: 115200)')
    parser.add_argument('--lidar-port', default='/dev/ttyUSB1',
                       help='LIDAR serial port (default: /dev/ttyUSB1)')
    parser.add_argument('--no-lidar', action='store_true',
                       help='Disable LIDAR obstacle avoidance')
    parser.add_argument('--enable-collision-avoidance', action='store_true',
                       help='Enable camera-based collision avoidance')
    
    args = parser.parse_args()
    
    print(f"🔌 Using Arduino port: {args.port}")
    print(f"📡 Using baudrate: {args.baudrate}")
    if not args.no_lidar:
        print(f"📡 Using LIDAR port: {args.lidar_port}")
    else:
        print("⚠️  LIDAR disabled")
    
    controller = PathfindingRobotController(
        port=args.port, 
        baudrate=args.baudrate,
        enable_collision_avoidance=args.enable_collision_avoidance,
        enable_lidar=not args.no_lidar,
        lidar_port=args.lidar_port
    )
    controller.run()

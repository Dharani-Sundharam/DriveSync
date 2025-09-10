#!/usr/bin/env python3
"""
Console LIDAR Obstacle Avoidance Test
=====================================

A console-only test that demonstrates LIDAR obstacle detection and avoidance
without any GUI components that might cause issues.
"""

import time
import math
import threading
import logging
import sys

# Handle both relative and absolute imports
try:
    from .ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
    from .pathfinding_integration import LidarEnhancedPathfindingController
except ImportError:
    from ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
    from pathfinding_integration import LidarEnhancedPathfindingController


class MockRobot:
    """Mock robot for testing"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Movement state
        self.forward_speed = 0.0
        self.angular_speed = 0.0
        self.last_update = time.time()
    
    def apply_movement(self, forward_speed, angular_speed):
        """Apply movement commands"""
        self.forward_speed = forward_speed
        self.angular_speed = angular_speed
    
    def update_position(self):
        """Update robot position based on current movement"""
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        
        if dt > 0.1:  # Limit large time steps
            dt = 0.1
        
        # Update position
        self.x += self.forward_speed * math.cos(self.theta) * dt
        self.y += self.forward_speed * math.sin(self.theta) * dt
        self.theta += self.angular_speed * dt
        
        # Normalize angle
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi


class MockPathfindingController:
    """Mock pathfinding controller for testing"""
    
    def __init__(self):
        self.robot = MockRobot()
        self.width = 800
        self.height = 600
        
        # Mock fonts (not used in console mode)
        self.small_font = None
        self.font = None
        
        self.running = True
    
    def apply_obstacle_avoidance(self, avoidance_command):
        """Apply obstacle avoidance command to robot"""
        left_speed, right_speed = avoidance_command
        
        # Convert to robot movement
        forward_speed = (left_speed + right_speed) / 2
        angular_speed = (right_speed - left_speed) / 2
        
        # Apply to mock robot
        self.robot.apply_movement(forward_speed, angular_speed)
        
        # Print avoidance action
        action_type = "BACKING UP" if forward_speed < 0 else ("TURNING" if abs(angular_speed) > 0.1 else "MOVING")
        print(f"🚨 {action_type}: Forward={forward_speed:.2f} m/s, Turn={math.degrees(angular_speed):.1f}°/s")


class ConsoleLidarTest:
    """Console-only LIDAR test"""
    
    def __init__(self):
        # Create mock controller
        self.mock_controller = MockPathfindingController()
        
        # Create LIDAR-enhanced controller
        print("🚀 Initializing LIDAR-enhanced pathfinding controller...")
        self.enhanced_controller = LidarEnhancedPathfindingController(
            pathfinding_controller=self.mock_controller,
            lidar_port='/dev/ttyUSB1',
            enable_lidar_mapping=True
        )
        
        # Test state
        self.running = True
        self.last_status_update = time.time()
        
        # Setup logging
        logging.basicConfig(level=logging.INFO, 
                          format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger('ConsoleLidarTest')
        
        # Robot update thread
        self.robot_thread = threading.Thread(target=self.robot_update_loop, daemon=True)
        self.robot_thread.start()
        
        # Status display thread
        self.status_thread = threading.Thread(target=self.status_display_loop, daemon=True)
        self.status_thread.start()
    
    def robot_update_loop(self):
        """Update robot position continuously"""
        while self.running:
            try:
                self.mock_controller.robot.update_position()
                time.sleep(0.05)  # 20 Hz update
            except Exception as e:
                self.logger.error(f"Robot update error: {e}")
                time.sleep(0.1)
    
    def status_display_loop(self):
        """Display status information periodically"""
        while self.running:
            try:
                current_time = time.time()
                if current_time - self.last_status_update >= 2.0:  # Every 2 seconds
                    self.display_status()
                    self.last_status_update = current_time
                time.sleep(0.5)
            except Exception as e:
                self.logger.error(f"Status display error: {e}")
                time.sleep(1.0)
    
    def display_status(self):
        """Display current robot and LIDAR status"""
        robot = self.mock_controller.robot
        
        print("\n" + "=" * 80)
        print(f"🤖 ROBOT STATUS - {time.strftime('%H:%M:%S')}")
        print("=" * 80)
        
        # Robot position
        print(f"📍 Position: X={robot.x:.2f}m, Y={robot.y:.2f}m, θ={math.degrees(robot.theta):.1f}°")
        print(f"🎯 Movement: Forward={robot.forward_speed:.2f}m/s, Angular={math.degrees(robot.angular_speed):.1f}°/s")
        
        # LIDAR status
        if self.enhanced_controller.lidar and self.enhanced_controller.lidar_running:
            print("📡 LIDAR: ✅ CONNECTED & SCANNING")
            
            # Get LIDAR statistics
            lidar_stats = self.enhanced_controller.lidar.get_statistics()
            if lidar_stats:
                print(f"📊 Scan Rate: {lidar_stats.get('scan_frequency', 0):.1f} Hz")
        else:
            print("📡 LIDAR: ❌ DISCONNECTED")
        
        # Obstacle detection
        if self.enhanced_controller.obstacle_detected:
            print(f"⚠️  OBSTACLE DETECTED: {self.enhanced_controller.obstacle_direction or 'UNKNOWN DIRECTION'}")
            
            # Get clearance information
            clearance = self.enhanced_controller.check_robot_clearance()
            if clearance.get('obstacle_distances'):
                distances = clearance['obstacle_distances']
                print(f"📏 Clearances:")
                print(f"   • Front: {distances['front']:.2f}m (req: {self.enhanced_controller.clearance_front + self.enhanced_controller.lidar_to_front:.2f}m)")
                print(f"   • Left:  {distances['left']:.2f}m (req: {self.enhanced_controller.clearance_sides + self.enhanced_controller.robot_width/2:.2f}m)")
                print(f"   • Right: {distances['right']:.2f}m (req: {self.enhanced_controller.clearance_sides + self.enhanced_controller.robot_width/2:.2f}m)")
                print(f"   • Back:  {distances['back']:.2f}m (req: {self.enhanced_controller.clearance_back + self.enhanced_controller.lidar_to_back:.2f}m)")
            
            if self.enhanced_controller.avoidance_active:
                print("🔄 AVOIDANCE: ACTIVE")
            else:
                print("🔄 AVOIDANCE: STANDBY")
        else:
            print("✅ PATH CLEAR - No obstacles detected")
        
        # Robot specifications
        print(f"🔧 Robot Specs: Length={self.enhanced_controller.robot_length*1000:.0f}mm, "
              f"LIDAR-to-Front={self.enhanced_controller.lidar_to_front*1000:.0f}mm")
        
        print("=" * 80)
    
    def simulate_forward_movement(self):
        """Simulate robot moving forward to trigger obstacle avoidance"""
        print("🎮 Simulating forward movement...")
        self.mock_controller.robot.apply_movement(0.5, 0.0)  # Move forward at 0.5 m/s
    
    def run(self):
        """Run the console test"""
        print("\n" + "=" * 80)
        print("🤖 CONSOLE LIDAR OBSTACLE AVOIDANCE TEST")
        print("=" * 80)
        print("📋 Robot Specifications:")
        print(f"   • Total Length: {self.enhanced_controller.robot_length*1000:.0f}mm")
        print(f"   • LIDAR to Front: {self.enhanced_controller.lidar_to_front*1000:.0f}mm")
        print(f"   • Required Clearances:")
        print(f"     - Front: {self.enhanced_controller.clearance_front*100:.0f}cm")
        print(f"     - Sides: {self.enhanced_controller.clearance_sides*100:.0f}cm")
        print(f"     - Back: {self.enhanced_controller.clearance_back*100:.0f}cm")
        print("=" * 80)
        
        # Try to start LIDAR
        print("📡 Connecting to LIDAR...")
        if self.enhanced_controller.start_lidar_mapping():
            print("✅ LIDAR connected successfully at /dev/ttyUSB1 (115200 baud)")
        else:
            print("⚠️  LIDAR connection failed - check connection and permissions")
            print("   Running in simulation mode for demonstration")
        
        print("\n🎮 Test Controls:")
        print("   • Press ENTER to simulate forward movement")
        print("   • Type 'status' to show detailed status")
        print("   • Type 'quit' or Ctrl+C to exit")
        print("\n🔄 Starting test loop...")
        
        # Simulate some movement to trigger obstacle detection
        self.simulate_forward_movement()
        
        try:
            while self.running:
                try:
                    # Get user input (non-blocking would be better, but this works for demo)
                    user_input = input("\n> ").strip().lower()
                    
                    if user_input in ['quit', 'exit', 'q']:
                        break
                    elif user_input == 'status':
                        self.display_status()
                    elif user_input == 'forward' or user_input == '':
                        self.simulate_forward_movement()
                    elif user_input == 'stop':
                        self.mock_controller.robot.apply_movement(0.0, 0.0)
                        print("🛑 Robot stopped")
                    elif user_input == 'left':
                        self.mock_controller.robot.apply_movement(0.0, 1.0)
                        print("↺ Turning left")
                    elif user_input == 'right':
                        self.mock_controller.robot.apply_movement(0.0, -1.0)
                        print("↻ Turning right")
                    elif user_input == 'help':
                        print("Available commands: forward, stop, left, right, status, quit")
                    else:
                        print("Unknown command. Type 'help' for available commands.")
                        
                except EOFError:
                    break
                except KeyboardInterrupt:
                    break
                    
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted by user")
        
        # Cleanup
        print("\n🧹 Shutting down...")
        self.running = False
        self.enhanced_controller.cleanup()
        print("✅ Test completed successfully")


def main():
    """Main entry point"""
    try:
        test = ConsoleLidarTest()
        test.run()
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("👋 Goodbye!")


if __name__ == "__main__":
    main()

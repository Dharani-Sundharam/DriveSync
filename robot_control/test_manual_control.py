#!/usr/bin/env python3
"""
Test manual control with collision avoidance
"""

import pygame
import time
from pathfinding_robot_controller import PathfindingRobotController

def main():
    print("🎮 Testing Manual Control with Collision Avoidance")
    print("=" * 60)
    print("🎯 Instructions:")
    print("   W - Move forward")
    print("   S - Move backward") 
    print("   A - Turn right")
    print("   D - Turn left")
    print("   ESC - Quit")
    print("   👋 Wave hand in front of camera to test emergency stop")
    print("-" * 60)
    
    try:
        # Create robot controller
        controller = PathfindingRobotController(enable_collision_avoidance=True)
        
        # Simple manual control loop
        clock = pygame.time.Clock()
        running = True
        
        while running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            # Check collision avoidance
            is_safe = controller.check_collision_avoidance()
            
            # Handle manual control only if safe
            if is_safe:
                keys = pygame.key.get_pressed()
                
                if keys[pygame.K_w]:
                    print("🎮 Forward command sent")
                    controller.robot.send_motor_command(100, 100)
                elif keys[pygame.K_s]:
                    print("🎮 Backward command sent")
                    controller.robot.send_motor_command(-100, -100)
                elif keys[pygame.K_a]:
                    print("🎮 Turn right command sent")
                    controller.robot.send_motor_command(100, -100)
                elif keys[pygame.K_d]:
                    print("🎮 Turn left command sent")
                    controller.robot.send_motor_command(-100, 100)
                else:
                    controller.robot.send_motor_command(0, 0)
            else:
                print("🚨 BLOCKED: Objects detected - manual control disabled")
                controller.robot.send_motor_command(0, 0)
            
            clock.tick(10)  # 10 FPS
            
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'controller' in locals():
            controller.cleanup()
        print("✅ Test complete")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Debug script to test collision avoidance stop/resume behavior
"""

import time
from collision_avoidance_system import CollisionAvoidanceSystem

def main():
    print("🚀 Testing Collision Avoidance Stop/Resume Behavior")
    print("=" * 60)
    
    # Create collision avoidance system
    cas = CollisionAvoidanceSystem(camera_id=0, confidence_threshold=0.4)
    
    try:
        cas.start()
        print("✅ Collision avoidance started")
        print("👋 Wave your hand in front of camera to test")
        print("🎮 Press Ctrl+C to stop")
        print("-" * 60)
        
        last_state = None
        
        for i in range(300):  # Run for 30 seconds (100ms intervals)
            # Get current safety status
            is_safe = cas.is_safe_to_move()
            status = cas.get_safety_status()
            latest = cas.get_latest_detection()
            
            current_state = status['state']
            
            # Only print when state changes or every 5 seconds
            if current_state != last_state or i % 50 == 0:
                objects_count = 0
                object_names = []
                
                if latest and latest.get('objects'):
                    objects = latest['objects']
                    objects_count = len(objects)
                    object_names = [obj['name'] for obj in objects]
                
                print(f"[{i*0.1:05.1f}s] State: {current_state.upper():8} | Safe: {is_safe:5} | Objects: {objects_count:2} | Clear checks: {status['clear_check_count']}/{status['required_checks']}")
                
                if object_names:
                    print(f"         🎯 Detected: {', '.join(object_names)}")
                
                if current_state != last_state:
                    if current_state == 'danger':
                        print("         🚨 EMERGENCY STOP!")
                    elif current_state == 'checking':
                        print("         🔍 Checking if path is clear...")
                    elif current_state == 'safe':
                        print("         ✅ SAFE TO MOVE!")
                
                last_state = current_state
            
            time.sleep(0.1)  # 100ms intervals
            
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    finally:
        cas.stop()
        print("✅ Collision avoidance stopped")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Quick Test: Try Different Encoder Mappings
"""

import sys
import os

def test_encoder_mapping(mapping_type):
    print(f"🧪 TESTING ENCODER MAPPING: {mapping_type}")
    print("=" * 50)
    
    if mapping_type == "original":
        # Original mapping (before any fixes)
        print("Reverting to ORIGINAL encoder mapping...")
        print("  delta_left = encoder_a")
        print("  delta_right = encoder_b")
        
        # Fix smooth_robot_controller.py
        with open("smooth_robot_controller.py", "r") as f:
            content = f.read()
        
        content = content.replace(
            "delta_left = new_encoder_b - self.last_encoder_b   # Left wheel (Motor B due to firmware swap)",
            "delta_left = new_encoder_a - self.last_encoder_a   # Left wheel (Motor A - ORIGINAL)"
        ).replace(
            "delta_right = new_encoder_a - self.last_encoder_a  # Right wheel (Motor A due to firmware swap)",
            "delta_right = new_encoder_b - self.last_encoder_b  # Right wheel (Motor B - ORIGINAL)"
        )
        
        with open("smooth_robot_controller.py", "w") as f:
            f.write(content)
            
        # Fix robot_controller.py
        with open("robot_controller.py", "r") as f:
            content = f.read()
        
        content = content.replace(
            "distance_left = delta_b * self.MOTOR_B_METERS_PER_TICK   # Left wheel (now encoder B)",
            "distance_left = delta_a * self.MOTOR_A_METERS_PER_TICK   # Left wheel (ORIGINAL)"
        ).replace(
            "distance_right = delta_a * self.MOTOR_A_METERS_PER_TICK  # Right wheel (now encoder A)",
            "distance_right = delta_b * self.MOTOR_B_METERS_PER_TICK  # Right wheel (ORIGINAL)"
        )
        
        with open("robot_controller.py", "w") as f:
            f.write(content)
            
    elif mapping_type == "negated":
        print("Testing NEGATED encoder mapping...")
        print("  delta_left = -encoder_b")
        print("  delta_right = -encoder_a")
        
        # This would require more complex changes - let's just document it
        print("❌ Complex change - use debug tool first")
        return False
        
    print("✅ Encoder mapping changed!")
    print("Test with: ./run_pathfinding_robot.sh")
    print("")
    print("If this doesn't work, run:")
    print("  python3 test_encoder_swap.py revert")
    return True

def revert_changes():
    print("🔄 REVERTING TO SWAPPED ENCODER MAPPING")
    print("=" * 40)
    
    # Revert smooth_robot_controller.py
    with open("smooth_robot_controller.py", "r") as f:
        content = f.read()
    
    content = content.replace(
        "delta_left = new_encoder_a - self.last_encoder_a   # Left wheel (Motor A - ORIGINAL)",
        "delta_left = new_encoder_b - self.last_encoder_b   # Left wheel (Motor B due to firmware swap)"
    ).replace(
        "delta_right = new_encoder_b - self.last_encoder_b  # Right wheel (Motor B - ORIGINAL)",
        "delta_right = new_encoder_a - self.last_encoder_a  # Right wheel (Motor A due to firmware swap)"
    )
    
    with open("smooth_robot_controller.py", "w") as f:
        f.write(content)
        
    # Revert robot_controller.py
    with open("robot_controller.py", "r") as f:
        content = f.read()
    
    content = content.replace(
        "distance_left = delta_a * self.MOTOR_A_METERS_PER_TICK   # Left wheel (ORIGINAL)",
        "distance_left = delta_b * self.MOTOR_B_METERS_PER_TICK   # Left wheel (now encoder B)"
    ).replace(
        "distance_right = delta_b * self.MOTOR_B_METERS_PER_TICK  # Right wheel (ORIGINAL)",
        "distance_right = delta_a * self.MOTOR_A_METERS_PER_TICK  # Right wheel (now encoder A)"
    )
    
    with open("robot_controller.py", "w") as f:
        f.write(content)
        
    print("✅ Reverted to swapped mapping")

if __name__ == "__main__":
    os.chdir("/home/dharani/Desktop/DriveSync/robot_control")
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "revert":
            revert_changes()
        else:
            test_encoder_mapping(sys.argv[1])
    else:
        print("🔧 ENCODER MAPPING TEST TOOL")
        print("===========================")
        print("")
        print("Usage:")
        print("  python3 test_encoder_swap.py original  # Try original encoder mapping")
        print("  python3 test_encoder_swap.py revert    # Go back to swapped mapping")
        print("")
        print("OR better yet, run the debug tool first:")
        print("  python3 debug_odometry.py")


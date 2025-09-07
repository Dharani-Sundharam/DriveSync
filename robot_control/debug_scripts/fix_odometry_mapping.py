#!/usr/bin/env python3
"""
Fix Odometry Mapping Based on Test Results
"""

import sys

def fix_odometry_mapping(encoder_mapping):
    """
    Fix the odometry mapping based on test results
    
    encoder_mapping examples:
    "A=LEFT,B=RIGHT" - Encoder A tracks left wheel, B tracks right wheel
    "A=RIGHT,B=LEFT" - Encoder A tracks right wheel, B tracks left wheel
    """
    
    print(f"🔧 FIXING ODOMETRY MAPPING: {encoder_mapping}")
    print("=" * 50)
    
    if encoder_mapping == "A=LEFT,B=RIGHT":
        print("Setting: Encoder A = Left wheel, Encoder B = Right wheel")
        
        # Fix smooth_robot_controller.py
        with open("smooth_robot_controller.py", "r") as f:
            content = f.read()
        
        # Change to A=left, B=right
        content = content.replace(
            "delta_left = new_encoder_b - self.last_encoder_b   # Left wheel (Motor B due to firmware swap)",
            "delta_left = new_encoder_a - self.last_encoder_a   # Left wheel (Encoder A)"
        ).replace(
            "delta_right = new_encoder_a - self.last_encoder_a  # Right wheel (Motor A due to firmware swap)",
            "delta_right = new_encoder_b - self.last_encoder_b  # Right wheel (Encoder B)"
        )
        
        with open("smooth_robot_controller.py", "w") as f:
            f.write(content)
            
        # Fix robot_controller.py
        with open("robot_controller.py", "r") as f:
            content = f.read()
        
        content = content.replace(
            "distance_left = delta_b * self.MOTOR_B_METERS_PER_TICK   # Left wheel (now encoder B)",
            "distance_left = delta_a * self.MOTOR_A_METERS_PER_TICK   # Left wheel (Encoder A)"
        ).replace(
            "distance_right = delta_a * self.MOTOR_A_METERS_PER_TICK  # Right wheel (now encoder A)",
            "distance_right = delta_b * self.MOTOR_B_METERS_PER_TICK  # Right wheel (Encoder B)"
        )
        
        with open("robot_controller.py", "w") as f:
            f.write(content)
            
    elif encoder_mapping == "A=RIGHT,B=LEFT":
        print("Setting: Encoder A = Right wheel, Encoder B = Left wheel")
        print("(This is the current mapping - no changes needed)")
        
    elif encoder_mapping == "A=LEFT_NEG,B=RIGHT":
        print("Setting: Encoder A = Left wheel (NEGATED), Encoder B = Right wheel")
        
        # This would need more complex changes - implement if needed
        print("❌ Complex negation mapping - implement manually if needed")
        return False
        
    else:
        print(f"❌ Unknown mapping: {encoder_mapping}")
        print("Supported mappings:")
        print("  A=LEFT,B=RIGHT")
        print("  A=RIGHT,B=LEFT") 
        return False
        
    print("✅ Odometry mapping updated!")
    print("Test with: ./run_pathfinding_robot.sh")
    return True

def show_usage():
    print("🔧 ODOMETRY MAPPING FIXER")
    print("=========================")
    print("")
    print("First, run the motor test:")
    print("  python3 motor_encoder_test.py")
    print("")
    print("Then based on results, fix the mapping:")
    print("  python3 fix_odometry_mapping.py A=LEFT,B=RIGHT")
    print("  python3 fix_odometry_mapping.py A=RIGHT,B=LEFT")
    print("")
    print("Examples of how to interpret test results:")
    print("")
    print("If TURN RIGHT shows:")
    print("  A=+100, B=-100  → A=left wheel, B=right wheel")
    print("  A=-100, B=+100  → A=right wheel, B=left wheel")
    print("")
    print("If FORWARD shows:")
    print("  A=+100, B=+100  → Both encoders increase for forward")
    print("  A=-100, B=-100  → Both encoders decrease for forward (need negation)")

if __name__ == "__main__":
    import os
    os.chdir("/home/dharani/Desktop/DriveSync/robot_control")
    
    if len(sys.argv) > 1:
        fix_odometry_mapping(sys.argv[1])
    else:
        show_usage()


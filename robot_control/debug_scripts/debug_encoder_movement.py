#!/usr/bin/env python3
"""
Debug Encoder Movement
Shows exactly what happens to encoders when you press keys
"""

import serial
import time
import sys

def debug_encoder_movement(port='/dev/ttyUSB0', baudrate=115200):
    print("🔍 ENCODER MOVEMENT DEBUG")
    print("========================")
    print("This will show you EXACTLY what happens to encoders when you move")
    print("")
    
    try:
        # Connect to Arduino
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)  # Wait for Arduino reset
        
        print("✅ Connected to Arduino")
        print("")
        
        def get_encoders():
            """Get current encoder values"""
            ser.write(b'e\r')
            response = ser.readline().decode().strip()
            if response and ' ' in response:
                parts = response.split()
                if len(parts) >= 2:
                    return int(parts[0]), int(parts[1])
            return None, None
        
        def stop_motors():
            """Stop all motors"""
            ser.write(b's\r')
            ser.readline()  # Read OK response
        
        print("📋 TEST SEQUENCE:")
        print("=================")
        print("For each test, I'll:")
        print("1. Show initial encoder values")
        print("2. Send the command")
        print("3. Show encoder changes after movement")
        print("4. Tell you what the GUI SHOULD show")
        print("")
        
        tests = [
            ("TURN LEFT (A key)", "m -100 100", "GUI should turn LEFT"),
            ("TURN RIGHT (D key)", "m 100 -100", "GUI should turn RIGHT"),
        ]
        
        for test_name, command, expected_gui in tests:
            print(f"\n🔄 TEST: {test_name}")
            print(f"Command: {command}")
            print(f"Expected: {expected_gui}")
            print("-" * 40)
            
            # Get initial encoders
            initial_a, initial_b = get_encoders()
            if initial_a is None:
                print("❌ Could not read initial encoders")
                continue
                
            print(f"Initial encoders: A={initial_a}, B={initial_b}")
            
            # Send command
            ser.write((command + '\r').encode())
            response = ser.readline().decode().strip()
            print(f"Arduino response: {response}")
            
            # Wait for movement
            print("Moving for 2 seconds...")
            time.sleep(2)
            
            # Stop motors
            stop_motors()
            
            # Get final encoders
            final_a, final_b = get_encoders()
            if final_a is None:
                print("❌ Could not read final encoders")
                continue
                
            print(f"Final encoders: A={final_a}, B={final_b}")
            
            # Calculate deltas
            delta_a = final_a - initial_a
            delta_b = final_b - initial_b
            
            print(f"Encoder deltas: A={delta_a}, B={delta_b}")
            
            # Analyze what GUI will show
            print("\n📊 GUI ANALYSIS:")
            print(f"Encoder A delta: {delta_a} (this is connected to ? wheel)")
            print(f"Encoder B delta: {delta_b} (this is connected to ? wheel)")
            
            # Based on current odometry logic
            distance_left = -delta_b * 0.000031416  # Approximate conversion
            distance_right = -delta_a * 0.000031628
            distance_left = -distance_left
            distance_right = -distance_right
            delta_theta = (distance_left - distance_right) / 0.20
            
            if abs(delta_theta) > 0.001:
                if delta_theta > 0:
                    gui_turn = "LEFT"
                else:
                    gui_turn = "RIGHT"
                print(f"GUI will show: Turn {gui_turn}")
                print(f"Expected: {expected_gui}")
                if gui_turn in expected_gui:
                    print("✅ CORRECT!")
                else:
                    print("❌ WRONG!")
            else:
                print("GUI will show: No rotation")
            
            input("\nPress ENTER to continue to next test...")
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    debug_encoder_movement(port)

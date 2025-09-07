#!/usr/bin/env python3
"""
Debug Robot Controller Encoder Communication
Shows exactly what the robot controller receives from Arduino
"""

import serial
import time
import sys

def debug_robot_controller_encoders(port='/dev/ttyUSB0', baudrate=115200):
    print("🔍 ROBOT CONTROLLER ENCODER DEBUG")
    print("==================================")
    print("This shows exactly what the robot controller receives")
    print("")
    
    try:
        # Connect exactly like robot controller does
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for Arduino reset
        
        print("✅ Connected to Arduino (same as robot controller)")
        print("")
        
        # Reset encoders like robot controller does
        print("🔄 Resetting encoders...")
        ser.write(b'r\r')
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"Reset response: '{response}'")
        time.sleep(0.5)
        
        print("\n📊 Reading encoders 20 times (like robot controller)...")
        print("If you see all zeros, encoders aren't working with Python")
        print("If you see changing values, encoders ARE working")
        print("")
        
        for i in range(20):
            # Send encoder request exactly like robot controller
            ser.write(b'e\r')
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if response and ' ' in response:
                parts = response.split()
                if len(parts) >= 2:
                    try:
                        encoder_a = int(parts[0])
                        encoder_b = int(parts[1])
                        print(f"Reading {i+1:2d}: A={encoder_a:6d}, B={encoder_b:6d} ✅")
                    except ValueError:
                        print(f"Reading {i+1:2d}: Invalid format: '{response}' ❌")
                else:
                    print(f"Reading {i+1:2d}: Wrong parts count: '{response}' ❌")
            else:
                print(f"Reading {i+1:2d}: No response or invalid: '{response}' ❌")
            
            time.sleep(0.1)
        
        print("\n🎯 DIAGNOSIS:")
        print("=============")
        print("If you saw all zeros:")
        print("  - Encoders aren't connected to Arduino")
        print("  - Wrong Arduino firmware")
        print("  - Different encoder command needed")
        print("")
        print("If you saw changing numbers:")
        print("  - Encoders work with Python!")
        print("  - Issue is in robot controller logic")
        print("")
        print("If you saw 'Invalid format' or 'No response':")
        print("  - Wrong communication protocol")
        print("  - Different Arduino firmware")
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    debug_robot_controller_encoders(port)

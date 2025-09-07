#!/usr/bin/env python3
"""
Test Motors and Encoders
Send motor commands and watch encoder responses
"""

import serial
import time

def test_motors_and_encoders(port='/dev/ttyUSB0', baudrate=115200):
    print("🔄 Testing Motors and Encoders")
    print("=" * 40)
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        
        # Wait for READY
        while ser.in_waiting:
            response = ser.readline().decode().strip()
            if response == "READY":
                print("✅ Arduino ready!")
                break
        
        # Reset encoders first
        print("\n🔄 Resetting encoders...")
        ser.write(b'r\r')
        response = ser.readline().decode().strip()
        print(f"Reset response: {response}")
        time.sleep(0.5)
        
        # Get initial encoder values
        ser.write(b'e\r')
        initial_response = ser.readline().decode().strip()
        print(f"Initial encoders: {initial_response}")
        
        # Test sequence
        tests = [
            ("Forward both motors", "m 100 100", 3),
            ("Stop motors", "s", 1),
            ("Backward both motors", "m -100 -100", 3),
            ("Stop motors", "s", 1),
            ("Turn left (right motor forward)", "m 0 100", 3),
            ("Stop motors", "s", 1),
            ("Turn right (left motor forward)", "m 100 0", 3),
            ("Stop motors", "s", 1),
        ]
        
        for test_name, command, duration in tests:
            print(f"\n🧪 Test: {test_name}")
            print(f"Command: {command}")
            
            # Get encoder values before
            ser.write(b'e\r')
            before = ser.readline().decode().strip()
            print(f"Before: {before}")
            
            # Send command
            ser.write((command + '\r').encode())
            response = ser.readline().decode().strip()
            print(f"Arduino response: {response}")
            
            # Wait for movement
            print(f"Running for {duration} seconds...")
            time.sleep(duration)
            
            # Get encoder values after
            ser.write(b'e\r')
            after = ser.readline().decode().strip()
            print(f"After: {after}")
            
            # Calculate change
            if before and after and ' ' in before and ' ' in after:
                try:
                    before_left, before_right = map(int, before.split())
                    after_left, after_right = map(int, after.split())
                    delta_left = after_left - before_left
                    delta_right = after_right - before_right
                    print(f"Change: Left={delta_left}, Right={delta_right}")
                    
                    if abs(delta_left) > 5 or abs(delta_right) > 5:
                        print("✅ Encoders detected movement!")
                    else:
                        print("⚠️  Little or no encoder movement detected")
                except:
                    print("❌ Could not parse encoder values")
            
            print("-" * 30)
        
        # Final stop
        ser.write(b's\r')
        ser.readline()
        
        ser.close()
        
        print("\n📊 ANALYSIS:")
        print("- If encoders change with motor commands: Hardware is working!")
        print("- If only one encoder changes: Check wiring of the other")
        print("- If no encoders change: Check motor power/connections")
        print("- If encoders change without commands: Check for vibrations/loose connections")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    test_motors_and_encoders()


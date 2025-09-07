#!/usr/bin/env python3
"""
Simple Motor Direction Test
Tests each motor individually to verify correct directions
"""

import serial
import time
import sys

def test_motor_directions(port='/dev/ttyUSB0', baudrate=115200):
    print("🔧 MOTOR DIRECTION TEST")
    print("======================")
    print(f"Connecting to Arduino on {port}...")
    
    try:
        # Connect to Arduino
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)  # Wait for Arduino reset
        
        # Check for READY signal
        print("Waiting for Arduino READY signal...")
        start_time = time.time()
        ready_received = False
        
        while time.time() - start_time < 5:
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"Arduino says: {response}")
                if response == "READY":
                    ready_received = True
                    break
        
        if not ready_received:
            print("⚠️  Warning: No READY signal received (might be old firmware)")
        else:
            print("✅ Arduino is ready!")
        
        print("\n🎯 MOTOR DIRECTION TEST SEQUENCE")
        print("================================")
        print("Watch your robot and note which direction each motor turns:")
        print("")
        
        tests = [
            ("LEFT MOTOR FORWARD", "m 100 0"),
            ("LEFT MOTOR BACKWARD", "m -100 0"),
            ("RIGHT MOTOR FORWARD", "m 0 100"),
            ("RIGHT MOTOR BACKWARD", "m 0 -100"),
            ("BOTH FORWARD", "m 100 100"),
            ("BOTH BACKWARD", "m -100 -100"),
            ("TURN LEFT", "m -100 100"),
            ("TURN RIGHT", "m 100 -100")
        ]
        
        for test_name, command in tests:
            print(f"\n🔄 Testing: {test_name}")
            print(f"Command: {command}")
            
            # Send command
            ser.write((command + '\r').encode())
            response = ser.readline().decode().strip()
            print(f"Arduino response: {response}")
            
            input("Press ENTER when you've observed the movement...")
            
            # Stop motors
            ser.write(b's\r')
            ser.readline()  # Read OK response
            time.sleep(0.5)
        
        print("\n📝 ANALYSIS:")
        print("===========")
        print("Based on what you observed:")
        print("1. Did LEFT MOTOR FORWARD make the LEFT wheel turn forward?")
        print("2. Did RIGHT MOTOR FORWARD make the RIGHT wheel turn forward?") 
        print("3. Did BOTH FORWARD make the robot move forward?")
        print("4. Did TURN LEFT make the robot turn left?")
        print("")
        print("If any of these were wrong, the motor wiring or firmware needs adjustment.")
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    test_motor_directions(port)

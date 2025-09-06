#!/usr/bin/env python3
"""
Comprehensive Motor Test Script
Tests motors at different speeds and provides feedback
"""

import serial
import time

def test_motors():
    print("🔧 COMPREHENSIVE MOTOR TEST")
    print("=" * 40)
    
    try:
        # Connect to Arduino
        ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)
        time.sleep(2)
        print("✅ Connected to Arduino")
        
        # Reset encoders
        ser.write(b'r\r')
        response = ser.readline().decode().strip()
        print(f"Reset encoders: {response}")
        
        # Read initial encoder values
        ser.write(b'e\r')
        initial = ser.readline().decode().strip()
        print(f"Initial encoders: {initial}")
        
        print("\n🚀 Testing Left Motor at different speeds...")
        speeds = [50, 100, 150, 200, 255]
        
        for speed in speeds:
            print(f"\n--- Testing Left Motor at PWM {speed} ---")
            ser.write(f'm {speed} 0\r'.encode())
            response = ser.readline().decode().strip()
            print(f"Command response: {response}")
            
            print("Running for 2 seconds... (Listen for motor sound)")
            time.sleep(2)
            
            # Check encoder change
            ser.write(b'e\r')
            encoders = ser.readline().decode().strip()
            print(f"Encoders after 2 sec: {encoders}")
            
            # Stop motor
            ser.write(b'm 0 0\r')
            ser.readline()  # Read OK response
            time.sleep(1)
        
        print("\n🚀 Testing Right Motor at different speeds...")
        for speed in speeds:
            print(f"\n--- Testing Right Motor at PWM {speed} ---")
            ser.write(f'm 0 {speed}\r'.encode())
            response = ser.readline().decode().strip()
            print(f"Command response: {response}")
            
            print("Running for 2 seconds... (Listen for motor sound)")
            time.sleep(2)
            
            # Check encoder change
            ser.write(b'e\r')
            encoders = ser.readline().decode().strip()
            print(f"Encoders after 2 sec: {encoders}")
            
            # Stop motor
            ser.write(b'm 0 0\r')
            ser.readline()  # Read OK response
            time.sleep(1)
        
        print("\n🚀 Testing Both Motors Together...")
        for speed in [100, 200, 255]:
            print(f"\n--- Testing Both Motors at PWM {speed} ---")
            ser.write(f'm {speed} {speed}\r'.encode())
            response = ser.readline().decode().strip()
            print(f"Command response: {response}")
            
            print("Running for 3 seconds... (Listen for motor sounds)")
            time.sleep(3)
            
            # Check encoder change
            ser.write(b'e\r')
            encoders = ser.readline().decode().strip()
            print(f"Encoders after 3 sec: {encoders}")
            
            # Stop motors
            ser.write(b'm 0 0\r')
            ser.readline()  # Read OK response
            time.sleep(1)
        
        print("\n🔄 Testing Direction Changes...")
        print("Forward then Backward...")
        
        # Forward
        ser.write(b'm 150 150\r')
        ser.readline()
        print("Moving FORWARD for 2 seconds...")
        time.sleep(2)
        
        ser.write(b'e\r')
        forward_enc = ser.readline().decode().strip()
        print(f"Forward encoders: {forward_enc}")
        
        # Backward
        ser.write(b'm -150 -150\r')
        ser.readline()
        print("Moving BACKWARD for 2 seconds...")
        time.sleep(2)
        
        ser.write(b'e\r')
        backward_enc = ser.readline().decode().strip()
        print(f"Backward encoders: {backward_enc}")
        
        # Stop
        ser.write(b'm 0 0\r')
        ser.readline()
        
        print("\n📊 MOTOR TEST RESULTS:")
        print("=" * 40)
        print("✅ If encoders changed during tests: Motors are working electrically")
        print("❓ If you didn't hear motor sounds: Check power supply or motor connections")
        print("❓ If motors sound but don't move: Check mechanical connections to wheels")
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nTroubleshooting:")
        print("1. Check Arduino connection")
        print("2. Check 12V power supply to L298N")
        print("3. Check motor connections to L298N")
        print("4. Check L298N jumpers (ENA, ENB)")

if __name__ == "__main__":
    test_motors()

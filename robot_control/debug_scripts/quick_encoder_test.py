#!/usr/bin/env python3
"""
Quick Encoder Test - No user interaction required
Tests basic Arduino communication and encoder reading
"""

import serial
import time
import sys

def quick_encoder_test(port='/dev/ttyUSB0', baudrate=115200):
    print("🔧 Quick Encoder Test")
    print("=" * 30)
    
    try:
        # Connect to Arduino
        print(f"Connecting to {port} at {baudrate} baud...")
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(3)  # Wait for Arduino reset
        
        print("✅ Connected to Arduino")
        
        # Check for READY signal
        print("Checking for READY signal...")
        ready_found = False
        for i in range(10):
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"📥 Arduino: '{response}'")
                if response == "READY":
                    ready_found = True
                    break
            time.sleep(0.5)
        
        if ready_found:
            print("✅ Arduino firmware is ready!")
        else:
            print("⚠️  No READY signal received")
        
        # Test encoder reading
        print("\n📊 Testing encoder reading...")
        for i in range(5):
            ser.write(b'e\r')
            time.sleep(0.1)
            
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"Encoder data: '{response}'")
                
                if response and ' ' in response:
                    try:
                        parts = response.split()
                        if len(parts) == 2:
                            left_enc = int(parts[0])
                            right_enc = int(parts[1])
                            print(f"  ✅ Left: {left_enc}, Right: {right_enc}")
                        else:
                            print(f"  ⚠️  Wrong format: {len(parts)} parts")
                    except ValueError as e:
                        print(f"  ❌ Parse error: {e}")
                else:
                    print(f"  ❌ Invalid response: '{response}'")
            else:
                print("  ❌ No response")
            
            time.sleep(0.5)
        
        # Test motor command
        print("\n🔄 Testing motor command...")
        ser.write(b'm 0 0\r')
        time.sleep(0.1)
        if ser.in_waiting:
            response = ser.readline().decode().strip()
            print(f"Motor command response: '{response}'")
        
        ser.close()
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    success = quick_encoder_test(port)
    if success:
        print("\n✅ Basic communication working!")
    else:
        print("\n❌ Communication failed!")
    
    print("\nNext steps:")
    print("1. If READY signal missing: Upload OptimizedArduinoFirmware.ino")
    print("2. If encoder data is '0 0': Check encoder wiring")
    print("3. If no response: Check Arduino power and USB connection")


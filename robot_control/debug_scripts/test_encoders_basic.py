#!/usr/bin/env python3
"""
Basic Encoder Test
Just checks if encoders are working at all
"""

import serial
import time
import sys

def test_encoders_basic(port='/dev/ttyUSB0', baudrate=115200):
    print("🔧 BASIC ENCODER TEST")
    print("====================")
    
    try:
        # Connect to Arduino
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)  # Wait for Arduino reset
        
        print("✅ Connected to Arduino")
        print("")
        
        # Wait for READY signal first
        print("Waiting for READY signal...")
        while True:
            if ser.in_waiting:
                try:
                    response = ser.readline().decode('utf-8', errors='ignore').strip()
                    print(f"Arduino says: '{response}'")
                    if response == "READY":
                        break
                except:
                    pass
            time.sleep(0.1)
        
        # Test encoder reading multiple times
        print("📊 Reading encoders 10 times...")
        for i in range(10):
            ser.write(b'e\r')
            try:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"Reading {i+1}: '{response}'")
            except Exception as e:
                print(f"Reading {i+1}: Error - {e}")
            time.sleep(0.5)
        
        print("\n🔄 Now manually turn your robot wheels and watch:")
        print("(Turn the wheels by hand while this runs)")
        print("")
        
        # Continuous encoder reading
        for i in range(20):
            ser.write(b'e\r')
            try:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"Manual test {i+1}: '{response}'")
            except Exception as e:
                print(f"Manual test {i+1}: Error - {e}")
            time.sleep(1)
            
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    test_encoders_basic(port)

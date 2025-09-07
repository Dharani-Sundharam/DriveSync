#!/usr/bin/env python3
"""
Find Arduino Baudrate
Tests different baudrates to find which one works
"""

import serial
import time

def test_baudrate(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        
        # Try motor command (we know this works)
        ser.write(b'm 0 0\r')
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        
        ser.close()
        return response
    except:
        return None

def find_arduino_baudrate(port='/dev/ttyUSB0'):
    print("🔍 FINDING ARDUINO BAUDRATE")
    print("===========================")
    
    baudrates = [9600, 19200, 38400, 57600, 115200]
    
    for baud in baudrates:
        print(f"Testing {baud} baud... ", end='')
        response = test_baudrate(port, baud)
        
        if response is not None:
            print(f"Response: '{response}' ✅")
            if response == "OK" or response == "READY" or len(response) > 0:
                print(f"🎯 FOUND: Arduino is using {baud} baud!")
                return baud
        else:
            print("No response ❌")
    
    print("❌ Could not find working baudrate")
    return None

if __name__ == "__main__":
    find_arduino_baudrate()

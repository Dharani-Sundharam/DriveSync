#!/usr/bin/env python3
"""
Simple Serial Reader
Just reads whatever the Arduino is sending
"""

import serial
import time

def read_serial(port='/dev/ttyUSB0', baudrate=115200, duration=10):
    print(f"Reading from {port} at {baudrate} baud for {duration} seconds...")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection
        
        print("Connected! Reading data...")
        print("-" * 40)
        
        start_time = time.time()
        line_count = 0
        
        while (time.time() - start_time) < duration:
            if ser.in_waiting > 0:
                try:
                    data = ser.readline().decode().strip()
                    if data:
                        line_count += 1
                        print(f"{line_count:03d}: {data}")
                except:
                    raw = ser.readline()
                    print(f"{line_count:03d}: RAW: {raw}")
                    line_count += 1
            
            time.sleep(0.1)
        
        print("-" * 40)
        print(f"Finished. Received {line_count} lines.")
        
        # Test sending a command
        print("\nTesting 'e' command:")
        ser.write(b'e\r')
        time.sleep(0.2)
        
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Response: '{response}'")
        else:
            print("No response to 'e' command")
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    read_serial()


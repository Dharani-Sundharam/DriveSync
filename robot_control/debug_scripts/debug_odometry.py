#!/usr/bin/env python3
"""
Comprehensive Odometry Debug Tool
Shows exactly what encoders are doing vs GUI visualization
"""

import serial
import time
import math
import sys

def debug_odometry(port='/dev/ttyUSB0', baudrate=115200):
    print("🔍 ODOMETRY DEBUG SESSION")
    print("========================")
    print("This will show you exactly what's happening with encoders vs GUI")
    print("")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)
        
        # Reset encoders first
        print("🔄 Resetting encoders...")
        ser.write(b'r\r')
        response = ser.readline().decode().strip()
        print(f"Reset response: {response}")
        time.sleep(0.5)
        
        print("\n📊 ENCODER MONITORING")
        print("====================")
        print("Format: [Encoder A] [Encoder B] -> [Delta A] [Delta B] -> [GUI Movement Prediction]")
        print("")
        
        # Get initial encoder values
        ser.write(b'e\r')
        initial_response = ser.readline().decode().strip()
        if ' ' in initial_response:
            initial_a, initial_b = map(int, initial_response.split())
        else:
            print("❌ Failed to read initial encoders")
            return
            
        print(f"Initial encoders: A={initial_a}, B={initial_b}")
        print("")
        print("Now perform these movements and watch the output:")
        print("1. Push robot forward manually")
        print("2. Push robot backward manually") 
        print("3. Turn robot left manually")
        print("4. Turn robot right manually")
        print("")
        print("Press Ctrl+C to exit")
        print("")
        
        last_a, last_b = initial_a, initial_b
        
        while True:
            try:
                # Read current encoders
                ser.write(b'e\r')
                response = ser.readline().decode().strip()
                
                if ' ' in response:
                    current_a, current_b = map(int, response.split())
                    
                    # Calculate deltas
                    delta_a = current_a - last_a
                    delta_b = current_b - last_b
                    
                    # Only show if there's movement
                    if abs(delta_a) > 2 or abs(delta_b) > 2:
                        
                        # Predict GUI behavior with CURRENT odometry logic
                        # Current logic: delta_left = delta_b, delta_right = delta_a (swapped)
                        gui_delta_left = delta_b
                        gui_delta_right = delta_a
                        
                        # Predict movement
                        movement = "UNKNOWN"
                        if gui_delta_left > 0 and gui_delta_right > 0:
                            movement = "GUI thinks: FORWARD"
                        elif gui_delta_left < 0 and gui_delta_right < 0:
                            movement = "GUI thinks: BACKWARD"
                        elif gui_delta_left > 0 and gui_delta_right < 0:
                            movement = "GUI thinks: TURN LEFT"
                        elif gui_delta_left < 0 and gui_delta_right > 0:
                            movement = "GUI thinks: TURN RIGHT"
                        elif abs(gui_delta_left) > abs(gui_delta_right):
                            movement = "GUI thinks: LEFT WHEEL DOMINANT"
                        elif abs(gui_delta_right) > abs(gui_delta_left):
                            movement = "GUI thinks: RIGHT WHEEL DOMINANT"
                            
                        print(f"[{current_a:6d}] [{current_b:6d}] -> [{delta_a:4d}] [{delta_b:4d}] -> {movement}")
                        
                        last_a, last_b = current_a, current_b
                
                time.sleep(0.1)  # 10Hz monitoring
                
            except KeyboardInterrupt:
                break
                
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")

def analyze_results():
    print("\n📋 ANALYSIS GUIDE:")
    print("=================")
    print("Compare what you observed vs what the GUI predicted:")
    print("")
    print("If you pushed FORWARD and GUI predicted BACKWARD:")
    print("  → Need to invert BOTH encoder assignments")
    print("")
    print("If you pushed FORWARD and GUI predicted TURN:")
    print("  → Encoder assignments are wrong (A/B swapped incorrectly)")
    print("")
    print("If you turned LEFT and GUI predicted RIGHT:")
    print("  → Need to swap the encoder assignments the other way")
    print("")
    print("If movements match:")
    print("  → Odometry is correct, problem might be elsewhere")

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    try:
        debug_odometry(port)
    finally:
        analyze_results()


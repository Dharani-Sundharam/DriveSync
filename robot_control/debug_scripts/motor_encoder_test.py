#!/usr/bin/env python3
"""
Motor Direction vs Encoder Test
Run motors in each direction and observe encoder changes
"""

import serial
import time
import sys

def test_motor_encoder_mapping(port='/dev/ttyUSB0', baudrate=115200):
    print("🔧 MOTOR vs ENCODER MAPPING TEST")
    print("================================")
    print("This will run motors and show encoder changes")
    print("")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)
        
        # Reset encoders
        print("🔄 Resetting encoders...")
        ser.write(b'r\r')
        response = ser.readline().decode().strip()
        print(f"Reset response: {response}")
        time.sleep(0.5)
        
        tests = [
            ("FORWARD (both motors +)", "m 100 100"),
            ("BACKWARD (both motors -)", "m -100 -100"),
            ("TURN RIGHT (left+, right-)", "m 100 -100"),
            ("TURN LEFT (left-, right+)", "m -100 100")
        ]
        
        print("\n📊 MOTOR TEST RESULTS:")
        print("======================")
        print("Format: [Test] -> Encoder A change, Encoder B change")
        print("")
        
        for test_name, command in tests:
            # Get initial encoder values
            ser.write(b'e\r')
            initial_response = ser.readline().decode().strip()
            if ' ' not in initial_response:
                print(f"❌ Failed to read encoders for {test_name}")
                continue
                
            initial_a, initial_b = map(int, initial_response.split())
            
            print(f"🎯 {test_name}")
            print(f"   Command: {command}")
            print(f"   Initial encoders: A={initial_a}, B={initial_b}")
            
            # Run motor command
            ser.write((command + '\r').encode())
            response = ser.readline().decode().strip()
            print(f"   Arduino response: {response}")
            
            # Run for 2 seconds
            time.sleep(2)
            
            # Stop motors
            ser.write(b's\r')
            ser.readline()  # Read OK response
            
            # Get final encoder values
            ser.write(b'e\r')
            final_response = ser.readline().decode().strip()
            if ' ' not in final_response:
                print(f"❌ Failed to read final encoders")
                continue
                
            final_a, final_b = map(int, final_response.split())
            
            # Calculate changes
            delta_a = final_a - initial_a
            delta_b = final_b - initial_b
            
            print(f"   Final encoders: A={final_a}, B={final_b}")
            print(f"   📈 ENCODER CHANGES: A={delta_a:+d}, B={delta_b:+d}")
            
            # Analyze the results
            if "FORWARD" in test_name:
                if delta_a > 0 and delta_b > 0:
                    print("   ✅ Both encoders increased (expected for forward)")
                else:
                    print("   ⚠️  Unexpected encoder pattern for forward motion")
            elif "BACKWARD" in test_name:
                if delta_a < 0 and delta_b < 0:
                    print("   ✅ Both encoders decreased (expected for backward)")
                else:
                    print("   ⚠️  Unexpected encoder pattern for backward motion")
            elif "RIGHT" in test_name:
                if delta_a > 0 and delta_b < 0:
                    print("   📝 Turn right: A+ B- (left wheel forward, right wheel back)")
                elif delta_a < 0 and delta_b > 0:
                    print("   📝 Turn right: A- B+ (left wheel back, right wheel forward)")
                else:
                    print("   ⚠️  Unexpected pattern for right turn")
            elif "LEFT" in test_name:
                if delta_a < 0 and delta_b > 0:
                    print("   📝 Turn left: A- B+ (left wheel back, right wheel forward)")
                elif delta_a > 0 and delta_b < 0:
                    print("   📝 Turn left: A+ B- (left wheel forward, right wheel back)")
                else:
                    print("   ⚠️  Unexpected pattern for left turn")
            
            print("")
            time.sleep(1)  # Pause between tests
        
        ser.close()
        
        print("🧠 ANALYSIS:")
        print("============")
        print("Based on the results above, determine:")
        print("1. Which encoder (A or B) corresponds to LEFT wheel?")
        print("2. Which encoder (A or B) corresponds to RIGHT wheel?")
        print("3. Do positive encoder values mean forward wheel motion?")
        print("")
        print("Then we can fix the odometry mapping accordingly!")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    test_motor_encoder_mapping(port)


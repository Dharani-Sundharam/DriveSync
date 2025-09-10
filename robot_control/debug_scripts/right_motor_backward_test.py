#!/usr/bin/env python3
"""
Right Motor Backward Movement Test
Specifically tests the right motor backward functionality
"""

import serial
import time
import sys

def test_right_motor_backward(port='/dev/ttyUSB0'):
    """Test only the right motor backward movement"""
    print("🔍 RIGHT MOTOR BACKWARD TEST")
    print("=" * 40)
    print("Testing right motor backward movement specifically...")
    
    try:
        # Connect to Arduino
        print(f"Connecting to {port}...")
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(2)
        
        # Clear any startup messages
        while ser.in_waiting:
            ser.readline()
        
        print("✅ Connected to Arduino")
        
        # Test sequence
        tests = [
            ("Right motor FORWARD (should work)", "m 0 100", 3),
            ("STOP motors", "s", 1),
            ("Right motor BACKWARD (the problem)", "m 0 -100", 3),
            ("STOP motors", "s", 1),
            ("Left motor BACKWARD (for comparison)", "m -100 0", 3),
            ("STOP motors", "s", 1)
        ]
        
        for description, command, duration in tests:
            print(f"\n🧪 {description}")
            print(f"   Command: {command}")
            
            # Send command
            ser.write(f"{command}\r".encode())
            time.sleep(0.1)
            
            # Read response
            response = ""
            if ser.in_waiting:
                response = ser.readline().decode().strip()
            print(f"   Response: '{response}'")
            
            # Wait and observe
            if "FORWARD" in description or "BACKWARD" in description:
                print(f"   👀 Observe motor for {duration} seconds...")
                time.sleep(duration)
                
                # Ask user for observation
                if "Right motor FORWARD" in description:
                    result = input("   Did right motor spin FORWARD? (y/n): ").lower().strip()
                    if result == 'n':
                        print("   ❌ Right motor forward not working - hardware issue!")
                        return
                    else:
                        print("   ✅ Right motor forward works")
                        
                elif "Right motor BACKWARD" in description:
                    result = input("   Did right motor spin BACKWARD? (y/n): ").lower().strip()
                    if result == 'n':
                        print("   ❌ RIGHT MOTOR BACKWARD ISSUE CONFIRMED!")
                        print("   🔧 This is the problem we need to fix")
                    else:
                        print("   ✅ Right motor backward works (problem may be intermittent)")
                        
                elif "Left motor BACKWARD" in description:
                    result = input("   Did left motor spin BACKWARD? (y/n): ").lower().strip()
                    if result == 'y':
                        print("   ✅ Left motor backward works fine (confirms Arduino code is OK)")
                    else:
                        print("   ❌ Left motor backward also has issues")
            else:
                time.sleep(duration)
        
        ser.close()
        print("\n📊 TEST COMPLETE")
        
    except serial.SerialException as e:
        print(f"❌ Serial connection error: {e}")
        print("💡 Make sure Arduino is connected and not in use by another program")
    except Exception as e:
        print(f"❌ Error: {e}")

def hardware_diagnosis():
    """Provide hardware diagnosis steps"""
    print("\n🔧 HARDWARE DIAGNOSIS FOR RIGHT MOTOR BACKWARD")
    print("=" * 50)
    
    print("Based on your symptoms (right motor forward works, backward doesn't):")
    print()
    
    print("🎯 MOST LIKELY CAUSES:")
    print("1. Pin 5 (Arduino) → IN4 (L298N) connection issue")
    print("   • Loose wire on Pin 5")
    print("   • Bad solder joint")
    print("   • Damaged jumper wire")
    print()
    
    print("2. L298N IN4 input damaged")
    print("   • Internal damage to L298N chip")
    print("   • IN4 input not responding")
    print()
    
    print("3. Arduino Pin 5 hardware fault")
    print("   • Pin 5 unable to output LOW signal")
    print("   • Pin 5 stuck HIGH")
    print()
    
    print("🔍 DIAGNOSTIC STEPS:")
    print()
    print("Step 1: Check connections")
    print("   • Verify Pin 5 wire is firmly connected")
    print("   • Check continuity from Arduino Pin 5 to L298N IN4")
    print("   • Look for loose connections")
    print()
    
    print("Step 2: Test with multimeter")
    print("   • Measure voltage on Pin 5 during backward command")
    print("   • Should be ~5V during forward, ~0V during backward")
    print("   • If Pin 5 stays HIGH, Arduino pin issue")
    print()
    
    print("Step 3: Swap test")
    print("   • Temporarily swap Pin 5 and Pin 6 wires")
    print("   • If problem moves to forward direction → Pin 5 fault")
    print("   • If problem stays with backward → L298N IN4 fault")

def quick_fix_suggestions():
    """Provide quick fix suggestions"""
    print("\n⚡ QUICK FIXES TO TRY:")
    print("=" * 25)
    
    print("1. 🔄 Wire Swap Test (5 minutes)")
    print("   • Swap the wires on Pin 5 and Pin 6")
    print("   • Test again - does the problem move?")
    print()
    
    print("2. 🔌 Re-seat Connections (2 minutes)")
    print("   • Unplug and replug Pin 5 connection")
    print("   • Make sure it's firmly seated")
    print()
    
    print("3. 📍 Try Different Pin (10 minutes)")
    print("   • Move Pin 5 wire to Pin 7 or Pin 8")
    print("   • Update Arduino code: #define RIGHT_MOTOR_BACKWARD 7")
    print("   • Recompile and test")
    print()
    
    print("4. 🔧 Firmware Force Fix (5 minutes)")
    print("   • Add explicit digitalWrite commands")
    print("   • Force pins LOW in stop function")

if __name__ == "__main__":
    print("🎯 RIGHT MOTOR BACKWARD DIAGNOSTIC TOOL")
    print()
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = '/dev/ttyUSB0'
    
    choice = input("What would you like to do?\n1. Test right motor backward\n2. Hardware diagnosis\n3. Quick fix suggestions\n4. All of the above\nChoice (1-4): ").strip()
    
    if choice == '1':
        test_right_motor_backward(port)
    elif choice == '2':
        hardware_diagnosis()
    elif choice == '3':
        quick_fix_suggestions()
    else:
        test_right_motor_backward(port)
        hardware_diagnosis()
        quick_fix_suggestions()

#!/usr/bin/env python3
"""
Right Motor Specific Debug
Since left motor works fine, debug only right motor control signals
"""

import serial
import time

def debug_right_motor_signals(port='/dev/ttyUSB0'):
    print("🔍 Right Motor Signal Debug")
    print("=" * 40)
    print("Left motor works fine, focusing on RIGHT motor...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        
        # Clear any startup messages
        while ser.in_waiting:
            ser.readline()
        
        print("\n1️⃣ Testing RIGHT motor control signals individually:")
        
        # Test right motor forward only (IN3 = Pin 6)
        print("\n   Testing RIGHT motor FORWARD signal (Pin 6 → IN3):")
        ser.write(b'm 0 100\r')  # Right motor forward only
        time.sleep(0.1)
        
        response = ""
        if ser.in_waiting:
            response = ser.readline().decode().strip()
        print(f"   Arduino response: '{response}'")
        
        print("   👀 Right motor should spin FORWARD...")
        time.sleep(2)
        
        # Try to stop
        print("   🛑 Sending STOP command...")
        ser.write(b's\r')
        time.sleep(0.1)
        
        forward_stops = input("   Did right motor STOP spinning forward? (y/n): ").lower().strip()
        
        if forward_stops == 'n':
            print("   ❌ RIGHT FORWARD signal stuck!")
            print("   🔧 Problem: Pin 6 (IN3) not releasing properly")
        else:
            print("   ✅ RIGHT FORWARD signal works")
        
        time.sleep(1)
        
        # Test right motor backward only (IN4 = Pin 5)  
        print("\n   Testing RIGHT motor BACKWARD signal (Pin 5 → IN4):")
        ser.write(b'm 0 -100\r')  # Right motor backward only
        time.sleep(0.1)
        
        response = ""
        if ser.in_waiting:
            response = ser.readline().decode().strip()
        print(f"   Arduino response: '{response}'")
        
        print("   👀 Right motor should spin BACKWARD...")
        time.sleep(2)
        
        # Try to stop
        print("   🛑 Sending STOP command...")
        ser.write(b's\r')
        time.sleep(0.1)
        
        backward_stops = input("   Did right motor STOP spinning backward? (y/n): ").lower().strip()
        
        if backward_stops == 'n':
            print("   ❌ RIGHT BACKWARD signal stuck!")
            print("   🔧 Problem: Pin 5 (IN4) not releasing properly")
        else:
            print("   ✅ RIGHT BACKWARD signal works")
        
        # Compare with left motor
        print("\n2️⃣ Comparing with LEFT motor (which works fine):")
        print("   Testing LEFT motor for comparison...")
        
        ser.write(b'm 100 0\r')  # Left motor only
        time.sleep(2)
        ser.write(b's\r')
        time.sleep(0.1)
        
        left_comparison = input("   Did LEFT motor start and stop properly? (y/n): ").lower().strip()
        
        # Analysis
        print("\n📊 ANALYSIS:")
        print("=" * 20)
        
        if forward_stops == 'n' or backward_stops == 'n':
            print("❌ RIGHT MOTOR CONTROL SIGNAL ISSUE CONFIRMED")
            print("\n🔧 SPECIFIC PROBLEMS:")
            
            if forward_stops == 'n':
                print("   • Pin 6 (Arduino) → IN3 (L298N) signal stuck HIGH")
                print("   • Right motor keeps spinning forward")
            
            if backward_stops == 'n':
                print("   • Pin 5 (Arduino) → IN4 (L298N) signal stuck HIGH") 
                print("   • Right motor keeps spinning backward")
            
            print("\n🛠️  HARDWARE FIXES TO TRY:")
            print("   1. Check Pin 6 and Pin 5 connections to L298N")
            print("   2. Swap Pin 6 ↔ Pin 10 (test with left motor pins)")
            print("   3. Add 1kΩ pull-down resistors on Pin 5 and Pin 6")
            print("   4. Check for short circuits in right motor wiring")
            print("   5. Try different L298N IN3/IN4 inputs")
            
            print("\n🔬 FIRMWARE DEBUG:")
            print("   The issue is likely hardware, but you can also try:")
            print("   • Add explicit digitalWrite(6, LOW) in stop function")
            print("   • Add explicit digitalWrite(5, LOW) in stop function")
            
        else:
            print("✅ RIGHT MOTOR SIGNALS WORK PROPERLY")
            print("🤔 If motor still runs continuously, check:")
            print("   • Mechanical issues (motor stuck)")
            print("   • L298N internal fault")
            print("   • Power supply noise causing restarts")
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")

def create_right_motor_fix():
    """Create specific fix for right motor control"""
    print("\n💻 RIGHT MOTOR FIRMWARE FIX:")
    print("=" * 35)
    
    fix_code = '''
// Add this to your stopMotors() function for explicit right motor control:

void stopMotors() {
  // Explicit control for all pins
  analogWrite(LEFT_MOTOR_FORWARD, 0);   // Pin 10
  analogWrite(LEFT_MOTOR_BACKWARD, 0);  // Pin 9
  analogWrite(RIGHT_MOTOR_FORWARD, 0);  // Pin 6 - EXPLICIT
  analogWrite(RIGHT_MOTOR_BACKWARD, 0); // Pin 5 - EXPLICIT
  
  // FORCE digital pins LOW (in case analogWrite doesn't work)
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);   // Pin 10
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);  // Pin 9  
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);  // Pin 6 - FORCE LOW
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); // Pin 5 - FORCE LOW
  
  // Keep enable pins HIGH as you had them
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  
  delay(50); // Longer delay to ensure signals processed
}
'''
    
    print(fix_code)

def hardware_swap_test():
    print("\n🔄 HARDWARE SWAP TEST:")
    print("=" * 25)
    print("To isolate if it's Arduino pins or L298N inputs:")
    print("")
    print("1. 📍 Current wiring:")
    print("   Left:  Pin 10 → IN1, Pin 9  → IN2")  
    print("   Right: Pin 6  → IN3, Pin 5  → IN4")
    print("")
    print("2. 🔄 SWAP TEST - Change to:")
    print("   Left:  Pin 6  → IN1, Pin 5  → IN2")
    print("   Right: Pin 10 → IN3, Pin 9  → IN4") 
    print("")
    print("3. 🧪 Test result:")
    print("   • If LEFT motor now has runaway → Arduino pins 6/5 faulty")
    print("   • If RIGHT motor still has runaway → L298N IN3/IN4 faulty")
    print("   • If both work → Original wiring had loose connections")

if __name__ == "__main__":
    print("🎯 RIGHT MOTOR SPECIFIC DEBUG")
    print("Left motor works fine, so focusing on right motor signals")
    print("")
    
    choice = input("What do you want to do?\n1. Debug right motor signals\n2. Show firmware fix\n3. Show hardware swap test\nChoice (1-3): ")
    
    if choice == '1':
        debug_right_motor_signals()
    elif choice == '2':
        create_right_motor_fix()
    elif choice == '3':
        hardware_swap_test()
    else:
        debug_right_motor_signals()
        create_right_motor_fix()
        hardware_swap_test()

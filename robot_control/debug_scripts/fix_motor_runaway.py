#!/usr/bin/env python3
"""
Fix Motor Runaway Issue
Diagnose and fix motors that spin continuously and won't stop
"""

import serial
import time

def emergency_stop_all_motors(port='/dev/ttyUSB0'):
    """Emergency stop - send multiple stop commands"""
    print("🚨 EMERGENCY MOTOR STOP")
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(1)
        
        # Send multiple stop commands
        for i in range(5):
            ser.write(b's\r')  # Stop command
            ser.write(b'm 0 0\r')  # Zero speed command
            time.sleep(0.1)
            print(f"   Stop command {i+1} sent")
        
        ser.close()
        print("✅ Emergency stop commands sent")
        
    except Exception as e:
        print(f"❌ Emergency stop failed: {e}")

def diagnose_motor_runaway(port='/dev/ttyUSB0'):
    print("🔍 Motor Runaway Diagnosis")
    print("=" * 40)
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        
        # First, ensure motors are stopped
        print("1️⃣ Sending initial stop commands...")
        for _ in range(3):
            ser.write(b's\r')
            ser.write(b'm 0 0\r')
            time.sleep(0.2)
        
        # Clear any pending responses
        while ser.in_waiting:
            ser.readline()
        
        print("\n2️⃣ Testing motor control precision...")
        
        # Test with very low speed first
        print("   Testing RIGHT motor at LOW speed (50)...")
        ser.write(b'm 0 50\r')
        time.sleep(0.1)
        
        response = ""
        if ser.in_waiting:
            response = ser.readline().decode().strip()
        print(f"   Arduino response: '{response}'")
        
        print("   ⏰ Motor should spin SLOWLY for 2 seconds...")
        time.sleep(2)
        
        print("   🛑 Sending STOP command...")
        ser.write(b's\r')
        time.sleep(0.1)
        
        stop_response = ""
        if ser.in_waiting:
            stop_response = ser.readline().decode().strip()
        print(f"   Stop response: '{response}'")
        
        motor_stopped = input("   Did the RIGHT motor STOP? (y/n): ").lower().strip()
        
        if motor_stopped == 'n':
            print("   ❌ MOTOR RUNAWAY CONFIRMED!")
            print("\n🔧 MOTOR RUNAWAY CAUSES:")
            print("   1. L298N Enable pins stuck HIGH")
            print("   2. Arduino pins floating or not properly controlled")
            print("   3. Wiring short circuits")
            print("   4. Faulty L298N driver")
            
            # Emergency stop
            emergency_stop_all_motors(port)
            
            print("\n🚨 IMMEDIATE FIXES TO TRY:")
            print("   1. Disconnect power to L298N immediately")
            print("   2. Check enable pin connections (Pin 12 → ENB)")
            print("   3. Verify no short circuits in wiring")
            print("   4. Test with different L298N if available")
            
        else:
            print("   ✅ Motor stops correctly at low speed")
            
            # Test higher speed
            print("\n3️⃣ Testing at HIGHER speed...")
            print("   Testing RIGHT motor at MEDIUM speed (150)...")
            
            ser.write(b'm 0 150\r')
            time.sleep(2)
            ser.write(b's\r')
            
            high_speed_stop = input("   Did motor stop at higher speed? (y/n): ").lower().strip()
            
            if high_speed_stop == 'n':
                print("   ❌ Motor runs away at HIGH speed only")
                print("   🔧 Possible causes:")
                print("      - Back EMF issues")
                print("      - Insufficient braking")
                print("      - PWM frequency problems")
            else:
                print("   ✅ Motor control working properly")
        
        # Test LEFT motor
        print("\n4️⃣ Testing LEFT motor...")
        ser.write(b'm 150 0\r')
        time.sleep(2)
        ser.write(b's\r')
        
        left_motor_stop = input("   Did LEFT motor stop properly? (y/n): ").lower().strip()
        
        if left_motor_stop == 'n':
            print("   ❌ LEFT motor also has runaway issue")
        else:
            print("   ✅ LEFT motor working properly")
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")

def create_motor_brake_firmware():
    """Create firmware with proper motor braking"""
    print("\n💾 IMPROVED FIRMWARE WITH MOTOR BRAKING:")
    print("=" * 50)
    
    firmware = '''
// Add this to your Arduino firmware for better motor control

void setMotorSpeeds(int left_speed, int right_speed) {
  // Constrain speeds
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  // LEFT MOTOR with braking
  if (left_speed > 0) {
    analogWrite(LEFT_MOTOR_FORWARD, left_speed);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  } else if (left_speed < 0) {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, -left_speed);
  } else {
    // BRAKE - both pins LOW for active braking
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  }
  
  // RIGHT MOTOR with braking
  if (right_speed > 0) {
    analogWrite(RIGHT_MOTOR_FORWARD, right_speed);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  } else if (right_speed < 0) {
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, -right_speed);
  } else {
    // BRAKE - both pins LOW for active braking
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  }
  
  // Enable motors only when needed
  if (left_speed != 0) {
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  } else {
    digitalWrite(LEFT_MOTOR_ENABLE, LOW);  // Disable when stopped
  }
  
  if (right_speed != 0) {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  } else {
    digitalWrite(RIGHT_MOTOR_ENABLE, LOW); // Disable when stopped
  }
}

void stopMotors() {
  // Force all pins LOW
  analogWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  
  // Disable motors completely
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);
  
  delay(10); // Brief delay to ensure signals are processed
}
'''
    
    print(firmware)

def hardware_fixes():
    print("\n🔧 HARDWARE FIXES FOR MOTOR RUNAWAY:")
    print("=" * 40)
    print("1. 🔌 ADD PULL-DOWN RESISTORS:")
    print("   - 10kΩ resistor from each Arduino pin to GND")
    print("   - Prevents floating pins when Arduino resets")
    print("")
    print("2. ⚡ ENABLE PIN CONTROL:")
    print("   - Make sure Pin 12 (ENB) is properly connected")
    print("   - Consider adding a switch to manually disable motors")
    print("")
    print("3. 🔄 MOTOR DRIVER FIXES:")
    print("   - Try different L298N module (might be faulty)")
    print("   - Check for loose connections")
    print("   - Verify power supply stability")
    print("")
    print("4. 🚨 EMERGENCY STOP:")
    print("   - Add physical switch to cut motor power")
    print("   - Wire emergency stop button to enable pins")

if __name__ == "__main__":
    print("🚨 MOTOR RUNAWAY FIXER")
    print("=" * 30)
    print("This will help fix motors that won't stop spinning")
    print("")
    
    choice = input("What do you want to do?\n1. Emergency stop NOW\n2. Diagnose runaway issue\n3. Show firmware fixes\n4. Show hardware fixes\nChoice (1-4): ")
    
    if choice == '1':
        emergency_stop_all_motors()
    elif choice == '2':
        diagnose_motor_runaway()
    elif choice == '3':
        create_motor_brake_firmware()
    elif choice == '4':
        hardware_fixes()
    else:
        print("Running full diagnosis...")
        diagnose_motor_runaway()
        create_motor_brake_firmware()
        hardware_fixes()

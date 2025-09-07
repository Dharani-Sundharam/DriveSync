#!/usr/bin/env python3
"""
Motor Configuration Fix
Diagnose and fix motor spinning issues
"""

import serial
import time

def diagnose_motors(port='/dev/ttyUSB0', baudrate=115200):
    print("🔧 Motor Configuration Diagnosis")
    print("=" * 50)
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)
        
        # Wait for READY
        ready_found = False
        for _ in range(10):
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                if response == "READY":
                    ready_found = True
                    print("✅ Arduino firmware ready")
                    break
        
        if not ready_found:
            print("❌ Arduino not ready - check firmware upload")
            return False
        
        print("\n🔍 MOTOR DIAGNOSIS SEQUENCE")
        print("=" * 30)
        
        # Test 1: Check if Arduino responds correctly to motor commands
        print("\n1️⃣ Testing Arduino motor command response:")
        ser.write(b'm 100 100\r')
        time.sleep(0.1)
        
        response = ""
        if ser.in_waiting:
            response = ser.readline().decode().strip()
            print(f"   Response to 'm 100 100': '{response}'")
            
            if response == "OK":
                print("   ✅ Arduino correctly acknowledges motor commands")
            elif "0" in response:
                print("   ❌ Arduino responding with encoder data instead of 'OK'")
                print("   🔧 This suggests firmware issue or wrong command format")
            else:
                print(f"   ⚠️  Unexpected response: '{response}'")
        else:
            print("   ❌ No response from Arduino")
        
        # Stop motors
        ser.write(b's\r')
        ser.readline()
        
        # Test 2: Individual motor testing
        print("\n2️⃣ Testing individual motors:")
        
        motor_tests = [
            ("Left Motor Forward", "m 150 0", "Should spin left motor forward"),
            ("Left Motor Backward", "m -150 0", "Should spin left motor backward"), 
            ("Right Motor Forward", "m 0 150", "Should spin right motor forward"),
            ("Right Motor Backward", "m 0 -150", "Should spin right motor backward"),
        ]
        
        for test_name, command, description in motor_tests:
            print(f"\n   🧪 {test_name}:")
            print(f"      Command: {command}")
            print(f"      Expected: {description}")
            
            # Send command
            ser.write((command + '\r').encode())
            time.sleep(0.1)
            
            response = ""
            if ser.in_waiting:
                response = ser.readline().decode().strip()
            
            print(f"      Arduino response: '{response}'")
            print(f"      👀 WATCH YOUR MOTORS NOW - Do you see/hear movement?")
            
            # Let it run for 2 seconds
            time.sleep(2)
            
            # Stop
            ser.write(b's\r')
            ser.readline()
            
            user_input = input(f"      Did the motor spin? (y/n): ").lower().strip()
            if user_input == 'y':
                print(f"      ✅ {test_name} working!")
            else:
                print(f"      ❌ {test_name} NOT working")
            
            time.sleep(1)
        
        # Test 3: Maximum power test
        print("\n3️⃣ Maximum power test:")
        print("   Testing both motors at maximum speed...")
        
        ser.write(b'm 255 255\r')
        response = ser.readline().decode().strip()
        print(f"   Command 'm 255 255' response: '{response}'")
        print("   👀 WATCH/LISTEN - Any motor movement or sounds?")
        
        time.sleep(3)
        
        ser.write(b's\r')
        ser.readline()
        
        max_power_result = input("   Did you see/hear ANY motor activity? (y/n): ").lower().strip()
        
        # Analysis and recommendations
        print("\n📊 DIAGNOSIS RESULTS:")
        print("=" * 30)
        
        if max_power_result == 'n':
            print("❌ NO MOTOR ACTIVITY DETECTED")
            print("\n🔧 HARDWARE ISSUES TO CHECK:")
            print("   1. L298N Motor Driver Power:")
            print("      - Check 12V power supply connected to L298N VCC")
            print("      - Verify ground connections (Arduino GND ↔ L298N GND)")
            print("      - Check 5V logic power (Arduino 5V → L298N VCC logic)")
            print("")
            print("   2. Enable Pins (CRITICAL):")
            print("      - Pin 13 (ENA) must be connected and HIGH")
            print("      - Pin 12 (ENB) must be connected and HIGH")
            print("      - Without these, motors will NEVER work!")
            print("")
            print("   3. Motor Connections:")
            print("      - Left motor: L298N OUT1, OUT2")
            print("      - Right motor: L298N OUT3, OUT4")
            print("")
            print("   4. Arduino → L298N Wiring:")
            print("      - Pin 10 → IN1,  Pin 9 → IN2")
            print("      - Pin 6 → IN3,   Pin 5 → IN4")
            print("      - Pin 13 → ENA,  Pin 12 → ENB")
        
        elif max_power_result == 'y':
            print("✅ SOME MOTOR ACTIVITY DETECTED")
            print("🔧 Possible issues:")
            print("   - Motors may be underpowered")
            print("   - Wiring may be loose")
            print("   - Individual motor issues")
            print("   - Check connections for each non-working motor")
        
        ser.close()
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def create_motor_test_firmware():
    """Create a simple test firmware to verify motor hardware"""
    firmware_code = '''
/*
 * Simple Motor Test Firmware
 * Upload this to test ONLY motor hardware
 */

// L298N connections
#define LEFT_MOTOR_FORWARD   10   // IN1
#define LEFT_MOTOR_BACKWARD   9   // IN2
#define RIGHT_MOTOR_FORWARD   6   // IN3
#define RIGHT_MOTOR_BACKWARD  5   // IN4
#define LEFT_MOTOR_ENABLE    13   // ENA - CRITICAL!
#define RIGHT_MOTOR_ENABLE   12   // ENB - CRITICAL!

void setup() {
  Serial.begin(115200);
  
  // Motor control pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  
  // ENABLE PINS - MUST BE HIGH FOR MOTORS TO WORK
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);   // Enable left motor
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);  // Enable right motor
  
  Serial.println("MOTOR_TEST_READY");
  Serial.println("Send: 1=Left Forward, 2=Left Back, 3=Right Forward, 4=Right Back, 0=Stop");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Stop all first
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    
    switch(cmd) {
      case '1': // Left forward
        analogWrite(LEFT_MOTOR_FORWARD, 200);
        Serial.println("LEFT_FORWARD");
        break;
      case '2': // Left backward
        analogWrite(LEFT_MOTOR_BACKWARD, 200);
        Serial.println("LEFT_BACKWARD");
        break;
      case '3': // Right forward
        analogWrite(RIGHT_MOTOR_FORWARD, 200);
        Serial.println("RIGHT_FORWARD");
        break;
      case '4': // Right backward
        analogWrite(RIGHT_MOTOR_BACKWARD, 200);
        Serial.println("RIGHT_BACKWARD");
        break;
      case '0': // Stop
        Serial.println("STOP");
        break;
    }
  }
}
'''
    
    print("\n💾 SIMPLE MOTOR TEST FIRMWARE:")
    print("=" * 40)
    print("If the main firmware isn't working, try this simple test firmware:")
    print("1. Copy the code below to Arduino IDE")
    print("2. Upload to your Arduino")
    print("3. Open Serial Monitor")
    print("4. Send commands: 1, 2, 3, 4, 0")
    print("")
    print(firmware_code)

if __name__ == "__main__":
    print("🚀 MOTOR CONFIGURATION DIAGNOSIS")
    print("This will help identify why motors aren't spinning")
    print("")
    
    success = diagnose_motors()
    
    if not success:
        print("\n❌ Could not connect to Arduino")
        print("Check USB connection and port")
    else:
        print("\n🔧 NEXT STEPS:")
        print("1. Fix any hardware issues identified above")
        print("2. Re-run this test to verify fixes")
        print("3. If still not working, try the simple test firmware below")
        
        create_motor_test_firmware()

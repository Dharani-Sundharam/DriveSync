#!/usr/bin/env python3
"""
Check Current Arduino Firmware
Determines what firmware is currently running on Arduino
"""

import serial
import time

def check_firmware(port='/dev/ttyUSB0', baudrate=115200):
    print("🔍 Checking Current Arduino Firmware")
    print("=" * 40)
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        
        print("📡 Testing different firmware protocols...")
        
        # Test 1: OptimizedArduinoFirmware protocol
        print("\n1️⃣ Testing OptimizedArduinoFirmware protocol:")
        ser.write(b'e\r')
        time.sleep(0.1)
        response = ser.readline().decode().strip()
        print(f"   'e' command response: '{response}'")
        
        # Test 2: ROSArduinoBridge protocol  
        print("\n2️⃣ Testing ROSArduinoBridge protocol:")
        ser.write(b'e\n')
        time.sleep(0.1)
        response = ser.readline().decode().strip()
        print(f"   'e\\n' command response: '{response}'")
        
        # Test 3: Check for version info
        print("\n3️⃣ Testing version command:")
        ser.write(b'v\r')
        time.sleep(0.1)
        response = ser.readline().decode().strip()
        print(f"   'v' command response: '{response}'")
        
        # Test 4: Check raw serial output
        print("\n4️⃣ Raw serial output (5 seconds):")
        print("   Listening for automatic messages...")
        start_time = time.time()
        messages = []
        while time.time() - start_time < 5:
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                if response and response not in messages:
                    messages.append(response)
                    print(f"   📥 '{response}'")
            time.sleep(0.1)
        
        if not messages:
            print("   No automatic messages")
        
        ser.close()
        
        # Analysis
        print("\n📊 ANALYSIS:")
        print("=" * 20)
        if response == "0 0":
            print("✅ Arduino responds to 'e' command with encoder format")
            print("❌ But missing READY signal - wrong firmware version")
            print("🔧 SOLUTION: Upload OptimizedArduinoFirmware.ino")
        elif "OK" in response:
            print("✅ Arduino uses ROSArduinoBridge protocol")
            print("❌ But smooth_robot_controller expects OptimizedArduinoFirmware")
            print("🔧 SOLUTION: Upload OptimizedArduinoFirmware.ino")
        else:
            print("❌ Unknown firmware or communication issue")
            print("🔧 SOLUTION: Check connections and upload OptimizedArduinoFirmware.ino")
        
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    check_firmware()


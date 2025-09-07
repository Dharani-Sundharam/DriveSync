#!/usr/bin/env python3
"""
Simple Motor Test
Test if motors respond to commands
"""

import serial
import time

def test_motors(port=None):
    print("🔧 Simple Motor Test")
    print("=" * 30)
    
    # Use port selection menu if not specified
    if port is None:
        try:
            import sys
            import os
            sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
            from modules.port_selector import quick_port_select
            port = quick_port_select()
            if port is None:
                print("❌ No port selected")
                return
        except ImportError:
            # Fallback to simple detection
            import glob
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            if ports:
                port = ports[0]
                print(f"🔍 Using first available port: {port}")
            else:
                print("❌ No Arduino ports found")
                return
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        
        print("✅ Connected to Arduino")
        
        # Test commands
        commands = [
            ("Left motor forward", "m 200 0"),
            ("Stop", "s"),
            ("Right motor forward", "m 0 200"), 
            ("Stop", "s"),
            ("Both forward", "m 200 200"),
            ("Stop", "s")
        ]
        
        for name, cmd in commands:
            print(f"\n🧪 {name}: {cmd}")
            ser.write((cmd + '\r').encode())
            time.sleep(0.1)
            
            # Read response
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"   Arduino: '{response}'")
            
            if "Stop" not in name:
                print("   👀 Check if motors are spinning...")
                time.sleep(3)
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    test_motors()
    
    print("\n📋 MOTOR TROUBLESHOOTING CHECKLIST:")
    print("=" * 40)
    print("If motors are NOT spinning, check:")
    print("")
    print("1. 🔌 POWER SUPPLY:")
    print("   - L298N needs 12V power on VCC pin")
    print("   - Check power supply is connected and working")
    print("   - Verify ground connection (Arduino GND ↔ L298N GND)")
    print("")
    print("2. ⚡ ENABLE PINS (MOST COMMON ISSUE):")
    print("   - Arduino Pin 13 → L298N ENA (Left motor enable)")
    print("   - Arduino Pin 12 → L298N ENB (Right motor enable)")
    print("   - These MUST be connected for motors to work!")
    print("")
    print("3. 🔗 SIGNAL PINS:")
    print("   - Arduino Pin 10 → L298N IN1")
    print("   - Arduino Pin 9  → L298N IN2") 
    print("   - Arduino Pin 6  → L298N IN3")
    print("   - Arduino Pin 5  → L298N IN4")
    print("")
    print("4. 🔄 MOTOR CONNECTIONS:")
    print("   - Left motor  → L298N OUT1, OUT2")
    print("   - Right motor → L298N OUT3, OUT4")
    print("")
    print("5. 🧪 QUICK TEST:")
    print("   - Connect motor directly to 12V briefly")
    print("   - If motor spins → wiring issue")
    print("   - If motor doesn't spin → motor problem")

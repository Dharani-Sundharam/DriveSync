#!/usr/bin/env python3
"""
Quick Right Motor Fix
Simple test and fix for right motor runaway
"""

import serial
import time

def emergency_stop_right_motor(port=None):
    print("🚨 Emergency Right Motor Stop")
    
    # Auto-detect port if not specified
    if port is None:
        import glob
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        if ports:
            port = ports[0]
            print(f"Using port: {port}")
        else:
            print("❌ No Arduino ports found")
            return
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(1)
        
        # Send multiple stop commands
        for i in range(10):
            ser.write(b's\r')
            ser.write(b'm 0 0\r')
            time.sleep(0.1)
        
        ser.close()
        print("✅ Emergency stop sent")
        
    except Exception as e:
        print(f"❌ Error: {e}")

def test_individual_pins(port=None):
    print("🔍 Testing Individual Right Motor Pins")
    print("This will help identify which pin is stuck")
    
    # Auto-detect port if not specified
    if port is None:
        import glob
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        if ports:
            port = ports[0]
            print(f"Using port: {port}")
        else:
            print("❌ No Arduino ports found")
            return
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        
        # Test only right motor forward (Pin 6)
        print("\n1. Testing RIGHT FORWARD (Pin 6 only):")
        print("   Sending: m 0 50")
        ser.write(b'm 0 50\r')
        time.sleep(2)
        
        print("   Sending STOP...")
        ser.write(b's\r')
        time.sleep(1)
        
        pin6_stops = input("   Did motor stop? (y/n): ").strip().lower()
        
        # Test only right motor backward (Pin 5)  
        print("\n2. Testing RIGHT BACKWARD (Pin 5 only):")
        print("   Sending: m 0 -50")
        ser.write(b'm 0 -50\r')
        time.sleep(2)
        
        print("   Sending STOP...")
        ser.write(b's\r')
        time.sleep(1)
        
        pin5_stops = input("   Did motor stop? (y/n): ").strip().lower()
        
        ser.close()
        
        # Analysis
        print("\n📊 DIAGNOSIS:")
        if pin6_stops == 'n':
            print("❌ Pin 6 (Arduino) → IN3 (L298N) STUCK HIGH")
            print("🔧 Solutions:")
            print("   • Check Pin 6 wiring to L298N IN3")
            print("   • Try different Arduino pin for right motor forward")
            print("   • Add pull-down resistor on Pin 6")
            
        if pin5_stops == 'n':
            print("❌ Pin 5 (Arduino) → IN4 (L298N) STUCK HIGH") 
            print("🔧 Solutions:")
            print("   • Check Pin 5 wiring to L298N IN4")
            print("   • Try different Arduino pin for right motor backward")
            print("   • Add pull-down resistor on Pin 5")
            
        if pin6_stops == 'y' and pin5_stops == 'y':
            print("✅ Both pins work individually")
            print("🤔 Issue might be:")
            print("   • L298N internal problem")
            print("   • Motor mechanical issue")
            print("   • Power supply noise")
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    print("🎯 Quick Right Motor Fix")
    print("=" * 25)
    
    choice = input("Choose:\n1. Emergency stop right motor NOW\n2. Test individual pins\n3. Both\nChoice: ")
    
    if choice == '1':
        emergency_stop_right_motor()
    elif choice == '2':
        test_individual_pins()
    else:
        emergency_stop_right_motor()
        print("\nNow testing pins...")
        test_individual_pins()
    
    print("\n🛠️  QUICK HARDWARE FIXES:")
    print("1. Disconnect Pin 6 wire from L298N IN3 → motor should stop")
    print("2. Disconnect Pin 5 wire from L298N IN4 → motor should stop")  
    print("3. Swap Pin 6 ↔ Pin 10 wires → test if issue moves to left motor")
    print("4. Replace L298N module if available")

#!/usr/bin/env python3
"""
Test Corrected Encoder Hardware
Tests encoders with the corrected pin assignments:
- Left motor: D2, D3 (interrupt pins)
- Right motor: A4, A5 (polled pins)
"""

import serial
import time

def test_encoders():
    print("🔧 Testing Corrected Encoder Hardware")
    print("Pin assignments:")
    print("  Left motor:  D2, D3 (interrupt pins)")
    print("  Right motor: A4, A5 (polled pins)")
    print("=" * 50)
    
    try:
        # Connect to Arduino
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
        time.sleep(3)  # Wait for Arduino reset and setup
        
        print("✅ Connected to Arduino")
        
        # Wait for READY signal
        start_time = time.time()
        while time.time() - start_time < 5:
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"📥 Arduino says: '{response}'")
                if response == "READY":
                    print("✅ Arduino is ready with corrected firmware!")
                    break
        
        # Reset encoders
        print("\n🔄 Resetting encoders...")
        ser.write(b'r\r')
        time.sleep(0.1)
        response = ser.readline().decode().strip()
        print(f"Reset response: '{response}'")
        
        # Test encoder reading
        print("\n📊 Testing encoder reading...")
        print("Now manually turn your motors and watch the values:")
        print("(Press Ctrl+C to stop)")
        
        try:
            while True:
                ser.write(b'e\r')
                time.sleep(0.1)
                
                if ser.in_waiting:
                    response = ser.readline().decode().strip()
                    if response and ' ' in response:
                        try:
                            parts = response.split()
                            if len(parts) == 2:
                                left_enc = int(parts[0])
                                right_enc = int(parts[1])
                                print(f"📍 Left: {left_enc:6d} | Right: {right_enc:6d}")
                        except ValueError:
                            print(f"⚠️  Invalid data: '{response}'")
                    else:
                        print(f"⚠️  Unexpected response: '{response}'")
                
                time.sleep(0.2)  # 5Hz monitoring
                
        except KeyboardInterrupt:
            print("\n⚠️  Stopped by user")
            
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass
        print("🛑 Test completed")

if __name__ == "__main__":
    print("🚀 IMPORTANT: Upload the corrected Arduino firmware first!")
    print("File: OptimizedArduinoFirmware/OptimizedArduinoFirmware.ino")
    print("Then run this test.\n")
    
    input("Press Enter when you've uploaded the firmware...")
    test_encoders()

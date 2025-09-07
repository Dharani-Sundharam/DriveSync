#!/usr/bin/env python3
"""
Quick firmware check to see if new firmware is running
"""

import serial
import time
import sys

def check_firmware(port='/dev/ttyUSB0', baudrate=115200):
    print("🔍 CHECKING ARDUINO FIRMWARE")
    print("============================")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=3)
        time.sleep(2)  # Wait for reset
        
        print("Listening for Arduino messages...")
        
        # Listen for any messages for 5 seconds
        start_time = time.time()
        messages = []
        
        while time.time() - start_time < 5:
            if ser.in_waiting:
                try:
                    msg = ser.readline().decode().strip()
                    if msg:
                        messages.append(msg)
                        print(f"📨 Arduino: {msg}")
                except:
                    pass
        
        if "READY" in messages:
            print("✅ NEW FIRMWARE DETECTED! (READY signal received)")
        else:
            print("⚠️  OLD FIRMWARE LIKELY RUNNING (No READY signal)")
            print("   Messages received:", messages if messages else "None")
        
        # Test a simple command
        print("\n🧪 Testing motor command...")
        ser.write(b'm 50 50\r')
        time.sleep(0.1)
        
        response = ""
        if ser.in_waiting:
            response = ser.readline().decode().strip()
        
        print(f"Response to 'm 50 50': {response}")
        
        # Stop motors
        ser.write(b's\r')
        time.sleep(0.1)
        if ser.in_waiting:
            stop_response = ser.readline().decode().strip()
            print(f"Response to 's': {stop_response}")
        
        ser.close()
        
        return "READY" in messages
        
    except Exception as e:
        print(f"❌ Error connecting to Arduino: {e}")
        return False

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    firmware_ok = check_firmware(port)
    
    if firmware_ok:
        print("\n✅ Firmware appears to be updated!")
        print("   If directions are still wrong, it's likely a wiring issue.")
    else:
        print("\n❌ Firmware may not be updated!")
        print("   Please re-upload the OptimizedArduinoFirmware.ino")

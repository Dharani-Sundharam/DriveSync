#!/usr/bin/env python3
"""
Arduino Firmware Test Script
Tests the new Arduino firmware for basic functionality
"""

import serial
import time
import sys

def test_arduino(port='/dev/ttyUSB0', baudrate=57600):
    """Test Arduino firmware functionality"""
    
    print(f"Connecting to Arduino on {port} at {baudrate} baud...")
    
    try:
        # Connect to Arduino
        arduino = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)  # Wait for Arduino to reset
        
        print("Connected! Testing firmware...")
        
        # Clear any initial messages
        arduino.flushInput()
        
        # Test 1: Reset encoders
        print("\n1. Testing encoder reset...")
        arduino.write(b'r\r')
        response = arduino.readline().decode().strip()
        print(f"Reset response: {response}")
        
        # Test 2: Read initial encoder values
        print("\n2. Reading initial encoder values...")
        arduino.write(b'e\r')
        response = arduino.readline().decode().strip()
        print(f"Initial encoders: {response}")
        
        # Test 3: Test motor commands
        print("\n3. Testing motor commands...")
        
        # Test left motor forward
        print("Testing left motor forward (speed 100)...")
        arduino.write(b'm 100 0\r')
        response = arduino.readline().decode().strip()
        print(f"Left motor forward response: {response}")
        time.sleep(2)
        
        # Stop motors
        arduino.write(b'm 0 0\r')
        response = arduino.readline().decode().strip()
        print(f"Stop motors response: {response}")
        time.sleep(1)
        
        # Test right motor forward
        print("Testing right motor forward (speed 100)...")
        arduino.write(b'm 0 100\r')
        response = arduino.readline().decode().strip()
        print(f"Right motor forward response: {response}")
        time.sleep(2)
        
        # Stop motors
        arduino.write(b'm 0 0\r')
        response = arduino.readline().decode().strip()
        print(f"Stop motors response: {response}")
        time.sleep(1)
        
        # Test both motors forward
        print("Testing both motors forward (speed 80)...")
        arduino.write(b'm 80 80\r')
        response = arduino.readline().decode().strip()
        print(f"Both motors forward response: {response}")
        time.sleep(2)
        
        # Stop motors
        arduino.write(b'm 0 0\r')
        response = arduino.readline().decode().strip()
        print(f"Stop motors response: {response}")
        time.sleep(1)
        
        # Test 4: Read encoders after movement
        print("\n4. Reading encoder values after movement...")
        arduino.write(b'e\r')
        response = arduino.readline().decode().strip()
        print(f"Encoders after movement: {response}")
        
        # Test 5: Get system info
        print("\n5. Getting system info...")
        arduino.write(b'i\r')
        response = arduino.readline().decode().strip()
        print(f"System info: {response}")
        
        # Test 6: Test backward movement
        print("\n6. Testing backward movement...")
        arduino.write(b'm -80 -80\r')
        response = arduino.readline().decode().strip()
        print(f"Both motors backward response: {response}")
        time.sleep(2)
        
        # Stop motors
        arduino.write(b'm 0 0\r')
        response = arduino.readline().decode().strip()
        print(f"Final stop response: {response}")
        
        # Final encoder reading
        print("\n7. Final encoder reading...")
        arduino.write(b'e\r')
        response = arduino.readline().decode().strip()
        print(f"Final encoders: {response}")
        
        print("\n✅ Arduino firmware test completed successfully!")
        print("\nIf you saw motor movement and encoder values changing,")
        print("your Arduino firmware is working correctly!")
        
    except serial.SerialException as e:
        print(f"❌ Serial connection error: {e}")
        print("Make sure:")
        print("1. Arduino is connected to the correct port")
        print("2. You have permission to access the serial port")
        print("3. No other program is using the serial port")
        return False
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False
        
    finally:
        if 'arduino' in locals():
            arduino.close()
    
    return True

def interactive_test(port='/dev/ttyUSB0', baudrate=57600):
    """Interactive test mode - send commands manually"""
    
    print(f"Interactive Arduino Test Mode")
    print(f"Connecting to Arduino on {port} at {baudrate} baud...")
    print("Commands: e (encoders), r (reset), m <left> <right> (motors), s (stop), q (quit)")
    
    try:
        arduino = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        
        print("Connected! Type commands (or 'q' to quit):")
        
        while True:
            command = input("> ").strip()
            
            if command.lower() == 'q':
                break
            
            if command:
                arduino.write((command + '\r').encode())
                time.sleep(0.1)
                
                # Read response
                if arduino.in_waiting > 0:
                    response = arduino.readline().decode().strip()
                    print(f"Response: {response}")
                else:
                    print("No response")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'arduino' in locals():
            arduino.close()

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print("Arduino Firmware Test")
    print("====================")
    print("Choose test mode:")
    print("1. Automatic test")
    print("2. Interactive test")
    
    choice = input("Enter choice (1 or 2): ").strip()
    
    if choice == "1":
        test_arduino(port)
    elif choice == "2":
        interactive_test(port)
    else:
        print("Invalid choice. Running automatic test...")
        test_arduino(port)

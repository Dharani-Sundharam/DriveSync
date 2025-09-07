#!/usr/bin/env python3
"""
Arduino Port Auto-Detection Module
Automatically finds and connects to Arduino on USB0 or USB1
"""

import serial
import time
import glob
import sys
import os

class ArduinoPortDetector:
    """Automatically detects and connects to Arduino on available USB ports"""
    
    def __init__(self, baudrate=115200, timeout=2):
        self.baudrate = baudrate
        self.timeout = timeout
        self.port = None
        self.serial_connection = None
    
    def find_arduino_ports(self):
        """Find all potential Arduino serial ports"""
        possible_ports = []
        
        # Common Arduino port patterns on Linux
        port_patterns = [
            '/dev/ttyUSB*',
            '/dev/ttyACM*',
            '/dev/ttyS*'
        ]
        
        for pattern in port_patterns:
            ports = glob.glob(pattern)
            possible_ports.extend(ports)
        
        # Sort ports to prioritize USB0 over USB1, etc.
        possible_ports.sort()
        
        print(f"🔍 Found potential Arduino ports: {possible_ports}")
        return possible_ports
    
    def test_arduino_port(self, port):
        """Test if a port has an Arduino with our firmware"""
        try:
            print(f"   Testing {port}...")
            ser = serial.Serial(port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # Wait for Arduino reset
            
            # Clear any initial messages
            start_time = time.time()
            ready_found = False
            
            # Look for READY signal or encoder data
            while (time.time() - start_time) < 3:
                if ser.in_waiting:
                    try:
                        response = ser.readline().decode().strip()
                        if response == "READY":
                            ready_found = True
                            print(f"   ✅ Found Arduino with READY signal on {port}")
                            break
                        elif response and ' ' in response and response.replace(' ', '').replace('-', '').isdigit():
                            # Looks like encoder data (e.g., "0 0", "123 -456")
                            print(f"   ✅ Found Arduino with encoder data on {port}: '{response}'")
                            ready_found = True
                            break
                    except UnicodeDecodeError:
                        continue
                time.sleep(0.1)
            
            # Test basic command if no automatic signals
            if not ready_found:
                print(f"   🧪 Testing command response on {port}...")
                ser.write(b'e\r')
                time.sleep(0.5)
                
                if ser.in_waiting:
                    response = ser.readline().decode().strip()
                    if response and (' ' in response or response.isdigit()):
                        print(f"   ✅ Arduino responds to commands on {port}: '{response}'")
                        ready_found = True
            
            ser.close()
            return ready_found
            
        except Exception as e:
            print(f"   ❌ Failed to connect to {port}: {e}")
            try:
                ser.close()
            except:
                pass
            return False
    
    def auto_connect(self):
        """Automatically find and connect to Arduino"""
        print("🔍 Auto-detecting Arduino port...")
        
        possible_ports = self.find_arduino_ports()
        
        if not possible_ports:
            print("❌ No serial ports found!")
            print("   Make sure Arduino is connected via USB")
            return None
        
        # Test each port
        for port in possible_ports:
            if self.test_arduino_port(port):
                print(f"✅ Arduino found on {port}")
                self.port = port
                
                # Establish connection
                try:
                    self.serial_connection = serial.Serial(port, self.baudrate, timeout=self.timeout)
                    time.sleep(2)  # Wait for Arduino reset
                    print(f"✅ Connected to Arduino on {port}")
                    return self.serial_connection
                except Exception as e:
                    print(f"❌ Failed to establish connection to {port}: {e}")
                    continue
        
        print("❌ No Arduino found on any port!")
        return None
    
    def get_port(self):
        """Get the detected port name"""
        return self.port
    
    def get_connection(self):
        """Get the serial connection"""
        return self.serial_connection
    
    def close(self):
        """Close the serial connection"""
        if self.serial_connection:
            try:
                self.serial_connection.close()
                print(f"🔌 Disconnected from {self.port}")
            except:
                pass

def find_arduino_port(baudrate=115200, timeout=2):
    """Simple function to find Arduino port"""
    detector = ArduinoPortDetector(baudrate, timeout)
    connection = detector.auto_connect()
    
    if connection:
        return detector.get_port(), connection
    else:
        return None, None

# Test function
if __name__ == "__main__":
    print("🤖 Arduino Port Auto-Detection Test")
    print("=" * 40)
    
    port, connection = find_arduino_port()
    
    if port and connection:
        print(f"\n✅ SUCCESS: Arduino found on {port}")
        
        # Test basic communication
        print("🧪 Testing basic communication...")
        try:
            connection.write(b'e\r')
            time.sleep(0.2)
            
            if connection.in_waiting:
                response = connection.readline().decode().strip()
                print(f"📥 Arduino response: '{response}'")
            else:
                print("⚠️  No response to test command")
                
        except Exception as e:
            print(f"❌ Communication test failed: {e}")
        
        connection.close()
        
    else:
        print("\n❌ FAILED: No Arduino found")
        print("\nTroubleshooting:")
        print("1. Check USB cable connection")
        print("2. Verify Arduino is powered on")
        print("3. Make sure correct firmware is uploaded")
        print("4. Try different USB ports")
        print("5. Check with: ls /dev/ttyUSB* /dev/ttyACM*")

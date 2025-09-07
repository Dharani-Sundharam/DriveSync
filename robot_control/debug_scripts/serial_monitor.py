#!/usr/bin/env python3
"""
Serial Monitor using PySerial
Reads and displays all data from Arduino serial port
"""

import serial
import time
import sys
from datetime import datetime

def serial_monitor(port='/dev/ttyUSB0', baudrate=115200, timeout=1):
    print("🔍 Arduino Serial Monitor")
    print("=" * 40)
    print(f"Port: {port}")
    print(f"Baud Rate: {baudrate}")
    print(f"Timeout: {timeout}s")
    print("=" * 40)
    print("Press Ctrl+C to stop monitoring")
    print()
    
    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"✅ Connected to {port}")
        print(f"⏰ Started monitoring at {datetime.now().strftime('%H:%M:%S')}")
        print("-" * 40)
        
        # Clear any initial buffer
        time.sleep(0.5)
        ser.flushInput()
        
        line_count = 0
        
        while True:
            try:
                # Read line from Arduino
                if ser.in_waiting > 0:
                    raw_data = ser.readline()
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    
                    try:
                        # Try to decode as string
                        data = raw_data.decode('utf-8').strip()
                        if data:
                            line_count += 1
                            print(f"[{timestamp}] #{line_count:03d}: '{data}'")
                    except UnicodeDecodeError:
                        # Show raw bytes if not valid UTF-8
                        print(f"[{timestamp}] #{line_count:03d}: RAW: {raw_data}")
                        line_count += 1
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.001)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ Read error: {e}")
                time.sleep(0.1)
        
    except serial.SerialException as e:
        print(f"❌ Serial connection error: {e}")
        print("\nTroubleshooting:")
        print("1. Check if Arduino is connected")
        print("2. Verify the port name (ls /dev/ttyUSB* /dev/ttyACM*)")
        print("3. Check if another program is using the port")
        return False
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False
        
    finally:
        try:
            ser.close()
            print(f"\n🛑 Monitoring stopped at {datetime.now().strftime('%H:%M:%S')}")
            print(f"📊 Total lines received: {line_count}")
        except:
            pass
    
    return True

def interactive_serial_monitor(port='/dev/ttyUSB0', baudrate=115200):
    """Interactive mode - can send commands while monitoring"""
    print("🔍 Interactive Arduino Serial Monitor")
    print("=" * 40)
    print("Type commands and press Enter to send")
    print("Commands to try:")
    print("  e     - Request encoder data")
    print("  r     - Reset encoders") 
    print("  s     - Stop motors")
    print("  m 50 -50  - Move motors (left=50, right=-50)")
    print("  quit  - Exit monitor")
    print("=" * 40)
    
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print(f"✅ Connected to {port}")
        
        # Start monitoring in background
        import threading
        monitoring = True
        
        def monitor_thread():
            line_count = 0
            while monitoring:
                try:
                    if ser.in_waiting > 0:
                        data = ser.readline().decode('utf-8').strip()
                        if data:
                            line_count += 1
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            print(f"\n📥 [{timestamp}] Arduino: '{data}'")
                            print(">>> ", end='', flush=True)
                    time.sleep(0.01)
                except:
                    pass
        
        monitor = threading.Thread(target=monitor_thread, daemon=True)
        monitor.start()
        
        print("\n🎮 Interactive mode started. Type commands:")
        
        while True:
            try:
                command = input(">>> ").strip()
                
                if command.lower() == 'quit':
                    break
                elif command:
                    # Send command to Arduino
                    ser.write((command + '\r').encode())
                    print(f"📤 Sent: '{command}'")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ Error: {e}")
        
        monitoring = False
        ser.close()
        print("\n🛑 Interactive monitor stopped")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    baudrate = 115200
    interactive = False
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        if sys.argv[1] == '-i' or sys.argv[1] == '--interactive':
            interactive = True
            if len(sys.argv) > 2:
                port = sys.argv[2]
        else:
            port = sys.argv[1]
    
    if len(sys.argv) > 2 and not interactive:
        try:
            baudrate = int(sys.argv[2])
        except ValueError:
            print("Invalid baud rate, using 115200")
    
    print(f"🚀 Starting {'interactive' if interactive else 'passive'} serial monitor...")
    
    if interactive:
        interactive_serial_monitor(port, baudrate)
    else:
        serial_monitor(port, baudrate)


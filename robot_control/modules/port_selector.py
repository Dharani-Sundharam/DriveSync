#!/usr/bin/env python3
"""
Arduino Port Selector
Simple menu to choose between USB0 or USB1
"""

import os
import glob

def select_arduino_port():
    """Show menu to select Arduino port"""
    print("🔌 Arduino Port Selection")
    print("=" * 30)
    
    # Check which ports are available
    available_ports = []
    usb0_available = os.path.exists('/dev/ttyUSB0')
    usb1_available = os.path.exists('/dev/ttyUSB1')
    acm_ports = glob.glob('/dev/ttyACM*')
    
    if usb0_available:
        available_ports.append(('/dev/ttyUSB0', 'USB0'))
    if usb1_available:
        available_ports.append(('/dev/ttyUSB1', 'USB1'))
    for acm in acm_ports:
        available_ports.append((acm, acm.split('/')[-1]))
    
    if not available_ports:
        print("❌ No Arduino ports found!")
        print("   Make sure Arduino is connected via USB")
        return None
    
    print("Available Arduino ports:")
    for i, (port, name) in enumerate(available_ports, 1):
        status = "✅" if os.path.exists(port) else "❌"
        print(f"   {i}. {name} ({port}) {status}")
    
    print()
    
    # Get user choice
    while True:
        try:
            choice = input(f"Select port (1-{len(available_ports)}): ").strip()
            
            if choice.isdigit():
                choice_num = int(choice)
                if 1 <= choice_num <= len(available_ports):
                    selected_port = available_ports[choice_num - 1][0]
                    selected_name = available_ports[choice_num - 1][1]
                    print(f"✅ Selected: {selected_name} ({selected_port})")
                    return selected_port
                else:
                    print(f"❌ Please enter a number between 1 and {len(available_ports)}")
            else:
                print("❌ Please enter a valid number")
                
        except KeyboardInterrupt:
            print("\n❌ Selection cancelled")
            return None
        except Exception as e:
            print(f"❌ Error: {e}")

def quick_port_select():
    """Quick selection with common defaults"""
    print("🚀 Quick Arduino Port Selection")
    print("=" * 35)
    
    # Check availability
    usb0_exists = os.path.exists('/dev/ttyUSB0')
    usb1_exists = os.path.exists('/dev/ttyUSB1')
    
    print("Choose your Arduino port:")
    print(f"1. USB0 (/dev/ttyUSB0) {'✅' if usb0_exists else '❌ Not found'}")
    print(f"2. USB1 (/dev/ttyUSB1) {'✅' if usb1_exists else '❌ Not found'}")
    print("3. Auto-detect")
    print("4. Custom port")
    print()
    
    while True:
        try:
            choice = input("Your choice (1-4): ").strip()
            
            if choice == '1':
                if usb0_exists:
                    print("✅ Selected: USB0 (/dev/ttyUSB0)")
                    return '/dev/ttyUSB0'
                else:
                    print("❌ USB0 not found. Try another option.")
                    continue
                    
            elif choice == '2':
                if usb1_exists:
                    print("✅ Selected: USB1 (/dev/ttyUSB1)")
                    return '/dev/ttyUSB1'
                else:
                    print("❌ USB1 not found. Try another option.")
                    continue
                    
            elif choice == '3':
                print("🔍 Auto-detecting...")
                if usb0_exists:
                    print("✅ Auto-selected: USB0 (/dev/ttyUSB0)")
                    return '/dev/ttyUSB0'
                elif usb1_exists:
                    print("✅ Auto-selected: USB1 (/dev/ttyUSB1)")
                    return '/dev/ttyUSB1'
                else:
                    print("❌ No USB ports found")
                    return None
                    
            elif choice == '4':
                custom_port = input("Enter custom port (e.g., /dev/ttyACM0): ").strip()
                if os.path.exists(custom_port):
                    print(f"✅ Selected: {custom_port}")
                    return custom_port
                else:
                    print(f"❌ Port {custom_port} not found")
                    continue
            else:
                print("❌ Please enter 1, 2, 3, or 4")
                
        except KeyboardInterrupt:
            print("\n❌ Selection cancelled")
            return None

if __name__ == "__main__":
    print("🧪 Testing Port Selector")
    port = quick_port_select()
    if port:
        print(f"\n🎯 You selected: {port}")
    else:
        print("\n❌ No port selected")

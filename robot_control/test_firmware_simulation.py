#!/usr/bin/env python3
"""
Firmware Test with Simulated Key Presses
Tests the current Arduino firmware by simulating robot movements
"""

import serial
import time
import threading

class FirmwareTester:
    def __init__(self, port='/dev/ttyUSB1', baudrate=57600):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = True
        
        print(f"🔧 Testing firmware on {port} at {baudrate} baud")
        
    def connect(self):
        """Connect to Arduino"""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for reset
            print("✅ Connected to Arduino")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
            
    def send_command(self, command):
        """Send command and get response"""
        if self.serial_port:
            try:
                print(f"📤 Sending: {command}")
                self.serial_port.write(f"{command}\r".encode())
                time.sleep(0.1)  # Wait for response
                
                # Read all available responses
                responses = []
                while self.serial_port.in_waiting:
                    response = self.serial_port.readline().decode().strip()
                    if response:
                        responses.append(response)
                        
                if responses:
                    print(f"📥 Responses: {responses}")
                else:
                    print("📥 No response")
                    
                return responses
            except Exception as e:
                print(f"❌ Command error: {e}")
                return []
        return []
        
    def test_encoder_reading(self):
        """Test encoder reading"""
        print("\n🔍 TESTING ENCODER READING")
        print("=" * 40)
        
        for i in range(5):
            print(f"\nTest {i+1}/5:")
            responses = self.send_command("e")
            time.sleep(0.5)
            
    def test_motor_commands(self):
        """Test motor commands with simulated key presses"""
        print("\n🎮 TESTING MOTOR COMMANDS (Simulated Key Presses)")
        print("=" * 50)
        
        # Test movements like the GUI would send
        test_commands = [
            ("FORWARD (W key)", "m 120 120"),
            ("STOP", "m 0 0"),
            ("BACKWARD (S key)", "m -120 -120"), 
            ("STOP", "m 0 0"),
            ("TURN LEFT (A key)", "m -100 100"),
            ("STOP", "m 0 0"),
            ("TURN RIGHT (D key)", "m 100 -100"),
            ("STOP", "m 0 0"),
        ]
        
        for description, command in test_commands:
            print(f"\n🎯 {description}")
            responses = self.send_command(command)
            
            # Read encoder values after each command
            time.sleep(0.5)  # Let motors run briefly
            print("   📊 Reading encoders:")
            encoder_responses = self.send_command("e")
            
            time.sleep(1)  # Wait between commands
            
    def test_reset(self):
        """Test encoder reset"""
        print("\n🔄 TESTING ENCODER RESET")
        print("=" * 30)
        
        print("Before reset:")
        self.send_command("e")
        
        print("\nResetting encoders:")
        self.send_command("r")
        
        print("\nAfter reset:")
        self.send_command("e")
        
    def continuous_monitoring(self, duration=10):
        """Monitor encoder values continuously"""
        print(f"\n📡 CONTINUOUS MONITORING ({duration} seconds)")
        print("=" * 40)
        print("Watching encoder values while motors are stopped...")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            responses = self.send_command("e")
            time.sleep(0.2)  # 5Hz monitoring
            
    def run_full_test(self):
        """Run complete firmware test"""
        if not self.connect():
            return
            
        try:
            # Test sequence
            self.test_encoder_reading()
            self.test_reset()
            self.test_motor_commands()
            self.continuous_monitoring(5)
            
            print("\n✅ FIRMWARE TEST COMPLETE!")
            print("\n📋 ANALYSIS:")
            print("- Check if encoder values are changing when motors run")
            print("- Look for 'OK' responses vs numeric encoder data")
            print("- Verify motor commands are being accepted")
            
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Clean up"""
        if self.serial_port:
            print("\n🛑 Stopping all motors...")
            self.send_command("m 0 0")
            time.sleep(0.5)
            self.serial_port.close()
        print("🛑 Test completed")

def main():
    print("🚀 Arduino Firmware Test with Simulated Key Presses")
    print("This will test your current firmware by simulating robot movements\n")
    
    # Test with both possible baud rates
    for baudrate in [57600, 115200]:
        print(f"\n{'='*60}")
        print(f"TESTING AT {baudrate} BAUD RATE")
        print(f"{'='*60}")
        
        tester = FirmwareTester(baudrate=baudrate)
        if tester.connect():
            tester.run_full_test()
            break  # If connection successful, don't try other baud rate
        else:
            print(f"❌ Failed at {baudrate} baud, trying next...")

if __name__ == "__main__":
    main()

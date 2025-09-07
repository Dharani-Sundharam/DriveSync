#!/usr/bin/env python3
"""
Motor Direction Calibration Test
This script helps you verify and calibrate the left/right motor directions
"""

import serial
import time

class MotorDirectionTester:
    def __init__(self, port='/dev/ttyUSB0', baudrate=57600):
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.connect_to_arduino()
        
    def connect_to_arduino(self):
        """Connect to Arduino via serial"""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            
            # Reset encoders
            self.serial_port.write(b'r\r')
            response = self.serial_port.readline().decode().strip()
            print(f"✅ Connected to Arduino on {self.port}")
            print(f"Reset response: {response}")
            
        except Exception as e:
            print(f"❌ Failed to connect to Arduino: {e}")
            self.serial_port = None
            
    def send_motor_command(self, left_speed, right_speed):
        """Send motor speed command to Arduino"""
        if self.serial_port:
            try:
                command = f'm {int(left_speed)} {int(right_speed)}\r'
                self.serial_port.write(command.encode())
                print(f"Sent: Left={left_speed}, Right={right_speed}")
                return True
            except Exception as e:
                print(f"Error sending motor command: {e}")
                return False
        return False
        
    def read_encoders(self):
        """Read encoder values from Arduino"""
        if self.serial_port:
            try:
                self.serial_port.write(b'e\r')
                response = self.serial_port.readline().decode().strip()
                if response and ' ' in response:
                    parts = response.split()
                    if len(parts) >= 2:
                        try:
                            encoder_a = int(parts[0])  # Motor A
                            encoder_b = int(parts[1])  # Motor B
                            return encoder_a, encoder_b
                        except ValueError:
                            pass
            except Exception as e:
                print(f"Error reading encoders: {e}")
        return None, None
        
    def test_individual_motors(self):
        """Test each motor individually to verify directions"""
        print("\n🔧 INDIVIDUAL MOTOR DIRECTION TEST")
        print("=" * 50)
        
        # Reset encoders
        self.serial_port.write(b'r\r')
        time.sleep(0.5)
        
        tests = [
            ("Left Motor Forward (Motor A positive)", 100, 0),
            ("Left Motor Backward (Motor A negative)", -100, 0),
            ("Right Motor Forward (Motor B positive)", 0, 100),
            ("Right Motor Backward (Motor B negative)", 0, -100),
        ]
        
        for test_name, left_speed, right_speed in tests:
            print(f"\n--- {test_name} ---")
            
            # Get initial encoder values
            initial_a, initial_b = self.read_encoders()
            if initial_a is None:
                print("❌ Failed to read initial encoders")
                continue
                
            print(f"Initial encoders: A={initial_a}, B={initial_b}")
            
            # Run motor
            self.send_motor_command(left_speed, right_speed)
            time.sleep(2)  # Run for 2 seconds
            
            # Stop motor
            self.send_motor_command(0, 0)
            time.sleep(0.5)
            
            # Get final encoder values
            final_a, final_b = self.read_encoders()
            if final_a is None:
                print("❌ Failed to read final encoders")
                continue
                
            print(f"Final encoders: A={final_a}, B={final_b}")
            
            # Calculate changes
            delta_a = final_a - initial_a
            delta_b = final_b - initial_b
            
            print(f"Encoder changes: ΔA={delta_a}, ΔB={delta_b}")
            
            # Analyze results
            if left_speed > 0:  # Left motor should move forward
                if delta_a > 0:
                    print("✅ Left motor moving forward correctly")
                elif delta_a < 0:
                    print("❌ Left motor moving backward (direction reversed)")
                else:
                    print("⚠️  Left motor not moving")
            elif left_speed < 0:  # Left motor should move backward
                if delta_a < 0:
                    print("✅ Left motor moving backward correctly")
                elif delta_a > 0:
                    print("❌ Left motor moving forward (direction reversed)")
                else:
                    print("⚠️  Left motor not moving")
                    
            if right_speed > 0:  # Right motor should move forward
                if delta_b > 0:
                    print("✅ Right motor moving forward correctly")
                elif delta_b < 0:
                    print("❌ Right motor moving backward (direction reversed)")
                else:
                    print("⚠️  Right motor not moving")
            elif right_speed < 0:  # Right motor should move backward
                if delta_b < 0:
                    print("✅ Right motor moving backward correctly")
                elif delta_b > 0:
                    print("❌ Right motor moving forward (direction reversed)")
                else:
                    print("⚠️  Right motor not moving")
            
            time.sleep(1)
            
    def test_robot_movements(self):
        """Test complete robot movements"""
        print("\n🤖 ROBOT MOVEMENT TEST")
        print("=" * 50)
        
        movements = [
            ("Forward", 100, 100, "Both motors should move forward"),
            ("Backward", -100, -100, "Both motors should move backward"),
            ("Turn Left", -100, 100, "Left backward, Right forward"),
            ("Turn Right", 100, -100, "Left forward, Right backward"),
        ]
        
        for movement_name, left_speed, right_speed, description in movements:
            print(f"\n--- {movement_name} ---")
            print(f"Description: {description}")
            print(f"Command: Left={left_speed}, Right={right_speed}")
            
            input("Press Enter to start this movement test...")
            
            # Get initial encoder values
            initial_a, initial_b = self.read_encoders()
            print(f"Initial encoders: A={initial_a}, B={initial_b}")
            
            # Run movement
            self.send_motor_command(left_speed, right_speed)
            time.sleep(3)  # Run for 3 seconds
            
            # Stop
            self.send_motor_command(0, 0)
            
            # Get final encoder values
            final_a, final_b = self.read_encoders()
            print(f"Final encoders: A={final_a}, B={final_b}")
            
            # Calculate changes
            delta_a = final_a - initial_a
            delta_b = final_b - initial_b
            
            print(f"Encoder changes: ΔA={delta_a}, ΔB={delta_b}")
            print("Watch the robot and verify it moved as expected!")
            
            time.sleep(2)
            
    def cleanup(self):
        """Clean up resources"""
        if self.serial_port:
            self.send_motor_command(0, 0)  # Stop motors
            self.serial_port.close()
        print("🛑 Motor direction tester cleaned up")

def main():
    print("🚀 Motor Direction Calibration Test")
    print("This will help you verify left/right motor directions")
    
    tester = MotorDirectionTester()
    
    if tester.serial_port is None:
        print("❌ Cannot connect to Arduino. Exiting.")
        return
        
    try:
        print("\nChoose test mode:")
        print("1. Individual motor test (automatic)")
        print("2. Robot movement test (manual)")
        print("3. Both tests")
        
        choice = input("Enter choice (1, 2, or 3): ").strip()
        
        if choice in ['1', '3']:
            tester.test_individual_motors()
            
        if choice in ['2', '3']:
            tester.test_robot_movements()
            
        print("\n✅ Testing complete!")
        print("Based on the results, you may need to:")
        print("1. Swap motor wires if directions are reversed")
        print("2. Update the motor mapping in robot_controller.py")
        
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        tester.cleanup()
        print("👋 Goodbye!")

if __name__ == "__main__":
    main()

# Arduino L298N Motor Driver - YOUR WIRING CONFIGURATION

## Your Actual Wiring Setup

### L298N to Arduino Connections
| L298N Pin | Arduino Pin | Function |
|-----------|-------------|----------|
| IN1       | Pin 10      | Left Motor Forward |
| IN2       | Pin 9       | Left Motor Backward |
| IN3       | Pin 6       | Right Motor Forward |
| IN4       | Pin 5       | Right Motor Backward |
| ENA       | Pin 13      | Left Motor Enable (PWM) |
| ENB       | Pin 12      | Right Motor Enable (PWM) |

### Encoder Connections (Recommended)
| Encoder Pin | Arduino Pin | Function |
|-------------|-------------|----------|
| Left Enc A  | Pin 2       | Interrupt 0 |
| Left Enc B  | Pin 3       | Interrupt 1 |
| Right Enc A | Pin A4      | Analog Pin (PC4) |
| Right Enc B | Pin A5      | Analog Pin (PC5) |
| VCC (both)  | 5V          | Power |
| GND (both)  | GND         | Ground |

## Complete Wiring Diagram

```
Arduino Uno/Mega          L298N Motor Driver
================          ==================
Pin 10 (PWM)     -------> IN1 (Left Forward)
Pin 9  (PWM)     -------> IN2 (Left Backward)
Pin 6  (PWM)     -------> IN3 (Right Forward)
Pin 5  (PWM)     -------> IN4 (Right Backward)
Pin 13           -------> ENA (Left Enable)
Pin 12           -------> ENB (Right Enable)
5V               -------> 5V
GND              -------> GND

Left Encoder             Right Encoder
============             =============
Channel A -----> Pin 2   Channel A -----> Pin A4
Channel B -----> Pin 3   Channel B -----> Pin A5
VCC       -----> 5V      VCC       -----> 5V
GND       -----> GND     GND       -----> GND

External 12V Power Supply
========================
Positive -------> L298N 12V
Negative -------> L298N GND (also connect to Arduino GND)
```

## Motor Direction Logic (Your Wiring)

### Left Motor
- **Forward**: IN1 (Pin 10) = HIGH, IN2 (Pin 9) = LOW
- **Backward**: IN1 (Pin 10) = LOW, IN2 (Pin 9) = HIGH
- **Stop**: IN1 (Pin 10) = LOW, IN2 (Pin 9) = LOW

### Right Motor
- **Forward**: IN3 (Pin 6) = HIGH, IN4 (Pin 5) = LOW
- **Backward**: IN3 (Pin 6) = LOW, IN4 (Pin 5) = HIGH
- **Stop**: IN3 (Pin 6) = LOW, IN4 (Pin 5) = LOW

## Testing Your Setup

### 1. Upload the Custom Firmware
Upload the `ROSArduinoBridge_YourWiring.ino` firmware to your Arduino.

### 2. Test Commands via Serial Monitor
Open Arduino IDE Serial Monitor (57600 baud) and try these commands:

```
t          # Run automatic motor test sequence
e          # Read encoder values
r          # Reset encoders
m 100 0    # Left motor forward, right motor stop
m 0 100    # Left motor stop, right motor forward
m 100 100  # Both motors forward
m -100 -100 # Both motors backward
s          # Stop all motors
i          # Get system info
```

### 3. Test with Python Script
```bash
python3 /home/dharani/Desktop/DriveSync/V2V-main/ros_arduino_bridge/test_arduino.py
```

### 4. Expected Motor Movements
When you send `m 100 0`:
- Left motor should spin forward
- Right motor should be stopped

When you send `m 0 100`:
- Left motor should be stopped  
- Right motor should spin forward

When you send `m 100 100`:
- Both motors should spin forward (robot moves forward)

## Troubleshooting Your Wiring

### If Left Motor Spins Wrong Direction
- Swap the motor wires connected to OUT1 and OUT2 on the L298N
- OR modify the firmware to invert left motor logic

### If Right Motor Spins Wrong Direction
- Swap the motor wires connected to OUT3 and OUT4 on the L298N
- OR modify the firmware to invert right motor logic

### If Motors Don't Move At All
1. Check 12V power supply is connected to L298N
2. Verify ENA and ENB jumpers are in place on L298N
3. Check all wire connections
4. Try higher PWM values (150-200 instead of 100)

### If Encoders Don't Count
1. Check encoder power (5V and GND)
2. Verify encoder A and B channels are connected correctly
3. Test by manually rotating motors and checking encoder counts with `e` command

## ROS2 Integration

After confirming the Arduino firmware works correctly, test with ROS2:

```bash
# Stop any running ROS processes first
pkill -f ros2

# Test the Arduino interface
source /opt/ros/jazzy/setup.bash
source /home/dharani/Desktop/DriveSync/V2V-main/install/setup.bash
ros2 run differential_drive_robot arduino_interface.py --ros-args -p port:=/dev/ttyUSB0

# In another terminal, test encoder reading
ros2 topic echo /encoder_ticks

# In another terminal, test motor commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}' --once
```

The "Invalid encoder response: OK" error should now be resolved with the corrected firmware!

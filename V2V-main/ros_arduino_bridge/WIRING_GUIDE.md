# Arduino L298N Motor Driver and Encoder Wiring Guide

## Hardware Requirements
- Arduino Uno/Mega
- L298N Motor Driver Module
- 2x DC Motors with Quadrature Encoders
- 12V Power Supply for motors
- Jumper wires

## L298N Motor Driver Connections

### Power Connections
- **12V**: Connect to external 12V power supply positive
- **GND**: Connect to external power supply ground AND Arduino GND
- **5V**: Can be used to power Arduino (if jumper is in place) OR connect to Arduino 5V

### Motor Connections
- **OUT1 & OUT2**: Left Motor terminals
- **OUT3 & OUT4**: Right Motor terminals

### Arduino to L298N Control Connections
| L298N Pin | Arduino Pin | Function |
|-----------|-------------|----------|
| IN1       | Pin 10      | Left Motor Forward |
| IN2       | Pin 6       | Left Motor Backward |
| IN3       | Pin 9       | Right Motor Forward |
| IN4       | Pin 5       | Right Motor Backward |
| ENA       | Pin 13      | Left Motor Enable (PWM) |
| ENB       | Pin 12      | Right Motor Enable (PWM) |

## Encoder Connections

### Left Encoder (Using Hardware Interrupts)
| Encoder Pin | Arduino Pin | Function |
|-------------|-------------|----------|
| Channel A   | Pin 2       | Interrupt 0 |
| Channel B   | Pin 3       | Interrupt 1 |
| VCC         | 5V          | Power |
| GND         | GND         | Ground |

### Right Encoder (Using Pin Change Interrupts)
| Encoder Pin | Arduino Pin | Function |
|-------------|-------------|----------|
| Channel A   | Pin A4      | Analog Pin (PC4) |
| Channel B   | Pin A5      | Analog Pin (PC5) |
| VCC         | 5V          | Power |
| GND         | GND         | Ground |

## Complete Wiring Diagram

```
Arduino Uno/Mega          L298N Motor Driver
================          ==================
Pin 10 (PWM)     -------> IN1 (Left Forward)
Pin 6  (PWM)     -------> IN2 (Left Backward)
Pin 9  (PWM)     -------> IN3 (Right Forward)
Pin 5  (PWM)     -------> IN4 (Right Backward)
Pin 13           -------> ENA (Left Enable)
Pin 12           -------> ENB (Right Enable)
5V               -------> 5V (if not using external power)
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

## Motor Direction Testing

After uploading the firmware, you can test motor directions:

1. **Test Left Motor Forward**: Send command `m 100 0`
2. **Test Left Motor Backward**: Send command `m -100 0`
3. **Test Right Motor Forward**: Send command `m 0 100`
4. **Test Right Motor Backward**: Send command `m 0 -100`
5. **Test Both Forward**: Send command `m 100 100`
6. **Stop All Motors**: Send command `m 0 0`

## Encoder Testing

1. **Read Encoders**: Send command `e`
2. **Reset Encoders**: Send command `r`
3. **Get Status**: Send command `i`

## Troubleshooting

### Motors Not Moving
1. Check 12V power supply connection
2. Verify L298N jumpers are in place for ENA/ENB
3. Check motor connections to OUT1/OUT2 and OUT3/OUT4
4. Test with higher PWM values (try 150-200 instead of 100)

### Wrong Motor Direction
1. Swap the motor wires (OUT1 with OUT2, or OUT3 with OUT4)
2. Or modify the firmware to invert motor directions

### Encoders Not Counting
1. Check encoder power (5V and GND)
2. Verify encoder signal connections
3. Ensure encoders are properly mounted to motors
4. Test by manually rotating motors and checking encoder counts

### Serial Communication Issues
1. Check baud rate is set to 57600
2. Ensure proper line endings (use '\r' or '\r\n')
3. Check Arduino is connected to correct COM port

## Firmware Features

### Available Serial Commands
- `e` - Read encoder values
- `r` - Reset encoders to zero
- `m left_speed right_speed` - Set motor speeds (-255 to 255)
- `o left_pwm right_pwm` - Set raw PWM values
- `s` - Stop all motors
- `i` - Get system info (motor speeds and encoder counts)

### Safety Features
- Motor timeout: Motors automatically stop if no command received for 2 seconds
- Speed limiting: Motor speeds constrained to -255 to +255 range
- Interrupt-safe encoder reading

## ROS Integration

This firmware is compatible with the ROS2 `differential_drive_robot` package. The Arduino interface node will:

1. Send `e` commands to read encoders
2. Send `m` commands to control motors
3. Send `r` command to reset encoders on startup

Make sure to set the correct serial port in your ROS launch files:
```bash
ros2 launch differential_drive_robot complete_robot.launch.py port:=/dev/ttyUSB0
```

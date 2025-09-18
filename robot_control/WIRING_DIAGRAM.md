# 🤖 DriveSync Robot Control System - Complete Wiring Diagram

## System Overview
This is a comprehensive autonomous robot navigation system with:
- **Arduino Uno/Nano** - Motor control and encoder reading
- **YDLidar X2** - 360° LIDAR scanner for obstacle detection
- **Differential drive robot** - Two motors with encoders
- **USB Camera** (optional) - YOLO-based collision avoidance
- **Raspberry Pi** - Main control computer

## 🔌 Complete Wiring Diagram

```
                    RASPBERRY PI
                    ┌─────────────┐
                    │             │
                    │  USB Ports  │
                    │             │
                    └─────────────┘
                           │
                    ┌──────┼──────┐
                    │      │      │
                   USB0   USB1   USB2
                    │      │      │
                    │      │      │
              ┌─────▼─┐  ┌─▼─┐  ┌─▼─┐
              │Arduino│  │LIDAR│  │Camera│
              │Uno/Nano│ │ X2  │  │(opt)│
              └─────┬─┘  └─┬─┘  └─┬─┘
                    │      │      │
                    │      │      │
              ┌─────▼─┐  ┌─▼─┐  ┌─▼─┐
              │L298N  │  │5V  │  │USB │
              │Motor  │  │PSU  │  │Cable│
              │Driver │  │     │  │     │
              └─────┬─┘  └─┬─┘  └─┬─┘
                    │      │      │
                    │      │      │
              ┌─────▼─┐  ┌─▼─┐  ┌─▼─┐
              │Motors │  │LIDAR│  │Pi  │
              │+Encoders│ │Motor│  │    │
              └─────┬─┘  └─┬─┘  └─┬─┘
                    │      │      │
                    │      │      │
              ┌─────▼─┐  ┌─▼─┐  ┌─▼─┐
              │Robot  │  │360°│  │GUI │
              │Chassis│  │Scan│  │    │
              └───────┘  └───┘  └────┘
```

## 📋 Hardware Components

### 1. Arduino Uno/Nano
- **Purpose**: Motor control and encoder reading
- **Firmware**: OptimizedArduinoFirmware.ino
- **Communication**: Serial over USB (/dev/ttyUSB0)

### 2. YDLidar X2
- **Purpose**: 360° obstacle detection and mapping
- **Port**: /dev/ttyUSB1
- **Baudrate**: 115200
- **Power**: 5V (external supply recommended)
- **Range**: 0.1m - 8m
- **Frequency**: 6Hz

### 3. L298N Motor Driver
- **Purpose**: Control differential drive motors
- **Input**: Arduino PWM signals
- **Output**: Motor power control

### 4. Differential Drive Motors
- **Left Motor**: 4993 ticks/revolution
- **Right Motor**: 4966 ticks/revolution
- **Wheel Diameter**: 3.5cm
- **Wheelbase**: 10cm

### 5. Encoders
- **Left Encoder**: D2, D3 (interrupt pins)
- **Right Encoder**: A4, A5 (analog pins with PCINT)
- **Type**: Quadrature encoders

## 🔗 Detailed Pin Connections

### Arduino to L298N Motor Driver
```
Arduino Pin    L298N Pin    Function
─────────────  ───────────  ──────────────
D10           IN1          Left Motor Forward
D9            IN2          Left Motor Backward
D6            IN3          Right Motor Forward
D11           IN4          Right Motor Backward
D13           ENA          Left Motor Enable
D12           ENB          Right Motor Enable
```

### Arduino to Encoders
```
Arduino Pin    Encoder Pin  Function
─────────────  ───────────  ──────────────
D2             A            Left Encoder A (PCINT18)
D3             B            Left Encoder B (PCINT19)
A4             A            Right Encoder A (PCINT12)
A5             B            Right Encoder B (PCINT13)
```

### Power Connections
```
Component     Voltage    Current    Connection
────────────  ────────   ────────   ──────────────
Arduino       5V         500mA     USB from Pi
L298N         12V        2A        External PSU
LIDAR         5V         1A        External PSU
Motors        12V        1A each   L298N output
```

## 🚀 System Architecture

### Software Components
1. **pathfinding_robot_controller.py** - Main GUI controller
2. **smooth_robot_controller.py** - Basic robot control
3. **collision_avoidance_system.py** - YOLO safety system
4. **modules/** - Navigation, pathfinding, mapping
5. **lidar_mapping/** - SLAM and LIDAR processing
6. **firmware/** - Arduino motor control code

### Communication Flow
```
Raspberry Pi ←→ Arduino (Serial USB)
     ↓
Motor Commands → L298N → Motors
     ↓
Encoder Data ← Arduino ← Encoders
     ↓
Odometry ← Python ← Serial Data

Raspberry Pi ←→ LIDAR (Serial USB)
     ↓
360° Scan Data → Python → Obstacle Detection
     ↓
Safety Zones → Navigation → Motor Commands

Raspberry Pi ←→ Camera (USB)
     ↓
Video Stream → YOLO → Object Detection
     ↓
Safety Override → Motor Stop
```

## ⚙️ Configuration Settings

### Robot Specifications
- **Wheel Diameter**: 3.5cm
- **Wheelbase**: 10cm
- **Left Motor**: 4993 ticks/rev
- **Right Motor**: 4966 ticks/rev
- **Resolution**: ~0.022mm/tick

### LIDAR Safety Zones
- **Front**: 30cm
- **Sides**: 25cm
- **Back**: 20cm
- **Scan Frequency**: 6Hz
- **Range**: 0.1m - 8m

### Navigation Parameters
- **Waypoint Tolerance**: 5cm
- **Goal Tolerance**: 8cm
- **Max Speed**: 120 PWM
- **Turn Speed**: 80 PWM
- **Approach Speed**: 60 PWM

## 🔧 Setup Instructions

### 1. Hardware Assembly
1. Mount Arduino on robot chassis
2. Connect L298N motor driver to Arduino
3. Wire motors to L298N outputs
4. Connect encoders to Arduino pins
5. Mount LIDAR on robot (5V power required)
6. Connect USB camera (optional)

### 2. Software Installation
```bash
# Install Python dependencies
pip install -r requirements.txt

# Upload Arduino firmware
# Use Arduino IDE to upload OptimizedArduinoFirmware.ino

# Test connections
python modules/arduino_port_detector.py
python lidar_mapping/ydlidar_x2_optimized.py
```

### 3. System Testing
```bash
# Test basic robot control
python smooth_robot_controller.py

# Test pathfinding system
python pathfinding_robot_controller.py

# Test LIDAR mapping
python lidar_mapping/slam_navigator_gui.py
```

## 🛡️ Safety Features

### Multi-Layer Safety System
1. **LIDAR Obstacle Detection** - Real-time 360° scanning
2. **Camera Object Detection** - YOLO-based safety
3. **Emergency Stop** - Immediate motor halt
4. **Path Verification** - 3-check confirmation before resuming
5. **Manual Override** - Keyboard control with safety limits

### Safety Zones
- **Front Zone**: 30cm × 25cm (rectangular)
- **Side Zones**: 25cm × 55cm (left/right)
- **Back Zone**: 20cm × 25cm (rectangular)
- **Emergency Stop**: Any obstacle in safety zones

## 📊 Performance Specifications

### Communication
- **Arduino Serial**: 115200 baud, 50Hz encoder updates
- **LIDAR Serial**: 115200 baud, 6Hz scan rate
- **Camera**: USB 2.0, 60 FPS for safety

### Navigation
- **Pathfinding**: A* and RRT algorithms
- **Update Rate**: 20Hz navigation, 60Hz GUI
- **Accuracy**: ±5cm waypoint tolerance
- **Speed**: 0-120 PWM range (adjustable)

### Mapping
- **Resolution**: 2-3cm per pixel
- **Workspace**: 3m × 3m
- **Real-time**: SLAM with obstacle avoidance
- **Persistence**: Save/load maps

## 🔍 Troubleshooting

### Common Issues
1. **Arduino not detected**: Check USB cable, try different port
2. **LIDAR not scanning**: Verify 5V power supply, check /dev/ttyUSB1
3. **Motors not responding**: Check L298N connections, verify PWM signals
4. **Encoders not counting**: Check wiring, verify interrupt pins
5. **Navigation issues**: Ensure robot starts on road network

### Debug Commands
```bash
# Check serial ports
ls /dev/ttyUSB* /dev/ttyACM*

# Test Arduino communication
python -c "import serial; s=serial.Serial('/dev/ttyUSB0', 115200); print(s.readline())"

# Test LIDAR
python lidar_mapping/ydlidar_x2_optimized.py

# Debug robot controller
python pathfinding_robot_controller.py --debug
```

## 📁 File Structure
```
robot_control/
├── pathfinding_robot_controller.py    # Main GUI controller
├── smooth_robot_controller.py         # Basic robot control
├── collision_avoidance_system.py      # YOLO safety system
├── modules/
│   ├── arduino_port_detector.py       # Auto-detect Arduino
│   ├── map_environment.py             # Map rendering
│   ├── navigation.py                  # Path following
│   └── pathfinding.py                 # A* and RRT algorithms
├── lidar_mapping/
│   ├── ydlidar_x2_optimized.py        # LIDAR interface
│   ├── lidar_cartesian_plot.py        # LIDAR visualization
│   └── slam_navigator_gui.py          # SLAM navigation
├── firmware/
│   └── OptimizedArduinoFirmware/
│       └── OptimizedArduinoFirmware.ino
└── requirements.txt                    # Python dependencies
```

This wiring diagram provides a complete overview of the DriveSync robot control system, including all hardware connections, software components, and configuration settings needed for autonomous navigation with obstacle avoidance.

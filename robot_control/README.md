# Robot Control - Standalone Pathfinding System

A complete autonomous navigation system for differential drive robots with human-like control logic.

## 🚀 Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Upload firmware to Arduino
# Open firmware/OptimizedArduinoFirmware/OptimizedArduinoFirmware.ino in Arduino IDE
# Upload to your Arduino board

# Run pathfinding system
python3 pathfinding_robot_controller.py

# Or run direct control
python3 smooth_robot_controller.py
```

## 🎮 Controls

### Pathfinding Mode
- **Left-click**: Set navigation target on roads
- **W/A/S/D**: Manual control override
- **ESC**: Exit application
- **I**: Toggle debug information

### Manual Control
- **W**: Forward
- **S**: Backward  
- **A**: Turn right (swapped)
- **D**: Turn left (swapped)
- **ESC**: Exit

## 🧠 How It Works

### Human-like Navigation
The robot uses simple, intuitive logic:

1. **Look at target**: Calculate direction to waypoint
2. **Turn if needed**: Only if >11° off target
3. **Drive straight**: Equal motor speeds for straight movement
4. **Repeat**: Continue until target reached

### Key Features
- **Angular pathfinding**: Only 90°, 45°, 180°, 360° turns
- **Junction-free boundaries**: Smooth intersection navigation
- **Real-time visualization**: See robot's decision process
- **Robust communication**: Error-tolerant Arduino interface

## 📁 File Structure

```
robot_control/
├── pathfinding_robot_controller.py  # Main application
├── smooth_robot_controller.py       # Direct control interface
├── modules/                         # Core navigation system
├── firmware/                        # Arduino code
├── debug_scripts/                   # Hardware testing
├── tests/                          # Test files
├── docs/                           # Documentation
└── requirements.txt                # Python dependencies
```

## 🔧 Hardware Setup

### Required Components
- Arduino Uno/Nano
- L298N Motor Driver
- 2x DC Motors with Quadrature Encoders
- Differential drive robot chassis

### Wiring
See main project README for detailed wiring diagram.

### Firmware
Upload `firmware/OptimizedArduinoFirmware/OptimizedArduinoFirmware.ino` to your Arduino.

## 🛠️ Troubleshooting

### Common Issues
1. **Robot doesn't move**: Check motor wiring and power supply
2. **Encoders read 0**: Verify encoder connections and pull-up resistors
3. **Write timeout errors**: Check USB cable and reduce command frequency
4. **Robot turns wrong way**: Swap A/D key mappings or motor wiring

### Debug Tools
```bash
# Test motor directions
python3 debug_scripts/simple_motor_test.py

# Test encoder readings
python3 debug_scripts/test_corrected_encoders.py

# Monitor serial communication
python3 debug_scripts/serial_monitor.py
```

## 📊 Performance

- **Navigation Update**: 20Hz
- **Encoder Resolution**: 0.03mm/tick
- **Turn Accuracy**: ±5°
- **Pathfinding**: Real-time A* with smoothing

---

For more details, see the main project documentation.

# 🚀 DriveSync Robot Control System - Installation Guide

## Quick Start Installation

### 1. Install Python Dependencies
```bash
# Run the Python dependencies installer
python3 install_python_dependencies.py

# Or install manually
pip3 install -r requirements.txt
```

### 2. Install YDLIDAR SDK
```bash
# Run the YDLIDAR SDK installer (requires sudo)
./install_ydlidar_sdk.sh

# This will:
# - Install system dependencies
# - Clone and build YDLIDAR SDK
# - Install Python wrapper
# - Configure udev rules
# - Create test scripts
```

### 3. Test Installation
```bash
# Test Python packages
python3 test_installation.py

# Test YDLIDAR (requires hardware)
python3 ~/ydlidar_installation/test_ydlidar.py

# Test robot system
python3 pathfinding_robot_controller.py
```

## Detailed Installation Steps

### Prerequisites
- **Operating System**: Linux (Ubuntu/Debian/Raspberry Pi OS)
- **Python**: 3.7 or higher
- **Hardware**: YDLidar X2, Arduino Uno/Nano, L298N Motor Driver

### System Dependencies
The installation script will automatically install:
- `build-essential` - Compilation tools
- `cmake` - Build system
- `pkg-config` - Package configuration
- `git` - Version control
- `python3-dev` - Python development headers
- `libudev-dev` - USB device support
- `libusb-1.0-0-dev` - USB library
- `libserial-dev` - Serial communication
- `libboost-all-dev` - Boost libraries

### YDLIDAR SDK Installation
The `install_ydlidar_sdk.sh` script performs these steps:

1. **System Setup**
   - Updates package list
   - Installs build dependencies
   - Detects system architecture

2. **SDK Compilation**
   - Clones YDLIDAR SDK from GitHub
   - Configures with CMake
   - Builds with optimized settings
   - Installs system-wide

3. **Python Integration**
   - Installs Python wrapper
   - Configures import paths
   - Tests installation

4. **Device Configuration**
   - Creates udev rules for USB devices
   - Adds user to dialout group
   - Sets up permissions

### Hardware Setup
1. **Connect YDLidar X2**
   - USB to `/dev/ttyUSB1`
   - 5V power supply (1A minimum)
   - Check with: `ls -la /dev/ttyUSB*`

2. **Connect Arduino**
   - USB to `/dev/ttyUSB0`
   - Upload firmware: `OptimizedArduinoFirmware.ino`
   - Test with: `python3 modules/arduino_port_detector.py`

3. **Motor Driver**
   - L298N connected to Arduino
   - 12V power supply for motors
   - Encoders connected to Arduino pins

## Troubleshooting

### Common Issues

#### YDLIDAR Not Detected
```bash
# Check USB devices
lsusb | grep -i ydlidar

# Check serial ports
ls -la /dev/ttyUSB*

# Check permissions
groups $USER

# Test connection
python3 ~/ydlidar_installation/test_ydlidar.py
```

#### Arduino Not Responding
```bash
# Check port
ls -la /dev/ttyUSB0

# Test communication
python3 modules/arduino_port_detector.py

# Check firmware
# Upload OptimizedArduinoFirmware.ino via Arduino IDE
```

#### Python Import Errors
```bash
# Reinstall packages
pip3 install --user --upgrade -r requirements.txt

# Check Python path
python3 -c "import sys; print(sys.path)"

# Use virtual environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

#### Permission Errors
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in, or run:
newgrp dialout

# Check groups
groups $USER
```

### Debug Commands

#### System Information
```bash
# Check architecture
uname -m

# Check Python version
python3 --version

# Check installed packages
pip3 list | grep -E "(ydlidar|opencv|pygame)"
```

#### Hardware Testing
```bash
# Test YDLIDAR
python3 -c "import ydlidar; print('YDLIDAR OK')"

# Test Arduino
python3 -c "import serial; s=serial.Serial('/dev/ttyUSB0', 115200); print(s.readline())"

# Test camera
python3 -c "import cv2; cap=cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera FAIL')"
```

## Configuration Files

### YDLIDAR Configuration
```python
# Default settings in ydlidar_x2_optimized.py
port = "/dev/ttyUSB1"
baudrate = 115200
scan_frequency = 6.0
max_range = 8.0
min_range = 0.10
```

### Arduino Configuration
```cpp
// Default settings in OptimizedArduinoFirmware.ino
#define LEFT_MOTOR_FORWARD   10
#define LEFT_MOTOR_BACKWARD   9
#define RIGHT_MOTOR_FORWARD   6
#define RIGHT_MOTOR_BACKWARD  11
#define LEFT_ENC_PIN_A    2
#define LEFT_ENC_PIN_B    3
#define RIGHT_ENC_PIN_A   A4
#define RIGHT_ENC_PIN_B   A5
```

## Performance Optimization

### For Raspberry Pi
```bash
# Increase GPU memory split
sudo raspi-config
# Advanced Options > Memory Split > 128

# Enable hardware acceleration
echo "gpu_mem=128" | sudo tee -a /boot/config.txt

# Optimize for real-time performance
echo "dtparam=audio=off" | sudo tee -a /boot/config.txt
```

### For Development
```bash
# Use virtual environment
python3 -m venv robot_env
source robot_env/bin/activate
pip install -r requirements.txt

# Install development tools
pip install jupyter ipython black flake8
```

## Uninstallation

### Remove YDLIDAR SDK
```bash
# Remove system installation
sudo rm -rf /usr/local/lib/libydlidar*
sudo rm -rf /usr/local/include/ydlidar
sudo rm -rf /etc/udev/rules.d/99-ydlidar.rules

# Remove user installation
rm -rf ~/ydlidar_installation
pip3 uninstall ydlidar
```

### Remove Python Packages
```bash
# Remove all packages
pip3 uninstall -r requirements.txt

# Or remove individually
pip3 uninstall pyserial pygame opencv-python ultralytics numpy pillow ydlidar matplotlib
```

## Support and Documentation

### File Structure
```
robot_control/
├── install_ydlidar_sdk.sh          # YDLIDAR SDK installer
├── install_python_dependencies.py  # Python dependencies installer
├── test_installation.py            # Installation test script
├── requirements.txt                # Python package requirements
├── WIRING_DIAGRAM.md              # Hardware wiring guide
├── INSTALLATION_GUIDE.md          # This file
└── README.md                      # System overview
```

### Getting Help
1. Check the troubleshooting section above
2. Review the WIRING_DIAGRAM.md for hardware issues
3. Run the test scripts to identify problems
4. Check system logs: `journalctl -u ydlidar.service`

### Useful Commands
```bash
# Quick system check
python3 test_installation.py

# Test robot system
python3 pathfinding_robot_controller.py --help

# Check all serial devices
ls -la /dev/tty*

# Monitor system resources
htop
```

This installation guide provides everything needed to set up the DriveSync robot control system with YDLIDAR support.

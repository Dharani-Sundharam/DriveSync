# LIDAR Mapping System for YDLIDAR X2

This module provides LIDAR-based mapping functionality using the YDLIDAR X2 sensor, integrated with the pathfinding robot controller.

## Features

- **Real-time LIDAR data acquisition** from YDLIDAR X2
- **Occupancy grid mapping** with configurable resolution
- **Cartesian plot visualization** optimized for 480x320 displays
- **Integration with existing pathfinding controller**
- **Real-time obstacle detection and mapping**
- **Map persistence** (save/load functionality)

## Hardware Requirements

- YDLIDAR X2 360° Laser Scanner
- Raspberry Pi or compatible system
- USB-to-Serial adapter (if needed)
- Display capable of 480x320 resolution (or higher)

## Software Requirements

- Python 3.7+
- pygame
- numpy
- pyserial
- YDLIDAR SDK (see installation instructions below)

## Installation

### 1. Install System Dependencies

```bash
sudo apt update
sudo apt install -y python3-dev python3-pip cmake build-essential libusb-1.0-0-dev
```

### 2. Install Python Dependencies

```bash
cd /path/to/robot_control/lidar_mapping
pip install -r requirements.txt
```

### 3. Install YDLIDAR SDK

```bash
# Clone the YDLIDAR SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK

# Build and install
mkdir build
cd build
cmake ..
make
sudo make install

# Update library cache
sudo ldconfig
```

### 4. Set up Device Permissions

```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Create udev rule for YDLIDAR X2 (optional)
sudo nano /etc/udev/rules.d/99-ydlidar.rules
```

Add the following content:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="ydlidar"
```

Then reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Usage

### Standalone LIDAR GUI

Run the standalone LIDAR mapping GUI:

```bash
cd /path/to/robot_control
python -m lidar_mapping.lidar_gui
```

**Note:** The system is configured to use `/dev/ttyUSB1` for the YDLIDAR X2 and `/dev/ttyUSB0` for the main robot controller.

### Integration with Pathfinding Controller

To integrate LIDAR mapping with the existing pathfinding controller, modify your main script:

```python
from pathfinding_robot_controller import PathfindingRobotController
from lidar_mapping.pathfinding_integration import integrate_lidar_with_pathfinding

# Create base controller
controller = PathfindingRobotController(port='/dev/ttyUSB0')

# Enhance with LIDAR mapping
enhanced_controller = integrate_lidar_with_pathfinding(
    pathfinding_controller=controller,
    lidar_port='/dev/ttyUSB1',  # LIDAR on different port
    enable_mapping=True
)

# Start LIDAR mapping
enhanced_controller.start_lidar_mapping()

# Run the enhanced controller
controller.run()
```

### Testing LIDAR Interface

Test the LIDAR interface directly:

```bash
cd /path/to/robot_control
python -m lidar_mapping.ydlidar_interface
```

## Configuration

### Display Configuration

The GUI is optimized for 480x320 displays but can be configured for other resolutions:

```python
from lidar_mapping.lidar_gui import LidarMappingGUI

# Custom resolution
gui = LidarMappingGUI(
    lidar_port='/dev/ttyUSB1',  # YDLIDAR X2 port
    display_width=800,
    display_height=600
)
```

### Mapping Parameters

Adjust mapping parameters in the `LidarMapper` initialization:

```python
from lidar_mapping.lidar_mapper import LidarMapper

mapper = LidarMapper(
    map_width=10.0,      # Map width in meters
    map_height=10.0,     # Map height in meters
    resolution=0.05,     # Resolution in meters/cell
    max_range=5.0        # Maximum LIDAR range to consider
)
```

## Controls

### GUI Controls

- **G**: Toggle coordinate grid display
- **L**: Toggle LIDAR points display
- **O**: Toggle occupancy map display
- **R**: Toggle robot display
- **I**: Toggle information panel
- **C**: Clear map
- **S**: Save map to file
- **SPACE**: Pause/resume LIDAR scanning
- **ESC/Q**: Exit application

### Integration Controls

When integrated with the pathfinding controller:

- **L**: Toggle LIDAR overlay
- **M**: Clear LIDAR map
- **N**: Save LIDAR map
- All existing pathfinding controller controls remain available

## File Structure

```
lidar_mapping/
├── __init__.py                 # Module initialization
├── ydlidar_interface.py        # YDLIDAR X2 interface
├── lidar_mapper.py            # Mapping and occupancy grid
├── lidar_gui.py               # GUI and visualization
├── pathfinding_integration.py  # Integration with pathfinding
├── requirements.txt           # Python dependencies
└── README.md                  # This file
```

## Troubleshooting

### LIDAR Connection Issues

1. **Check device permissions**:
   ```bash
   ls -l /dev/ttyUSB*
   # Should show your user in the group or mode 666
   ```

2. **Verify LIDAR is detected**:
   ```bash
   dmesg | grep tty
   # Look for USB device connections
   ```

3. **Test serial communication**:
   ```bash
   # Test LIDAR connection
   python -c "import serial; print(serial.Serial('/dev/ttyUSB1', 115200))"
   # Test robot controller connection  
   python -c "import serial; print(serial.Serial('/dev/ttyUSB0', 115200))"
   ```

### Performance Issues

1. **Reduce map resolution** for better performance on slower systems
2. **Adjust scan frequency** in the LIDAR interface
3. **Limit LIDAR range** to reduce processing load
4. **Use smaller display resolution** if needed

### Integration Issues

1. **Check import paths** - ensure all modules are in Python path
2. **Verify port assignments** - LIDAR and main controller need different ports
3. **Check threading** - ensure no conflicts between LIDAR and main threads

## Development

### Adding New Features

1. **Extend LidarMapper** for new mapping algorithms
2. **Modify LidarMappingGUI** for additional visualization
3. **Update pathfinding_integration.py** for enhanced navigation features

### Testing

Run tests with:
```bash
python -m pytest lidar_mapping/tests/
```

### Contributing

1. Follow existing code style and documentation
2. Add tests for new functionality
3. Update README.md for new features
4. Test integration with existing pathfinding controller

## License

This module is part of the robot control system and follows the same license terms.

## References

- [YDLIDAR X2 Documentation](https://www.ydlidar.com/products/view/5.html)
- [YDLidar-SDK GitHub](https://github.com/YDLIDAR/YDLidar-SDK)
- [Occupancy Grid Mapping](https://en.wikipedia.org/wiki/Occupancy_grid_mapping)
- [Pygame Documentation](https://www.pygame.org/docs/)

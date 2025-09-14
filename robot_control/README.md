# DriveSync Robot Control System

Advanced autonomous robot navigation system with LIDAR obstacle avoidance, pathfinding, and real-time safety features.

## 🚀 Quick Start

### Prerequisites
```bash
pip install pygame numpy opencv-python ultralytics dataclasses threading queue
pip install ydlidar  # For LIDAR support
```

### Hardware Requirements
- **Arduino Uno/Nano** - Motor control and encoders
- **YDLidar X2** - 360° LIDAR scanner (`/dev/ttyUSB1`)
- **Differential drive robot** - Two motors with encoders
- **USB Camera** (optional) - Collision avoidance

### Run the System
```bash
# Main pathfinding controller with GUI
python pathfinding_robot_controller.py

# SLAM-based navigation
python lidar_mapping/slam_navigator_gui.py

# Basic robot control
python smooth_robot_controller.py
```

## 🎮 Controls

### Keyboard Controls
- **W/A/S/D** - Manual robot control
- **Mouse Click** - Set navigation target
- **L** - Toggle LIDAR overlay
- **SPACE** - Stop navigation
- **1/2** - Switch pathfinding algorithms (A*/RRT)
- **R** - Reset robot position
- **P** - Toggle robot placement mode
- **X** - Debug information
- **H** - Help screen

## 🗺️ Features

### Navigation & Pathfinding
- **A* Algorithm** - Optimal pathfinding on road networks
- **RRT Algorithm** - Rapid exploration for complex paths
- **Road-based navigation** - Stays on defined road network
- **Waypoint following** - Multi-point navigation support

### LIDAR Integration
- **Real-time obstacle detection** - 30cm front, 25cm sides, 20cm back safety zones
- **Emergency stop** - Immediate halt when obstacles detected
- **Path continuation** - Automatically resumes navigation when clear
- **Visual overlay** - Color-coded LIDAR points and safety zones

### Safety Systems
- **Multi-layer safety** - LIDAR + Camera-based collision avoidance
- **Emergency stop** - Multiple safety triggers
- **Path verification** - 3-check confirmation before resuming
- **Visual warnings** - Red overlay during obstacle detection

### SLAM & Mapping
- **Real-time mapping** - Build maps while navigating
- **Occupancy grid** - 2cm resolution mapping
- **Click-to-navigate** - GUI-based target selection
- **Map persistence** - Save and load generated maps

## 📁 File Structure

### Core Controllers
- `pathfinding_robot_controller.py` - **Main GUI controller** with pathfinding
- `smooth_robot_controller.py` - **Basic robot control** with visualization
- `collision_avoidance_system.py` - **YOLO-based safety system**

### LIDAR & Navigation
- `lidar_mapping/slam_navigator_gui.py` - **SLAM-based navigation GUI**
- `lidar_mapping/autonomous_navigator.py` - **Autonomous navigation core**
- `lidar_mapping/ydlidar_x2_optimized.py` - **LIDAR interface**
- `lidar_mapping/simple_slam.py` - **SLAM implementation**

### Modules
- `modules/pathfinding.py` - **A* and RRT algorithms**
- `modules/navigation.py` - **Navigation controller**
- `modules/map_environment.py` - **Map rendering and road networks**
- `modules/arduino_port_detector.py` - **Auto Arduino detection**

### Firmware
- `firmware/OptimizedArduinoFirmware/` - **Arduino motor control code**

## 🔧 Configuration

### Robot Dimensions
- **Wheel diameter**: 7cm
- **Wheelbase**: 20cm  
- **Safety zones**: Front 30cm, Sides 25cm, Back 20cm

### Map Settings
- **Workspace**: 3m x 3m
- **Resolution**: 2-3cm per pixel
- **Road width**: 40-50cm main roads, 30-40cm secondary

### LIDAR Settings
- **Port**: `/dev/ttyUSB1` (auto-detected)
- **Baudrate**: 115200
- **Scan frequency**: 6Hz
- **Range**: 6m maximum

## 🚨 Safety Features

### Obstacle Detection
- **LIDAR safety zones** - Immediate stop when obstacles enter zones
- **Camera detection** - YOLO-based object recognition
- **Multi-check verification** - Confirms path is clear before resuming

### Emergency Systems
- **Immediate motor stop** - Hardware-level safety
- **Navigation pause** - Preserves target for resumption
- **Visual warnings** - Red overlay and status indicators

## 🎯 Usage Examples

### Basic Navigation
1. Run `python pathfinding_robot_controller.py`
2. Press **L** to enable LIDAR overlay
3. Click on map to set target
4. Robot navigates automatically with obstacle avoidance

### SLAM Mapping
1. Run `python lidar_mapping/slam_navigator_gui.py`
2. Robot scans environment to build map
3. Click on generated map to navigate
4. Real-time obstacle detection and avoidance

### Manual Control
1. Use **W/A/S/D** keys for direct control
2. LIDAR safety zones prevent collisions
3. Visual feedback shows robot state

## 🔍 Troubleshooting

### LIDAR Issues
- Check `/dev/ttyUSB1` connection
- Verify 5V power supply to LIDAR
- Press **X** for debug info

### Arduino Connection
- Auto-detects Arduino port
- Check serial permissions: `sudo usermod -a -G dialout $USER`
- Verify firmware upload

### Navigation Problems
- Ensure robot starts on road network
- Check wheel encoder connections
- Verify motor directions in firmware

## 📊 Status Indicators

### LIDAR Status
- **◆ Green** - Scanning, path clear
- **◆ Red** - Obstacle detected
- **◆ Yellow** - Connected, no data
- **◇ Gray** - LIDAR disabled

### Robot Status
- **Blue robot** - Normal operation
- **Green ring** - Navigating
- **Cyan ring** - Target reached
- **Red overlay** - Obstacle detected

## 🛠️ Development

### Adding New Features
- Extend `modules/` for new algorithms
- Add GUI elements to controllers
- Integrate with existing safety systems

### Testing
- Use `debug_scripts/` for hardware testing
- `test_*.py` files for system validation
- Check `logs/` for debugging information

---

**Built for autonomous navigation with safety-first design**

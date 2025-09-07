# 🤖 Pathfinding Robot Controller

## 🚀 Quick Start Guide

### 1️⃣ Upload Arduino Firmware
First, upload the corrected firmware to your Arduino:
```
OptimizedArduinoFirmware/OptimizedArduinoFirmware.ino
```

### 2️⃣ Run the Pathfinding System
```bash
cd /home/dharani/Desktop/DriveSync/robot_control
python3 pathfinding_robot_controller.py
```

## 🎮 How to Use

### 🖱️ Mouse Controls
- **Left Click**: Set navigation target (robot will automatically plan path and go there!)
- Click anywhere on the map - system will snap to nearest road if needed

### ⌨️ Keyboard Controls
- **W/A/S/D**: Manual control (when not navigating automatically)
- **SPACE**: Stop current navigation
- **1**: Switch to A* pathfinding algorithm (default)
- **2**: Switch to RRT pathfinding algorithm
- **R**: Reset robot position to center
- **H**: Toggle help screen
- **ESC**: Exit

### 🗺️ Map Elements
- **Dark Gray Roads**: Where robot can drive
- **Brown Lines**: Road boundaries 
- **Red Circles**: Obstacles to avoid
- **Green Areas**: Grass (robot can't drive here)
- **Yellow Line**: Planned path to target
- **Green Line**: Robot's actual traveled path
- **Cyan Circles**: Current waypoint robot is heading to

## 🎯 How It Works

1. **Click anywhere** on the screen to set a target
2. **System automatically**:
   - Finds nearest road point
   - Plans optimal path using A* or RRT algorithm
   - Robot follows path with differential drive control
3. **Robot navigates** autonomously to your target!

## 📁 File Structure
```
robot_control/
├── pathfinding_robot_controller.py  # Main application
├── smooth_robot_controller.py       # Robot hardware interface
├── OptimizedArduinoFirmware/        # Arduino code
└── modules/                         # Modular components
    ├── map_environment.py          # Map and roads
    ├── pathfinding.py              # A* and RRT algorithms  
    └── navigation.py               # Path following control
```

## 🔧 System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   User Clicks   │───▶│  Pathfinding     │───▶│   Navigation    │
│   Target Point  │    │  Algorithm       │    │   Controller    │
└─────────────────┘    │  (A* or RRT)     │    └─────────────────┘
                       └──────────────────┘             │
                                                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│     Arduino     │◀───│    Serial        │◀───│  Motor Commands │
│   (Motors +     │    │  Communication   │    │  (Left/Right    │
│   Encoders)     │    │                  │    │   Speeds)       │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 🎨 Visual Indicators

### Robot Status
- 🔵 **Blue Rectangle**: Robot body
- 🟡 **Yellow Arrow**: Robot heading direction  
- 🟢 **Green Ring**: Currently navigating
- 🔵 **Cyan Ring**: Reached target destination

### Path Planning
- 🟡 **Yellow Line**: Planned path from robot to target
- 🔵 **Cyan Circles**: Current waypoint robot is moving toward
- 🟢 **Green Line**: Robot's actual traveled path (odometry)

## 🛠️ Troubleshooting

### Robot not moving?
1. Check Arduino is connected (`/dev/ttyUSB1`)
2. Verify firmware uploaded correctly
3. Check encoder connections: D2,D3 (left), A4,A5 (right)

### Path not found?
1. Make sure target is on or near a road (dark gray areas)
2. Try clicking closer to existing roads
3. Switch pathfinding algorithm (press '1' or '2')

### Controls reversed?
1. In `smooth_robot_controller.py`, uncomment line:
   ```python
   # left_speed, right_speed = right_speed, left_speed
   ```

## 🎯 Advanced Features

- **Real-time Odometry**: Robot tracks its actual position using wheel encoders
- **Obstacle Avoidance**: Pathfinding algorithms avoid red obstacle circles
- **Road Constraints**: Robot only navigates on valid road areas
- **Smooth Path Following**: Advanced differential drive control for smooth curves
- **Multiple Algorithms**: Switch between A* (optimal) and RRT (exploratory)

## 🚀 Next Steps

Want to customize? Edit these modules:
- `modules/map_environment.py`: Add more roads, obstacles
- `modules/pathfinding.py`: Tune algorithm parameters
- `modules/navigation.py`: Adjust robot speeds, turning behavior

Have fun exploring autonomous robot navigation! 🎉

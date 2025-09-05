# 🤖 ROS2 Differential Drive Robot

A complete ROS2-based differential drive robot system with Arduino integration, designed for distributed deployment across PC and Raspberry Pi.

![Robot Demo](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Platform](https://img.shields.io/badge/Platform-PC%20%2B%20Pi-green) ![Arduino](https://img.shields.io/badge/Arduino-Compatible-orange)

## ✨ **Features**

- 🎮 **Distributed Control**: PC handles visualization, Pi handles real-time control
- 🔄 **Real-time Odometry**: Accurate pose estimation from wheel encoders  
- 🎯 **3D Visualization**: Complete robot model in RViz
- ⌨️ **Keyboard Teleop**: Intuitive manual control
- 🔧 **Modular Design**: Independent, reusable ROS2 nodes
- 🧪 **Testing Support**: Simulation mode without hardware
- 📚 **Complete Documentation**: Step-by-step setup guides

## 🚀 **Quick Start**

### **1. Hardware Setup**
- Arduino Mega/Uno + L298N motor driver
- Two DC motors with encoders  
- Raspberry Pi 4
- PC with ROS2

### **2. Upload Arduino Firmware**
```bash
# Upload ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino to Arduino
```

### **3. Setup Raspberry Pi**
```bash
curl -sSL https://raw.githubusercontent.com/yourusername/differential-drive-robot/main/scripts/setup_pi.sh | bash
```

### **4. Setup PC**
```bash
curl -sSL https://raw.githubusercontent.com/yourusername/differential-drive-robot/main/scripts/setup_pc.sh | bash
```

### **5. Run the System**

**On Raspberry Pi:**
```bash
ros2 launch differential_drive_robot pi_robot_nodes.launch.py port:=/dev/ttyUSB0
```

**On PC:**
```bash
ros2 launch differential_drive_robot pc_visualization.launch.py
```

## 🎮 **Control the Robot**

Use keyboard in teleop terminal:
```
   u    i    o
   j    k    l  
   m    ,    .

i/,: forward/backward
j/l: turn left/right
k/space: stop
q/z: speed up/down
```

## 🏗️ **System Architecture**

```
┌─────────────────┐    Network    ┌──────────────────┐    USB    ┌─────────────┐
│       PC        │◄──────────────►│  Raspberry Pi    │◄──────────►│   Arduino   │
│                 │                │                  │           │             │
│ • RViz          │                │ • Arduino Bridge│           │ • Motors    │
│ • Teleop        │                │ • Odometry       │           │ • Encoders  │
│ • Monitoring    │                │ • Robot State    │           │ • PID       │
└─────────────────┘                └──────────────────┘           └─────────────┘
```

## 📁 **Package Contents**

- **`src/`**: Python ROS2 nodes
- **`launch/`**: Launch files for different configurations
- **`urdf/`**: Robot description files
- **`rviz/`**: RViz configuration
- **`scripts/`**: Setup and deployment scripts
- **`ros_arduino_bridge/`**: Arduino firmware

## 🧪 **Testing Without Hardware**

```bash
# Test with simulated data
ros2 launch differential_drive_robot test_system.launch.py
```

## 📚 **Documentation**

- **[Complete Deployment Guide](DEPLOYMENT_GUIDE.md)**: Detailed setup instructions
- **[Arduino Guide](ros_arduino_bridge/README.md)**: Hardware setup and firmware
- **[Troubleshooting](DEPLOYMENT_GUIDE.md#troubleshooting)**: Common issues and solutions

## 🔧 **Configuration**

Key parameters (configurable in launch files):
```yaml
wheel_diameter: 0.20      # meters
wheel_track: 0.35         # meters
encoder_resolution: 360   # ticks per revolution
max_speed: 1.0           # m/s
```

## 🛠️ **Requirements**

### **Software:**
- ROS2 Jazzy
- Python 3.8+
- Arduino IDE (for firmware upload)

### **Hardware:**
- Raspberry Pi 4 (recommended) 
- Arduino Mega/Uno
- L298N motor driver
- DC motors with encoders
- 12V power supply

## 🤝 **Contributing**

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 🐛 **Issues & Support**

- **Hardware Issues**: Check [hardware setup guide](DEPLOYMENT_GUIDE.md#hardware-setup)
- **Software Issues**: See [troubleshooting section](DEPLOYMENT_GUIDE.md#troubleshooting)
- **Network Issues**: Verify domain ID and connectivity
- **Bug Reports**: Open an issue with detailed description

## 📜 **License**

MIT License - see [LICENSE](LICENSE) file for details.

## 🙏 **Acknowledgments**

- Built on [ROSArduinoBridge](https://github.com/hbrobotics/ros_arduino_bridge) firmware
- Inspired by differential drive robotics community
- Thanks to ROS2 and Arduino communities

---

**⭐ If this project helped you, please give it a star!**

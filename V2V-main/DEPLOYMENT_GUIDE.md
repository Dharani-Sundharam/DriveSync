# 🤖 Differential Drive Robot - Complete Deployment Guide

A complete ROS2-based differential drive robot system with Arduino integration, designed for distributed deployment (PC + Raspberry Pi).

## 🎯 **System Overview**

- **PC**: Runs RViz visualization and teleop control
- **Raspberry Pi**: Handles real-time robot control and Arduino communication
- **Arduino**: Controls motors and reads encoders using ROSArduinoBridge firmware

## 🏗️ **Architecture**

```
┌─────────────────┐    Network    ┌──────────────────┐    USB    ┌─────────────┐
│       PC        │◄──────────────►│  Raspberry Pi    │◄──────────►│   Arduino   │
│                 │                │                  │           │             │
│ • RViz          │                │ • Arduino Bridge│           │ • Motors    │
│ • Teleop        │                │ • Odometry       │           │ • Encoders  │
│ • Monitoring    │                │ • Robot State    │           │ • PID       │
└─────────────────┘                └──────────────────┘           └─────────────┘
```

## 🚀 **Quick Start**

### **Option 1: Automated Setup**

**On Raspberry Pi:**
```bash
curl -sSL https://raw.githubusercontent.com/yourusername/differential-drive-robot/main/scripts/setup_pi.sh | bash
```

**On PC:**
```bash
curl -sSL https://raw.githubusercontent.com/yourusername/differential-drive-robot/main/scripts/setup_pc.sh | bash
```

### **Option 2: Manual Setup**

Follow the detailed steps below.

---

## 📋 **Detailed Setup Instructions**

### **Step 1: Hardware Setup**

#### **Arduino Connections:**
```
L298N Motor Driver:
├── Left Motor:  Pin 10 (Forward), Pin 6 (Backward), Pin 13 (Enable)
├── Right Motor: Pin 9 (Forward), Pin 5 (Backward), Pin 12 (Enable)

Encoders:
├── Left:  Pin 2 (A), Pin 3 (B)
├── Right: Pin A4 (A), Pin A5 (B)

Power: 12V for motors, 5V for Arduino
USB: Connect to Raspberry Pi
```

#### **Upload Arduino Firmware:**
1. Open `ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino`
2. Ensure these lines are enabled:
   ```cpp
   #define USE_BASE
   #define ARDUINO_ENC_COUNTER  
   #define L298_MOTOR_DRIVER
   ```
3. Upload to Arduino

---

### **Step 2: Raspberry Pi Setup**

#### **2.1 Install Ubuntu 22.04**
- Use Raspberry Pi Imager
- Enable SSH during setup
- Connect to network

#### **2.2 Install ROS2**
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep

# Install dependencies
sudo apt install -y python3-serial ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros

# Setup rosdep
sudo rosdep init
rosdep update

# Add to bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

#### **2.3 Setup Serial Access**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

#### **2.4 Create Workspace**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy the differential_drive_robot package here
# (via scp, git clone, or USB transfer)
```

#### **2.5 Build Package**
```bash
cd ~/ros2_ws
colcon build --packages-select differential_drive_robot
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

### **Step 3: PC Setup**

#### **3.1 Install ROS2 Desktop**
Follow official ROS2 installation guide for your OS.

#### **3.2 Install Dependencies**
```bash
sudo apt install -y python3-serial ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros ros-jazzy-rviz2

# Setup environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

#### **3.3 Setup Workspace**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy the differential_drive_robot package
# Build the package
cd ~/ros2_ws
colcon build --packages-select differential_drive_robot
source install/setup.bash
```

---

## 🎮 **Running the System**

### **Start Robot Control (Raspberry Pi)**
```bash
# SSH into Pi
ssh ubuntu@<pi-ip>

# Launch robot nodes
ros2 launch differential_drive_robot pi_robot_nodes.launch.py port:=/dev/ttyUSB0
```

### **Start Visualization (PC)**
```bash
# On PC
ros2 launch differential_drive_robot pc_visualization.launch.py
```

### **Control the Robot**
Use keyboard in the teleop terminal:
```
   u    i    o
   j    k    l
   m    ,    .

i/,: forward/backward    j/l: turn left/right
k/space: stop           q/z: speed up/down
```

---

## 🔧 **Configuration**

### **Robot Parameters**
Edit in launch files or set as parameters:
```yaml
wheel_diameter: 0.20      # meters
wheel_track: 0.35         # meters  
encoder_resolution: 360   # ticks per revolution
max_speed: 1.0           # m/s
serial_port: '/dev/ttyUSB0'
baud_rate: 57600
```

### **Network Configuration**
Both machines must:
- Be on same network
- Have same `ROS_DOMAIN_ID` (default: 42)
- Be able to ping each other

---

## 🧪 **Testing**

### **Test Without Arduino**
```bash
# On PC - test with simulated data
ros2 launch differential_drive_robot test_system.launch.py
```

### **Test Serial Connection**
```bash
# On Pi - check Arduino connection
ls /dev/ttyUSB* /dev/ttyACM*
ros2 topic echo /encoder_ticks
```

### **Test Network Communication**
```bash
# On PC - check topics from Pi
ros2 topic list
ros2 topic echo /odom
```

---

## 🔍 **Troubleshooting**

### **Serial Issues**
```bash
# Check permissions
ls -l /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0

# Check if Arduino is responding
# Arduino IDE Serial Monitor should show encoder values
```

### **Network Issues**
```bash
# Check connectivity
ping <other-machine-ip>

# Restart ROS2 discovery
ros2 daemon stop && ros2 daemon start

# Check domain ID
echo $ROS_DOMAIN_ID
```

### **No Robot Movement**
1. Check Arduino connections
2. Verify encoder signals: `ros2 topic echo /encoder_ticks`
3. Test motor PWM values
4. Check PID parameters in Arduino code

### **RViz Issues**
```bash
# Check robot description
ros2 topic echo /robot_description

# Check TF tree
ros2 run tf2_tools view_frames
```

---

## 📁 **Package Structure**
```
differential_drive_robot/
├── src/                     # Python nodes
│   ├── arduino_interface.py # Arduino communication
│   ├── odometry_node.py     # Odometry calculation
│   ├── robot_state_publisher.py # Joint states
│   ├── teleop_keyboard.py   # Manual control
│   └── test_system.py       # Testing without Arduino
├── launch/                  # Launch files
│   ├── pi_robot_nodes.launch.py    # Pi: Robot control
│   ├── pc_visualization.launch.py  # PC: RViz + teleop
│   ├── complete_robot.launch.py    # All-in-one
│   └── test_system.launch.py       # Testing
├── urdf/                    # Robot description
├── rviz/                    # RViz configuration
├── scripts/                 # Setup scripts
└── ros_arduino_bridge/      # Arduino firmware
```

---

## 🚦 **System Status Indicators**

### **Healthy System:**
- ✅ Green: All nodes running
- ✅ Encoder data flowing: `/encoder_ticks` updating
- ✅ Odometry publishing: `/odom` updating  
- ✅ Robot visible in RViz
- ✅ Teleop responsive

### **Common Issues:**
- ❌ Red: Serial connection failed
- ⚠️ Yellow: Network communication issues
- 🔄 Blue: Nodes restarting

---

## 🔄 **Maintenance**

### **Regular Tasks:**
- Monitor encoder drift
- Calibrate wheel parameters
- Update PID parameters
- Check motor performance

### **Logs:**
```bash
# View logs
ros2 log list
ros2 log view <node_name>

# Debug mode
ros2 launch differential_drive_robot pi_robot_nodes.launch.py --ros-args --log-level debug
```

---

## 🆘 **Support**

- **Issues**: Check troubleshooting section
- **Hardware**: Verify connections and power
- **Software**: Check ROS2 installation and dependencies
- **Network**: Ensure proper domain ID and connectivity

---

## 📜 **License**

MIT License - See LICENSE file for details.

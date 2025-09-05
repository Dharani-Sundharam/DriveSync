# Differential Drive Robot with Arduino Integration

This ROS2 package provides a complete differential drive robot system that interfaces with Arduino using the ROSArduinoBridge firmware.

## Features

- **Arduino Integration**: Communicates with Arduino via serial using ROSArduinoBridge protocol
- **Odometry Calculation**: Computes robot pose from wheel encoder data
- **Robot Visualization**: URDF model and RViz configuration for robot visualization
- **Teleop Control**: Keyboard-based teleoperation
- **Real-time TF Broadcasting**: Publishes coordinate transformations
- **Modular Design**: Separate nodes for different functionalities

## Hardware Requirements

### Arduino Setup
- Arduino Mega (recommended) or Uno
- L298N motor driver or compatible
- Two DC motors with encoders
- Encoder connections:
  - Left encoder: Pins 2 (A), 3 (B)
  - Right encoder: Pins A4 (A), A5 (B)
- Motor connections (L298 driver):
  - Left motor: Pins 10 (forward), 6 (backward), 13 (enable)
  - Right motor: Pins 9 (forward), 5 (backward), 12 (enable)

### Robot Physical Parameters (configurable)
- Wheel diameter: 20cm
- Wheel track (distance between wheels): 35cm
- Encoder resolution: 360 ticks per revolution

## Installation

1. **Upload Arduino Firmware**:
   ```bash
   # Upload the ROSArduinoBridge.ino to your Arduino
   # Make sure to configure the correct motor driver and encoder settings
   ```

2. **Build the ROS2 Package**:
   ```bash
   cd /path/to/your/ros2_workspace
   colcon build --packages-select differential_drive_robot
   source install/setup.bash
   ```

3. **Install Dependencies**:
   ```bash
   sudo apt install python3-serial
   pip3 install pyserial
   ```

## Usage

### 1. Basic Robot Bringup

Start all essential nodes (without RViz or teleop):
```bash
ros2 launch differential_drive_robot robot_bringup.launch.py port:=/dev/ttyUSB0
```

### 2. Complete System with Visualization and Control

Start everything including RViz and keyboard teleop:
```bash
ros2 launch differential_drive_robot complete_robot.launch.py port:=/dev/ttyUSB0
```

### 3. Individual Components

**Arduino Interface Only**:
```bash
ros2 run differential_drive_robot arduino_interface.py --ros-args -p port:=/dev/ttyUSB0
```

**Odometry Node Only**:
```bash
ros2 run differential_drive_robot odometry_node.py
```

**RViz Visualization**:
```bash
ros2 launch differential_drive_robot rviz_display.launch.py
```

**Keyboard Teleop**:
```bash
ros2 run differential_drive_robot teleop_keyboard.py
```

## Robot Control

### Keyboard Teleop Controls
```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

space key, k : force stop
CTRL-C to quit
```

### Programmatic Control
Send velocity commands to the `/cmd_vel` topic:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.5}}'
```

## Topics and Services

### Published Topics
- `/odom` (nav_msgs/Odometry): Robot odometry
- `/joint_states` (sensor_msgs/JointState): Wheel joint states
- `/encoder_ticks` (std_msgs/Int32MultiArray): Raw encoder values
- `/wheel_speeds` (std_msgs/Float32MultiArray): Wheel speeds in m/s

### Subscribed Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

### TF Frames
- `odom` → `base_footprint`: Robot pose in odometry frame
- `base_footprint` → `base_link`: Robot base
- `base_link` → `drivewhl_l_link`, `drivewhl_r_link`: Wheel positions
- `base_link` → `laser`: Sensor mounting position

## Configuration

### Arduino Interface Parameters
```yaml
port: '/dev/ttyUSB0'          # Serial port
baud: 57600                   # Baud rate
wheel_diameter: 0.20          # Wheel diameter in meters
wheel_track: 0.35             # Distance between wheels in meters
encoder_resolution: 360       # Encoder ticks per revolution
max_speed: 1.0               # Maximum linear speed in m/s
```

### Odometry Parameters
```yaml
publish_tf: true             # Publish TF transforms
odom_frame: 'odom'          # Odometry frame name
base_frame: 'base_footprint' # Base frame name
```

## Troubleshooting

### Arduino Connection Issues
1. Check serial port permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```

2. Verify Arduino is connected:
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```

3. Test serial communication:
   ```bash
   ros2 topic echo /encoder_ticks
   ```

### Robot Not Moving
1. Check motor connections and power supply
2. Verify encoder signals:
   ```bash
   ros2 topic echo /encoder_ticks
   ```
3. Check Arduino serial monitor for error messages
4. Verify PID parameters in Arduino code

### Odometry Issues
1. Check encoder tick counts are changing
2. Verify wheel parameters match physical robot
3. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```

## Arduino Firmware Commands

The ROSArduinoBridge responds to these serial commands:
- `e` - Read encoders
- `r` - Reset encoders
- `m <left> <right>` - Set motor speeds (ticks per frame)
- `o <left> <right>` - Set raw PWM values
- `u <Kp>:<Kd>:<Ki>:<Ko>` - Update PID parameters

## Customization

### Modifying Robot Parameters
Edit the launch files to change robot dimensions, encoder resolution, or other parameters.

### Adding Sensors
1. Update the URDF file to add sensor links
2. Create new nodes for sensor data processing
3. Update launch files to include new nodes

### Different Motor Controllers
Modify the Arduino firmware motor driver section to support your specific motor controller.

## License

MIT License - see LICENSE file for details.

# Distributed Setup: PC + Raspberry Pi

## Quick Setup Guide

### On Raspberry Pi (Robot Control)

```bash
# 1. Set ROS domain
export ROS_DOMAIN_ID=42
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# 2. Navigate to workspace
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 3. Launch robot control nodes
ros2 launch differential_drive_robot pi_robot_nodes.launch.py port:=/dev/ttyUSB0
```

### On PC (Visualization)

```bash
# 1. Set same ROS domain
export ROS_DOMAIN_ID=42
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# 2. Navigate to workspace
cd /home/dharani/Desktop/V2V
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 3. Launch RViz and teleop
ros2 launch differential_drive_robot pc_visualization.launch.py
```

## Testing Connection

### On PC - Check topics from Pi:
```bash
ros2 topic list
ros2 topic echo /odom
ros2 node list
```

### Send commands from PC to Pi:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

## Troubleshooting

### Can't see topics from other machine:
```bash
# Check network connectivity
ping <other-machine-ip>

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Check domain ID matches
echo $ROS_DOMAIN_ID
```

### RViz shows no data:
```bash
# Check if robot_description topic exists
ros2 topic list | grep robot_description

# Check TF data
ros2 run tf2_tools view_frames
```

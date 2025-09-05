#!/bin/bash

# Raspberry Pi Setup Script for Differential Drive Robot
# Run this script on your Raspberry Pi

set -e

echo "🤖 Setting up Differential Drive Robot on Raspberry Pi..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Pi
if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo -e "${YELLOW}Warning: This doesn't appear to be a Raspberry Pi${NC}"
fi

# Update system
echo -e "${YELLOW}Updating system packages...${NC}"
sudo apt update && sudo apt upgrade -y

# Install ROS2 if not already installed
if ! command -v ros2 &> /dev/null; then
    echo -e "${YELLOW}Installing ROS2 Jazzy...${NC}"
    
    # Add ROS2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install -y ros-jazzy-ros-base
    sudo apt install -y python3-colcon-common-extensions python3-rosdep
    
    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
else
    echo -e "${GREEN}ROS2 already installed${NC}"
fi

# Install additional dependencies
echo -e "${YELLOW}Installing additional dependencies...${NC}"
sudo apt install -y python3-serial ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Setup ROS environment
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo -e "${GREEN}Added ROS2 to bashrc${NC}"
fi

# Set ROS domain ID
if ! grep -q "export ROS_DOMAIN_ID=42" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo -e "${GREEN}Set ROS_DOMAIN_ID=42${NC}"
fi

# Create workspace
mkdir -p ~/ros2_ws/src

echo -e "${GREEN}✅ Raspberry Pi setup complete!${NC}"
echo -e "${YELLOW}Next steps:${NC}"
echo "1. Copy the differential_drive_robot package to ~/ros2_ws/src/"
echo "2. Run: cd ~/ros2_ws && colcon build --packages-select differential_drive_robot"
echo "3. Connect Arduino via USB"
echo "4. Upload ROSArduinoBridge.ino to Arduino"
echo "5. Run: ros2 launch differential_drive_robot pi_robot_nodes.launch.py"
echo ""
echo -e "${RED}⚠️  Please log out and log back in for group changes to take effect${NC}"

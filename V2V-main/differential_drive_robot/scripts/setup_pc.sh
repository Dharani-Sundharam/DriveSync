#!/bin/bash

# PC Setup Script for Differential Drive Robot
# Run this script on your PC for visualization

set -e

echo "🖥️  Setting up Differential Drive Robot on PC..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}❌ ROS2 not found. Please install ROS2 Desktop first.${NC}"
    echo "Visit: https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

echo -e "${GREEN}✅ ROS2 found${NC}"

# Install additional dependencies
echo -e "${YELLOW}Installing additional dependencies...${NC}"
sudo apt update
sudo apt install -y python3-serial ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros ros-jazzy-rviz2

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

echo -e "${GREEN}✅ PC setup complete!${NC}"
echo -e "${YELLOW}Next steps:${NC}"
echo "1. Build the differential_drive_robot package"
echo "2. Make sure your Pi and PC are on the same network"
echo "3. Run: ros2 launch differential_drive_robot pc_visualization.launch.py"
echo ""
echo -e "${YELLOW}💡 Tip: Source your workspace with: source install/setup.bash${NC}"

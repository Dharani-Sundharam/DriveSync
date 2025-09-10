#!/bin/bash
# LIDAR Mapping System Installation Script
# ========================================

set -e  # Exit on any error

echo "🚀 LIDAR Mapping System Installation"
echo "===================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
    print_error "Please do not run this script as root (except for system dependencies)"
    exit 1
fi

# Step 1: Update system packages
print_step "1. Updating system packages..."
sudo apt update
sudo apt install -y python3-dev python3-pip cmake build-essential libusb-1.0-0-dev

# Step 2: Install Python dependencies
print_step "2. Installing Python dependencies..."
pip3 install --user -r requirements.txt

# Step 3: Set up device permissions
print_step "3. Setting up device permissions..."
sudo usermod -a -G dialout $USER
print_status "Added user $USER to dialout group"

# Step 4: Create udev rule for YDLIDAR (optional)
print_step "4. Creating udev rule for YDLIDAR..."
sudo tee /etc/udev/rules.d/99-ydlidar.rules > /dev/null << EOF
# YDLIDAR X2 device rule
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="ydlidar"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
print_status "YDLIDAR udev rule created"

# Step 5: Create logs directory
print_step "5. Creating logs directory..."
mkdir -p ../logs
print_status "Logs directory created"

# Step 6: YDLIDAR SDK installation (optional)
print_step "6. YDLIDAR SDK installation..."
read -p "Do you want to install the YDLIDAR SDK? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    print_status "Installing YDLIDAR SDK..."
    
    # Check if SDK already exists
    if [ ! -d "YDLidar-SDK" ]; then
        git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    else
        print_warning "YDLIDAR SDK directory already exists, skipping clone"
    fi
    
    cd YDLidar-SDK
    
    # Build SDK
    if [ ! -d "build" ]; then
        mkdir build
    fi
    
    cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig
    
    cd ../..
    print_status "YDLIDAR SDK installed successfully"
else
    print_warning "Skipping YDLIDAR SDK installation"
    print_warning "Note: The system will run in simulation mode without the SDK"
fi

# Step 7: Test installation
print_step "7. Testing installation..."
python3 test_lidar_system.py

# Step 8: Final instructions
print_step "8. Installation complete!"
echo
print_status "LIDAR Mapping System is now installed!"
echo
echo "Next steps:"
echo "1. Reboot or log out/in for group permissions to take effect"
echo "2. Connect your YDLIDAR X2 to a USB port"
echo "3. Run the test script: python3 test_lidar_system.py"
echo "4. Run the GUI: python3 -m lidar_mapping.lidar_gui"
echo
echo "Integration with pathfinding controller:"
echo "- Modify your main script to include LIDAR mapping"
echo "- See README.md for integration examples"
echo
print_status "Installation completed successfully!"

# Optional: Display device information
if [ -e /dev/ttyUSB0 ]; then
    print_status "USB serial devices found:"
    ls -l /dev/ttyUSB*
else
    print_warning "No USB serial devices found. Connect your YDLIDAR X2 and check connections."
fi

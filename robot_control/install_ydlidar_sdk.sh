#!/bin/bash

# =============================================================================
# YDLIDAR SDK Installation Script for DriveSync Robot Control System
# =============================================================================
# This script installs the YDLIDAR SDK and Python wrapper for YDLidar X2
# Compatible with Raspberry Pi and Linux systems
# =============================================================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if running on Raspberry Pi
is_raspberry_pi() {
    if [ -f /proc/device-tree/model ]; then
        grep -q "Raspberry Pi" /proc/device-tree/model
    else
        return 1
    fi
}

# Function to detect architecture
detect_architecture() {
    case $(uname -m) in
        x86_64) echo "x86_64" ;;
        armv7l) echo "armv7l" ;;
        aarch64) echo "aarch64" ;;
        armv6l) echo "armv6l" ;;
        *) echo "unknown" ;;
    esac
}

# Print header
echo "=============================================================================="
echo "🤖 YDLIDAR SDK Installation Script for DriveSync Robot Control System"
echo "=============================================================================="
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    print_warning "Running as root. This is not recommended for development."
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Detect system information
ARCH=$(detect_architecture)
IS_PI=$(is_raspberry_pi && echo "true" || echo "false")

print_status "System Information:"
echo "  Architecture: $ARCH"
echo "  Raspberry Pi: $IS_PI"
echo "  OS: $(lsb_release -d 2>/dev/null | cut -f2 || uname -s)"
echo ""

# Update package list
print_status "Updating package list..."
sudo apt update

# Install system dependencies
print_status "Installing system dependencies..."
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    libudev-dev \
    libusb-1.0-0-dev \
    libserial-dev \
    libboost-all-dev

# Install additional dependencies for Raspberry Pi
if [ "$IS_PI" = "true" ]; then
    print_status "Installing Raspberry Pi specific dependencies..."
    sudo apt install -y \
        i2c-tools \
        libi2c-dev \
        wiringpi \
        libwiringpi-dev
fi

# Create installation directory
INSTALL_DIR="$HOME/ydlidar_installation"
print_status "Creating installation directory: $INSTALL_DIR"
mkdir -p "$INSTALL_DIR"
cd "$INSTALL_DIR"

# Clone YDLIDAR SDK
print_status "Cloning YDLIDAR SDK repository..."
if [ -d "YDLidar-SDK" ]; then
    print_warning "YDLidar-SDK directory already exists. Removing..."
    rm -rf YDLidar-SDK
fi

git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK

# Checkout stable version (adjust as needed)
print_status "Checking out stable version..."
git checkout master

# Create build directory
print_status "Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
print_status "Configuring with CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_SHARED_LIBS=ON

# Build the project
print_status "Building YDLIDAR SDK (this may take several minutes)..."
make -j$(nproc)

# Install the SDK
print_status "Installing YDLIDAR SDK system-wide..."
sudo make install

# Update library cache
print_status "Updating library cache..."
sudo ldconfig

# Install Python wrapper
print_status "Installing Python YDLIDAR wrapper..."

# Try to install from PyPI first
if pip3 install ydlidar --user; then
    print_success "YDLIDAR Python package installed from PyPI"
else
    print_warning "PyPI installation failed, trying alternative methods..."
    
    # Try installing from source
    cd "$INSTALL_DIR"
    if [ ! -d "ydlidar-python" ]; then
        git clone https://github.com/YDLIDAR/ydlidar-python.git
    fi
    cd ydlidar-python
    
    # Install Python package
    pip3 install . --user
    print_success "YDLIDAR Python package installed from source"
fi

# Install additional Python dependencies
print_status "Installing additional Python dependencies..."
pip3 install --user \
    pyserial>=3.5 \
    pygame>=2.0.0 \
    opencv-python>=4.5.0 \
    ultralytics>=8.0.0 \
    numpy>=1.21.0 \
    pillow>=8.0.0 \
    matplotlib \
    scipy

# Create udev rules for YDLIDAR
print_status "Creating udev rules for YDLIDAR devices..."
sudo tee /etc/udev/rules.d/99-ydlidar.rules > /dev/null <<EOF
# YDLIDAR USB devices
SUBSYSTEM=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"
EOF

# Add user to dialout group
print_status "Adding user to dialout group..."
sudo usermod -a -G dialout "$USER"

# Reload udev rules
print_status "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# Create test script
print_status "Creating YDLIDAR test script..."
cat > "$INSTALL_DIR/test_ydlidar.py" <<'EOF'
#!/usr/bin/env python3
"""
YDLIDAR Test Script
Tests the YDLIDAR installation and basic functionality
"""

import sys
import time

def test_ydlidar_import():
    """Test if YDLIDAR can be imported"""
    try:
        import ydlidar
        print("✅ YDLIDAR Python package imported successfully")
        return True
    except ImportError as e:
        print(f"❌ Failed to import YDLIDAR: {e}")
        return False

def test_ydlidar_connection():
    """Test YDLIDAR connection"""
    try:
        import ydlidar
        
        # Create LIDAR instance
        laser = ydlidar.CYdLidar()
        
        # Configure for YDLidar X2
        laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB1")
        laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
        laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        laser.setlidaropt(ydlidar.LidarPropScanFrequency, 6.0)
        laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
        laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        laser.setlidaropt(ydlidar.LidarPropMaxRange, 8.0)
        laser.setlidaropt(ydlidar.LidarPropMinRange, 0.10)
        
        # Try to initialize
        if laser.initialize():
            print("✅ YDLIDAR initialized successfully")
            if laser.turnOn():
                print("✅ YDLIDAR motor started successfully")
                time.sleep(2)
                laser.turnOff()
                print("✅ YDLIDAR test completed successfully")
                return True
            else:
                print("❌ Failed to start YDLIDAR motor")
                return False
        else:
            print("❌ Failed to initialize YDLIDAR")
            return False
            
    except Exception as e:
        print(f"❌ YDLIDAR connection test failed: {e}")
        return False

def main():
    print("🔍 Testing YDLIDAR Installation")
    print("=" * 40)
    
    # Test import
    if not test_ydlidar_import():
        print("\n❌ Installation test failed at import stage")
        sys.exit(1)
    
    # Test connection (optional - requires hardware)
    print("\n🔌 Testing YDLIDAR connection (requires hardware)...")
    print("   If no LIDAR is connected, this test will fail - that's normal!")
    
    if test_ydlidar_connection():
        print("\n🎉 All tests passed! YDLIDAR is ready to use.")
    else:
        print("\n⚠️  Connection test failed, but import test passed.")
        print("   This is normal if no LIDAR hardware is connected.")
        print("   YDLIDAR SDK is installed and ready for use.")

if __name__ == "__main__":
    main()
EOF

chmod +x "$INSTALL_DIR/test_ydlidar.py"

# Create systemd service for YDLIDAR (optional)
print_status "Creating YDLIDAR systemd service template..."
sudo tee /etc/systemd/system/ydlidar.service > /dev/null <<EOF
[Unit]
Description=YDLIDAR Service
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$HOME
ExecStart=/usr/bin/python3 $INSTALL_DIR/test_ydlidar.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Cleanup
print_status "Cleaning up build files..."
cd "$INSTALL_DIR"
rm -rf YDLidar-SDK/build

# Print completion message
echo ""
echo "=============================================================================="
print_success "YDLIDAR SDK Installation Completed Successfully!"
echo "=============================================================================="
echo ""
print_status "Installation Summary:"
echo "  ✅ YDLIDAR SDK installed to: /usr/local"
echo "  ✅ Python wrapper installed"
echo "  ✅ System dependencies installed"
echo "  ✅ Udev rules configured"
echo "  ✅ User added to dialout group"
echo "  ✅ Test script created: $INSTALL_DIR/test_ydlidar.py"
echo ""
print_status "Next Steps:"
echo "  1. Reboot your system or log out/in to apply group changes"
echo "  2. Connect your YDLidar X2 to /dev/ttyUSB1"
echo "  3. Run the test script: python3 $INSTALL_DIR/test_ydlidar.py"
echo "  4. Test with your robot: python3 pathfinding_robot_controller.py"
echo ""
print_warning "Important Notes:"
echo "  • Make sure YDLidar X2 is powered with 5V (1A minimum)"
echo "  • Check /dev/ttyUSB1 permissions after reboot"
echo "  • If using different port, update configuration in your scripts"
echo ""
print_status "Troubleshooting:"
echo "  • Check device: ls -la /dev/ttyUSB*"
echo "  • Check permissions: groups $USER"
echo "  • Test connection: python3 $INSTALL_DIR/test_ydlidar.py"
echo "  • View logs: journalctl -u ydlidar.service"
echo ""
echo "🎉 YDLIDAR SDK is ready for your DriveSync robot control system!"
echo "=============================================================================="

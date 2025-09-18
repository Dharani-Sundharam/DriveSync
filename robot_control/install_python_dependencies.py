#!/usr/bin/env python3
"""
Python Dependencies Installation Script for DriveSync Robot Control System
=======================================================================
This script installs all Python dependencies needed for the robot control system.
It handles both pip installation and system-specific requirements.
"""

import subprocess
import sys
import os
import platform
import importlib.util

def run_command(command, description=""):
    """Run a command and return success status"""
    print(f"🔧 {description}")
    try:
        result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
        print(f"✅ {description} - Success")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ {description} - Failed")
        print(f"   Error: {e.stderr}")
        return False

def check_package_installed(package_name):
    """Check if a package is already installed"""
    try:
        spec = importlib.util.find_spec(package_name)
        return spec is not None
    except ImportError:
        return False

def install_python_dependencies():
    """Install all Python dependencies"""
    print("🐍 Installing Python Dependencies for DriveSync Robot Control System")
    print("=" * 70)
    
    # Check Python version
    python_version = sys.version_info
    print(f"Python version: {python_version.major}.{python_version.minor}.{python_version.micro}")
    
    if python_version < (3, 7):
        print("❌ Python 3.7 or higher is required")
        return False
    
    # Upgrade pip first
    if not run_command(f"{sys.executable} -m pip install --upgrade pip", "Upgrading pip"):
        print("⚠️  Failed to upgrade pip, continuing anyway...")
    
    # Install requirements from file
    requirements_file = os.path.join(os.path.dirname(__file__), "requirements.txt")
    if os.path.exists(requirements_file):
        print(f"📦 Installing packages from {requirements_file}")
        if not run_command(f"{sys.executable} -m pip install -r {requirements_file}", "Installing requirements"):
            print("❌ Failed to install requirements from file")
            return False
    else:
        print("⚠️  requirements.txt not found, installing packages individually...")
        
        # Install packages individually
        packages = [
            "pyserial>=3.5",
            "pygame>=2.0.0", 
            "opencv-python>=4.5.0",
            "ultralytics>=8.0.0",
            "numpy>=1.21.0",
            "pillow>=8.0.0",
            "ydlidar>=1.0.0",
            "matplotlib>=3.5.0"
        ]
        
        for package in packages:
            if not run_command(f"{sys.executable} -m pip install {package}", f"Installing {package}"):
                print(f"⚠️  Failed to install {package}, continuing...")
    
    # Install additional packages that might be needed
    additional_packages = [
        "scipy",
        "scikit-learn", 
        "pandas",
        "tqdm"
    ]
    
    print("\n📦 Installing additional useful packages...")
    for package in additional_packages:
        if not check_package_installed(package.split('>=')[0]):
            run_command(f"{sys.executable} -m pip install {package}", f"Installing {package}")
        else:
            print(f"✅ {package} already installed")
    
    return True

def test_imports():
    """Test if all required packages can be imported"""
    print("\n🧪 Testing package imports...")
    
    required_packages = [
        ("serial", "pyserial"),
        ("pygame", "pygame"),
        ("cv2", "opencv-python"),
        ("ultralytics", "ultralytics"),
        ("numpy", "numpy"),
        ("PIL", "pillow"),
        ("matplotlib", "matplotlib")
    ]
    
    all_imports_successful = True
    
    for import_name, package_name in required_packages:
        try:
            __import__(import_name)
            print(f"✅ {package_name} imported successfully")
        except ImportError as e:
            print(f"❌ {package_name} import failed: {e}")
            all_imports_successful = False
    
    # Test YDLIDAR separately (might not be available)
    try:
        import ydlidar
        print("✅ ydlidar imported successfully")
    except ImportError:
        print("⚠️  ydlidar not available - run install_ydlidar_sdk.sh first")
    
    return all_imports_successful

def create_test_script():
    """Create a test script to verify the installation"""
    test_script = """#!/usr/bin/env python3
'''
DriveSync Robot Control System - Installation Test
================================================
This script tests if all components are properly installed.
'''

import sys
import importlib

def test_import(module_name, package_name=None):
    try:
        importlib.import_module(module_name)
        print(f"✅ {package_name or module_name} - OK")
        return True
    except ImportError as e:
        print(f"❌ {package_name or module_name} - FAILED: {e}")
        return False

def main():
    print("🧪 Testing DriveSync Robot Control System Installation")
    print("=" * 55)
    
    tests = [
        ("serial", "PySerial"),
        ("pygame", "Pygame"),
        ("cv2", "OpenCV"),
        ("ultralytics", "Ultralytics YOLO"),
        ("numpy", "NumPy"),
        ("PIL", "Pillow"),
        ("matplotlib", "Matplotlib"),
        ("scipy", "SciPy"),
    ]
    
    passed = 0
    total = len(tests)
    
    for module, name in tests:
        if test_import(module, name):
            passed += 1
    
    # Test YDLIDAR separately
    print("\\n🔍 Testing YDLIDAR (optional):")
    if test_import("ydlidar", "YDLIDAR SDK"):
        passed += 1
        total += 1
    else:
        print("   Run ./install_ydlidar_sdk.sh to install YDLIDAR SDK")
    
    print(f"\\n📊 Test Results: {passed}/{total} packages working")
    
    if passed == total:
        print("🎉 All tests passed! System is ready.")
        return 0
    else:
        print("⚠️  Some tests failed. Check the errors above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
"""
    
    script_path = os.path.join(os.path.dirname(__file__), "test_installation.py")
    with open(script_path, 'w') as f:
        f.write(test_script)
    
    # Make it executable
    os.chmod(script_path, 0o755)
    print(f"✅ Test script created: {script_path}")

def main():
    """Main installation function"""
    print("🤖 DriveSync Robot Control System - Python Dependencies Installer")
    print("=" * 70)
    
    # Check if running on supported platform
    system = platform.system()
    print(f"Operating System: {system}")
    
    if system not in ["Linux", "Darwin"]:  # Linux or macOS
        print("⚠️  This script is designed for Linux/macOS. Windows users should use pip directly.")
    
    # Install dependencies
    if not install_python_dependencies():
        print("❌ Installation failed!")
        return 1
    
    # Test imports
    if not test_imports():
        print("⚠️  Some packages failed to import. Check the errors above.")
    
    # Create test script
    create_test_script()
    
    print("\n" + "=" * 70)
    print("🎉 Python Dependencies Installation Complete!")
    print("=" * 70)
    print("\n📋 Next Steps:")
    print("1. Run the test script: python3 test_installation.py")
    print("2. Install YDLIDAR SDK: ./install_ydlidar_sdk.sh")
    print("3. Test your robot: python3 pathfinding_robot_controller.py")
    print("\n💡 Tips:")
    print("• If you get permission errors, use: pip install --user <package>")
    print("• For virtual environments: python3 -m venv venv && source venv/bin/activate")
    print("• Check the WIRING_DIAGRAM.md for hardware setup")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

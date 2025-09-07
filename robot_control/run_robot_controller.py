#!/usr/bin/env python3
"""
Run Robot Controller
Simple launcher for the smooth robot controller
"""

import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import and run the smooth robot controller
from smooth_robot_controller import main

if __name__ == "__main__":
    print("🤖 Launching Smooth Robot Controller...")
    main()

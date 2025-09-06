#!/bin/bash

echo "🤖 Starting Pathfinding Robot Controller..."
echo "================================================"
echo ""
echo "✅ FULLSCREEN MODE - Large visible roads!"
echo "✅ Robot starts at center (0,0) on main road intersection"
echo "✅ Roads properly centered around (0,0)"
echo "✅ Robot placement mode - Press P to place robot anywhere!"
echo ""
echo "Make sure:"
echo "1. Arduino is connected and firmware uploaded"
echo "2. Robot is on a flat surface"
echo "3. Motors and encoders are connected properly"
echo ""
echo "Controls:"
echo "- Left Click: Set navigation target OR place robot (in P mode)"
echo "- P: Toggle robot placement mode"
echo "- W/A/S/D: Manual control"
echo "- SPACE: Stop navigation"
echo "- R: Reset robot to center (0,0)"
echo "- F: Toggle fullscreen"
echo "- X: Debug info (if issues occur)"
echo "- H: Help screen"
echo "- ESC: Exit"
echo ""
echo "Starting in 3 seconds..."
sleep 3

cd /home/dharani/Desktop/DriveSync/robot_control
python3 pathfinding_robot_controller.py

#!/usr/bin/env python3
"""
Test script for YDLIDAR X2 using official SDK
=============================================

This script tests the YDLIDAR X2 using the official YDLidar-SDK.
"""

import sys
import time
import logging
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from lidar_mapping.ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
    print("✅ Using optimized YDLidar X2 interface")
    USE_OPTIMIZED = True
except ImportError as e:
    print(f"❌ Failed to import optimized interface: {e}")
    print("Falling back to SDK interface...")
    from lidar_mapping.ydlidar_interface_sdk import YDLidarX2Interface
    USE_OPTIMIZED = False


def test_ydlidar_x2():
    """Test YDLIDAR X2 with official SDK"""
    print("🚀 Testing YDLIDAR X2 with Official SDK")
    print("=" * 50)
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    # Create LIDAR interface with optimized configuration
    if USE_OPTIMIZED:
        config = LidarConfig(port='/dev/ttyUSB1', baudrate=115200)
        lidar = YDLidarX2Optimized(config)
    else:
        lidar = YDLidarX2Interface('/dev/ttyUSB1', 115200)
    
    try:
        print("🔌 Connecting to YDLIDAR X2...")
        if lidar.connect():
            print("✅ Connection successful!")
            
            print("🚀 Starting scan...")
            if lidar.start_scanning():
                print("✅ Scanning started!")
                
                # Monitor for 15 seconds
                print("📡 Monitoring scans for 15 seconds...")
                start_time = time.time()
                last_report = start_time
                
                while time.time() - start_time < 15.0:
                    scan = lidar.get_latest_scan()
                    stats = lidar.get_statistics()
                    
                    # Report every 2 seconds
                    if time.time() - last_report >= 2.0:
                        if scan:
                            print(f"📊 Scan #{stats['total_scans']}: "
                                  f"{len(scan.points)} points, "
                                  f"{stats['scan_frequency']:.1f} Hz, "
                                  f"Port: {stats['port']}")
                            
                            # Show some sample points
                            if len(scan.points) >= 5:
                                print("   Sample points:")
                                for i in range(0, min(5, len(scan.points))):
                                    point = scan.points[i]
                                    print(f"     {i+1}: {point.angle:.1f}° -> {point.distance:.2f}m "
                                          f"(quality: {point.quality})")
                        else:
                            print("⏳ Waiting for scan data...")
                        
                        last_report = time.time()
                    
                    time.sleep(0.1)
                
                print("✅ Test completed successfully!")
            else:
                print("❌ Failed to start scanning")
        else:
            print("❌ Failed to connect to LIDAR")
    
    except KeyboardInterrupt:
        print("\n👋 Test interrupted by user")
    
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("🧹 Cleaning up...")
        lidar.cleanup()
        print("✅ Cleanup completed")


if __name__ == "__main__":
    test_ydlidar_x2()

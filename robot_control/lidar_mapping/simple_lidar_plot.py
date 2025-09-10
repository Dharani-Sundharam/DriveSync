#!/usr/bin/env python3
"""
Simple LIDAR Plotter
===================

Simple script to test LIDAR plotting without GUI dependencies.
Based on the reference code for proper polar plotting.
"""

import time
import math
import numpy as np
import matplotlib.pyplot as plt
import logging

# Handle both relative and absolute imports
try:
    from .ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
except ImportError:
    from ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig


def test_lidar_plotting():
    """Test LIDAR with proper plotting"""
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger('SimpleLidarPlot')
    
    # Create LIDAR interface
    config = LidarConfig(port='/dev/ttyUSB1', baudrate=115200)
    lidar = YDLidarX2Optimized(config)
    
    try:
        logger.info("🔌 Connecting to LIDAR...")
        if not lidar.connect():
            logger.error("❌ Failed to connect to LIDAR")
            return
        
        if not lidar.start_scanning():
            logger.error("❌ Failed to start scanning")
            return
        
        logger.info("✅ LIDAR connected and scanning")
        
        # Setup matplotlib for non-interactive plotting
        plt.ioff()  # Turn off interactive mode
        fig, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(10, 10))
        
        logger.info("📊 Starting data collection and plotting...")
        
        # Collect data for 10 seconds
        start_time = time.time()
        data_points = 0
        
        while time.time() - start_time < 10.0:
            scan = lidar.get_latest_scan()
            if scan and scan.points:
                # Extract angles and ranges
                angles = [point.angle for point in scan.points]
                ranges = [point.distance for point in scan.points]
                
                # Filter valid data
                valid_angles = []
                valid_ranges = []
                for angle, range_val in zip(angles, ranges):
                    if 0.05 <= range_val <= 8.0:  # Valid range for X2
                        valid_angles.append(angle)
                        valid_ranges.append(range_val)
                
                if valid_angles:
                    data_points += len(valid_angles)
                    
                    # Update plot
                    ax.clear()
                    ax.set_title('YDLidar X2 - Real-time Data', fontsize=14, fontweight='bold')
                    ax.set_rmax(8.0)
                    ax.grid(True)
                    ax.set_theta_zero_location('N')  # 0 degrees at top
                    ax.set_theta_direction(-1)  # Clockwise
                    
                    # Plot points
                    ax.scatter(valid_angles, valid_ranges, 
                             c=valid_ranges, cmap='plasma', 
                             alpha=0.8, s=3)
                    
                    # Save plot every 2 seconds
                    if int(time.time() - start_time) % 2 == 0:
                        plt.savefig(f'lidar_plot_{int(time.time() - start_time)}.png', 
                                  dpi=100, bbox_inches='tight')
                        logger.info(f"📸 Plot saved with {len(valid_angles)} points")
            
            time.sleep(0.1)
        
        # Save final plot
        plt.savefig('lidar_final_plot.png', dpi=150, bbox_inches='tight')
        logger.info(f"✅ Test completed! Collected {data_points} total data points")
        logger.info("📁 Check the generated PNG files to see the plots")
        
    except KeyboardInterrupt:
        logger.info("👋 Test interrupted by user")
    except Exception as e:
        logger.error(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        lidar.cleanup()
        plt.close('all')
        logger.info("🧹 Cleanup completed")


if __name__ == "__main__":
    test_lidar_plotting()

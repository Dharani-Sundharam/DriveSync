#!/usr/bin/env python3
"""
LIDAR System Test Script
========================

Simple test script to verify LIDAR mapping system functionality.
Can run in simulation mode if no hardware is connected.
"""

import sys
import time
import logging
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from lidar_mapping.ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig
from lidar_mapping.lidar_mapper import LidarMapper
from lidar_mapping.lidar_gui import LidarMappingGUI


def test_lidar_interface():
    """Test LIDAR interface functionality"""
    print("🧪 Testing LIDAR Interface...")
    
    # Create LIDAR interface with optimized configuration
    config = LidarConfig(port='/dev/ttyUSB1', baudrate=115200)
    lidar = YDLidarX2Optimized(config)
    
    try:
        # Test connection (will run in simulation mode if no hardware)
        print("🔌 Testing connection...")
        connected = lidar.connect()
        print(f"   Connection: {'✅ Success' if connected else '❌ Failed (running in simulation)'}")
        
        if lidar.start_scanning():
            print("🚀 Scanning started")
            
            # Monitor for a few seconds
            for i in range(5):
                scan = lidar.get_latest_scan()
                stats = lidar.get_statistics()
                
                if scan:
                    print(f"   Scan {i+1}: {len(scan.points)} points, "
                          f"Freq: {stats['scan_frequency']:.1f} Hz")
                else:
                    print(f"   Scan {i+1}: No data yet")
                
                time.sleep(1)
        
        print("✅ LIDAR interface test completed")
        
    except Exception as e:
        print(f"❌ LIDAR interface test failed: {e}")
    
    finally:
        lidar.cleanup()


def test_lidar_mapper():
    """Test LIDAR mapper functionality"""
    print("\n🧪 Testing LIDAR Mapper...")
    
    try:
        # Create mapper
        mapper = LidarMapper(
            map_width=8.0,
            map_height=8.0,
            resolution=0.05,
            max_range=4.0
        )
        
        print(f"   Map size: {mapper.map_width}x{mapper.map_height}m")
        print(f"   Grid size: {mapper.grid_width}x{mapper.grid_height} cells")
        print(f"   Resolution: {mapper.resolution}m/cell")
        
        # Test coordinate conversions
        test_points = [(0, 0), (1, 1), (-2, 2), (3, -1)]
        print("   Testing coordinate conversions:")
        
        for wx, wy in test_points:
            gx, gy = mapper.world_to_grid(wx, wy)
            wx2, wy2 = mapper.grid_to_world(gx, gy)
            print(f"     ({wx:4.1f}, {wy:4.1f}) -> ({gx:3d}, {gy:3d}) -> ({wx2:4.1f}, {wy2:4.1f})")
        
        # Test with simulated LIDAR data
        lidar = YDLidarX2Interface()
        if lidar.connect():
            lidar.start_scanning()
            time.sleep(1)  # Let it generate some data
            
            scan = lidar.get_latest_scan()
            if scan:
                print(f"   Processing scan with {len(scan.points)} points...")
                mapper.update_map(scan)
                
                stats = mapper.get_statistics()
                print(f"   Processed {stats['total_scans_processed']} scans")
            
            lidar.cleanup()
        
        print("✅ LIDAR mapper test completed")
        
    except Exception as e:
        print(f"❌ LIDAR mapper test failed: {e}")


def test_gui_components():
    """Test GUI components without opening window"""
    print("\n🧪 Testing GUI Components...")
    
    try:
        from lidar_mapping.lidar_gui import CartesianPlot, DisplayConfig
        
        # Test display config
        config = DisplayConfig(width=480, height=320)
        print(f"   Display config: {config.width}x{config.height}")
        
        # Test Cartesian plot
        plot = CartesianPlot(
            display_width=480,
            display_height=320,
            world_width=8.0,
            world_height=8.0,
            plot_margin=30
        )
        
        print(f"   Plot scale: {plot.scale:.1f} pixels/meter")
        
        # Test coordinate conversions
        test_coords = [(0, 0), (1, 1), (-2, 2), (4, -4)]
        print("   Testing screen/world conversions:")
        
        for wx, wy in test_coords:
            sx, sy = plot.world_to_screen(wx, wy)
            wx2, wy2 = plot.screen_to_world(sx, sy)
            print(f"     World({wx:4.1f}, {wy:4.1f}) -> Screen({sx:3d}, {sy:3d}) -> World({wx2:4.1f}, {wy2:4.1f})")
        
        print("✅ GUI components test completed")
        
    except Exception as e:
        print(f"❌ GUI components test failed: {e}")


def main():
    """Main test function"""
    print("🚀 LIDAR System Test Suite")
    print("=" * 50)
    
    # Setup logging
    logging.basicConfig(level=logging.WARNING)  # Reduce log noise during testing
    
    # Run tests
    test_lidar_interface()
    test_lidar_mapper()
    test_gui_components()
    
    print("\n" + "=" * 50)
    print("🏁 Test suite completed!")
    print("\nTo run the full GUI, use:")
    print("   python -m lidar_mapping.lidar_gui")
    print("\nTo test integration with pathfinding controller:")
    print("   python lidar_mapping/pathfinding_integration.py")


if __name__ == "__main__":
    main()

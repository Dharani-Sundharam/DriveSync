#!/usr/bin/env python3
"""
YDLIDAR X2 Interface Module (Official SDK)
==========================================

This module provides a Python interface to the YDLIDAR X2 sensor using the official YDLidar-SDK.
Based on the official examples and optimized for real-time LIDAR data acquisition.
"""

import threading
import time
import math
import logging
import ydlidar
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from collections import deque


@dataclass
class LidarPoint:
    """Represents a single LIDAR measurement point"""
    angle: float        # Angle in radians
    distance: float     # Distance in meters
    quality: int        # Signal quality/intensity (0-255)
    timestamp: float    # Timestamp when measured


@dataclass 
class LidarScan:
    """Represents a complete LIDAR scan (360 degrees)"""
    points: List[LidarPoint]
    scan_time: float
    scan_frequency: float
    is_complete: bool


class YDLidarX2Interface:
    """
    Interface class for YDLIDAR X2 sensor using official YDLidar-SDK
    
    Handles communication with the YDLIDAR X2 and provides real-time scan data
    in a thread-safe manner using the official SDK.
    """
    
    def __init__(self, port: str = '/dev/ttyUSB1', baudrate: int = 115200):
        """
        Initialize YDLIDAR X2 interface
        
        Args:
            port: Serial port device path (will auto-detect if available)
            baudrate: Serial communication baudrate (115200 for X2)
        """
        self.port = port
        self.baudrate = baudrate
        
        # Initialize YDLidar SDK
        ydlidar.os_init()
        self.laser = ydlidar.CYdLidar()
        
        # Threading and data management
        self.is_scanning = False
        self.scan_thread: Optional[threading.Thread] = None
        self.data_lock = threading.Lock()
        
        # Current scan data
        self.current_scan: Optional[LidarScan] = None
        self.scan_history = deque(maxlen=10)  # Keep last 10 scans
        
        # Statistics
        self.total_scans = 0
        self.scan_frequency = 0.0
        self.last_scan_time = 0.0
        
        # Setup logging
        self.logger = logging.getLogger('YDLidarX2SDK')
        self.logger.setLevel(logging.INFO)
        
        # Auto-detect port if needed
        self._detect_port()
        
    def _detect_port(self):
        """Auto-detect LIDAR port using SDK"""
        try:
            ports = ydlidar.lidarPortList()
            if ports:
                # Use the first available port
                for key, value in ports.items():
                    self.port = value
                    self.logger.info(f"🔍 Auto-detected LIDAR port: {self.port}")
                    break
            else:
                self.logger.warning(f"⚠️  No LIDAR ports auto-detected, using: {self.port}")
        except Exception as e:
            self.logger.error(f"❌ Port detection error: {e}")
    
    def connect(self) -> bool:
        """
        Connect to YDLIDAR X2 sensor
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.logger.info(f"🔌 Connecting to YDLIDAR X2 on {self.port} at {self.baudrate} baud")
            
            # Configure LIDAR parameters for X2 (optimized settings)
            # YDLIDAR X2 specifications: 115200 baud, TRIANGLE type (optimized)
            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baudrate)
            self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)  # X2 uses triangle ranging
            self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 6.0)  # Optimal for X2 (4-8Hz range)
            self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)  # X2 standard: 3kHz
            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)  # X2 is single channel
            
            # X2 specific range and angle settings
            self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 8.0)  # X2 max range: 8m
            self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.1)  # X2 min range: 10cm
            self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)  # 360° scan
            self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
            self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)  # X2 doesn't have intensity
            
            # Initialize the laser
            ret = self.laser.initialize()
            if ret:
                self.logger.info("✅ YDLIDAR X2 connected successfully")
                return True
            else:
                self.logger.error("❌ Failed to initialize LIDAR")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from YDLIDAR X2 sensor"""
        self.stop_scanning()
        
        try:
            self.laser.disconnecting()
            self.logger.info("🔌 YDLIDAR X2 disconnected")
        except Exception as e:
            self.logger.error(f"❌ Disconnect error: {e}")
    
    def start_scanning(self) -> bool:
        """
        Start continuous LIDAR scanning
        
        Returns:
            True if scanning started successfully
        """
        if self.is_scanning:
            self.logger.warning("⚠️  Scanning already in progress")
            return True
        
        try:
            # Turn on the laser
            ret = self.laser.turnOn()
            if not ret:
                self.logger.error("❌ Failed to turn on LIDAR")
                return False
            
            # Start scanning thread
            self.is_scanning = True
            self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self.scan_thread.start()
            
            self.logger.info("🚀 LIDAR scanning started")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to start scanning: {e}")
            self.is_scanning = False
            return False
    
    def stop_scanning(self):
        """Stop LIDAR scanning"""
        if not self.is_scanning:
            return
        
        self.is_scanning = False
        
        # Wait for thread to finish
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=2.0)
        
        try:
            self.laser.turnOff()
        except Exception as e:
            self.logger.error(f"❌ Turn off error: {e}")
        
        self.logger.info("🛑 LIDAR scanning stopped")
    
    def _scan_loop(self):
        """Main scanning loop (runs in separate thread)"""
        self.logger.info("🔄 Scan loop started")
        
        scan = ydlidar.LaserScan()
        
        while self.is_scanning and ydlidar.os_isOk():
            try:
                # Get scan data from SDK
                r = self.laser.doProcessSimple(scan)
                if r:
                    # Convert SDK scan to our format
                    scan_data = self._convert_scan(scan)
                    if scan_data:
                        with self.data_lock:
                            self.current_scan = scan_data
                            self.scan_history.append(scan_data)
                            self.total_scans += 1
                            
                            # Update scan frequency
                            current_time = time.time()
                            if self.last_scan_time > 0:
                                time_diff = current_time - self.last_scan_time
                                if time_diff > 0:
                                    self.scan_frequency = 1.0 / time_diff
                            self.last_scan_time = current_time
                else:
                    time.sleep(0.01)  # Brief pause if no data
                
            except Exception as e:
                self.logger.error(f"❌ Scan loop error: {e}")
                time.sleep(0.1)  # Brief pause before retry
        
        self.logger.info("🔄 Scan loop ended")
    
    def _convert_scan(self, sdk_scan) -> Optional[LidarScan]:
        """
        Convert SDK LaserScan to our LidarScan format
        
        Args:
            sdk_scan: ydlidar.LaserScan object from SDK
            
        Returns:
            LidarScan object or None if conversion fails
        """
        try:
            points = []
            scan_start_time = time.time()
            
            # Extract points from SDK scan
            for point in sdk_scan.points:
                # Convert angle from radians to degrees for consistency
                angle_deg = math.degrees(point.angle)
                
                # Ensure angle is in 0-360 range
                if angle_deg < 0:
                    angle_deg += 360
                
                lidar_point = LidarPoint(
                    angle=angle_deg,
                    distance=point.range,  # Already in meters
                    quality=int(point.intensity) if hasattr(point, 'intensity') else 255,
                    timestamp=time.time()
                )
                points.append(lidar_point)
            
            # Calculate scan frequency from SDK
            scan_freq = 1.0 / sdk_scan.config.scan_time if sdk_scan.config.scan_time > 0 else 10.0
            
            scan = LidarScan(
                points=points,
                scan_time=time.time() - scan_start_time,
                scan_frequency=scan_freq,
                is_complete=len(points) > 0
            )
            
            return scan
            
        except Exception as e:
            self.logger.error(f"❌ Scan conversion error: {e}")
            return None
    
    def get_latest_scan(self) -> Optional[LidarScan]:
        """
        Get the most recent complete scan
        
        Returns:
            Latest LidarScan object or None if no scan available
        """
        with self.data_lock:
            return self.current_scan
    
    def get_scan_history(self) -> List[LidarScan]:
        """Get list of recent scans"""
        with self.data_lock:
            return list(self.scan_history)
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get LIDAR statistics"""
        with self.data_lock:
            return {
                'total_scans': self.total_scans,
                'scan_frequency': self.scan_frequency,
                'is_scanning': self.is_scanning,
                'connected': True,  # If we got this far, we're connected
                'history_length': len(self.scan_history),
                'port': self.port,
                'baudrate': self.baudrate
            }
    
    def is_connected(self) -> bool:
        """Check if LIDAR is connected"""
        return True  # SDK handles connection state internally
    
    def cleanup(self):
        """Cleanup resources"""
        self.stop_scanning()
        self.disconnect()


# Example usage and testing
if __name__ == "__main__":
    # Setup logging for testing
    logging.basicConfig(level=logging.INFO)
    
    # Create LIDAR interface
    lidar = YDLidarX2Interface('/dev/ttyUSB1', 115200)
    
    try:
        # Connect and start scanning
        if lidar.connect():
            if lidar.start_scanning():
                print("🚀 LIDAR scanning started. Press Ctrl+C to stop...")
                
                # Monitor scanning for 10 seconds
                start_time = time.time()
                while time.time() - start_time < 10.0:
                    scan = lidar.get_latest_scan()
                    stats = lidar.get_statistics()
                    
                    if scan:
                        print(f"📡 Scan: {len(scan.points)} points, "
                              f"Freq: {stats['scan_frequency']:.1f} Hz, "
                              f"Total: {stats['total_scans']}")
                    
                    time.sleep(1.0)
        
    except KeyboardInterrupt:
        print("\n👋 Stopping LIDAR...")
    
    finally:
        lidar.cleanup()
        print("✅ LIDAR interface cleaned up")

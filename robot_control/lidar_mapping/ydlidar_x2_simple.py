#!/usr/bin/env python3
"""
Simple YDLIDAR X2 Interface
============================

Simplified interface for YDLIDAR X2 that works with your existing mapping system.
Based on the official SDK examples but adapted for your 480x320 GUI.
"""

import ydlidar
import time
import math
import logging
import threading
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from collections import deque


@dataclass
class LidarPoint:
    """Represents a single LIDAR measurement point"""
    angle: float        # Angle in degrees (0-360)
    distance: float     # Distance in meters
    quality: int        # Signal quality (0-255)
    timestamp: float    # Timestamp when measured


@dataclass 
class LidarScan:
    """Represents a complete LIDAR scan"""
    points: List[LidarPoint]
    scan_time: float
    scan_frequency: float
    is_complete: bool


class YDLidarX2Simple:
    """
    Simplified YDLIDAR X2 interface that works reliably
    """
    
    def __init__(self, port: str = '/dev/ttyUSB1'):
        """Initialize YDLIDAR X2"""
        self.port = port
        self.laser = None
        self.scan = None
        self.is_connected = False
        self.is_scanning = False
        
        # Data management
        self.current_scan: Optional[LidarScan] = None
        self.scan_history = deque(maxlen=5)
        self.total_scans = 0
        self.scan_frequency = 0.0
        self.last_scan_time = time.time()
        
        # Threading
        self.scan_thread: Optional[threading.Thread] = None
        self.data_lock = threading.Lock()
        
        # Setup logging
        self.logger = logging.getLogger('YDLidarX2Simple')
        self.logger.setLevel(logging.INFO)
        
        # Initialize SDK
        ydlidar.os_init()
    
    def connect(self) -> bool:
        """Connect to YDLIDAR X2"""
        try:
            self.logger.info(f"🔌 Connecting to YDLIDAR X2 on {self.port}")
            
            # Auto-detect port if needed
            ports = ydlidar.lidarPortList()
            if ports:
                for key, value in ports.items():
                    if "USB1" in value or "ttyUSB1" in value:
                        self.port = value
                        break
            
            # Create laser instance
            self.laser = ydlidar.CYdLidar()
            
            # Configure for YDLIDAR X2 (optimized settings from GUI analysis)
            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
            self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 6.0)  # Optimal frequency
            self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)  # X2 standard
            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)  # X2 is single channel
            
            # Initialize
            ret = self.laser.initialize()
            if ret:
                self.is_connected = True
                self.scan = ydlidar.LaserScan()
                self.logger.info("✅ YDLIDAR X2 connected successfully")
                return True
            else:
                self.logger.error("❌ Failed to initialize LIDAR")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ Connection failed: {e}")
            return False
    
    def start_scanning(self) -> bool:
        """Start scanning"""
        if not self.is_connected or not self.laser:
            self.logger.error("❌ Not connected")
            return False
        
        try:
            ret = self.laser.turnOn()
            if ret:
                self.is_scanning = True
                self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
                self.scan_thread.start()
                self.logger.info("🚀 Scanning started")
                return True
            else:
                self.logger.error("❌ Failed to start scanning")
                return False
        except Exception as e:
            self.logger.error(f"❌ Start scanning error: {e}")
            return False
    
    def stop_scanning(self):
        """Stop scanning"""
        self.is_scanning = False
        
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=1.0)
        
        if self.laser:
            try:
                self.laser.turnOff()
            except:
                pass
        
        self.logger.info("🛑 Scanning stopped")
    
    def _scan_loop(self):
        """Scanning loop"""
        self.logger.info("🔄 Scan loop started")
        
        while self.is_scanning and self.laser and ydlidar.os_isOk():
            try:
                r = self.laser.doProcessSimple(self.scan)
                if r:
                    # Convert scan
                    points = []
                    for point in self.scan.points:
                        # Convert angle from radians to degrees
                        angle_deg = math.degrees(point.angle)
                        if angle_deg < 0:
                            angle_deg += 360
                        
                        lidar_point = LidarPoint(
                            angle=angle_deg,
                            distance=point.range,
                            quality=255,  # Default quality
                            timestamp=time.time()
                        )
                        points.append(lidar_point)
                    
                    # Create scan object
                    scan_obj = LidarScan(
                        points=points,
                        scan_time=0.1,  # Approximate
                        scan_frequency=10.0,  # Approximate
                        is_complete=len(points) > 0
                    )
                    
                    # Store scan
                    with self.data_lock:
                        self.current_scan = scan_obj
                        self.scan_history.append(scan_obj)
                        self.total_scans += 1
                        
                        # Update frequency
                        current_time = time.time()
                        time_diff = current_time - self.last_scan_time
                        if time_diff > 0:
                            self.scan_frequency = 1.0 / time_diff
                        self.last_scan_time = current_time
                
                time.sleep(0.05)  # 20Hz max
                
            except Exception as e:
                self.logger.error(f"❌ Scan loop error: {e}")
                time.sleep(0.1)
        
        self.logger.info("🔄 Scan loop ended")
    
    def get_latest_scan(self) -> Optional[LidarScan]:
        """Get latest scan"""
        with self.data_lock:
            return self.current_scan
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get statistics"""
        with self.data_lock:
            return {
                'total_scans': self.total_scans,
                'scan_frequency': self.scan_frequency,
                'is_scanning': self.is_scanning,
                'connected': self.is_connected,
                'history_length': len(self.scan_history),
                'port': self.port
            }
    
    def is_connected_status(self) -> bool:
        """Check connection status"""
        return self.is_connected
    
    def cleanup(self):
        """Cleanup"""
        self.stop_scanning()
        
        if self.laser:
            try:
                self.laser.disconnecting()
            except:
                pass
        
        self.is_connected = False
        self.logger.info("🧹 Cleanup completed")


# Test function
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    lidar = YDLidarX2Simple()
    
    try:
        if lidar.connect():
            print("✅ Connected! Attempting to start scanning...")
            
            if lidar.start_scanning():
                print("✅ Scanning started! Monitoring for 10 seconds...")
                
                start_time = time.time()
                while time.time() - start_time < 10:
                    scan = lidar.get_latest_scan()
                    stats = lidar.get_statistics()
                    
                    if scan:
                        print(f"📡 Scan: {len(scan.points)} points, "
                              f"Freq: {stats['scan_frequency']:.1f} Hz")
                    else:
                        print("⏳ Waiting for scan...")
                    
                    time.sleep(1)
            else:
                print("❌ Failed to start scanning")
                print("💡 Check LIDAR power supply (5V) and connections")
        else:
            print("❌ Failed to connect")
    
    except KeyboardInterrupt:
        print("\n👋 Interrupted")
    
    finally:
        lidar.cleanup()

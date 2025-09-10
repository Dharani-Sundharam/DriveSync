#!/usr/bin/env python3
"""
YDLidar X2 Optimized Interface
==============================

Optimized interface for YDLidar X2 based on the comprehensive GUI analysis.
Uses the best configuration settings identified for reliable operation.

Configuration optimized for:
- Port: /dev/ttyUSB1 with fallback to auto-detection
- Baudrate: 115200 (X2 standard)
- Type: TYPE_TRIANGLE (X2 uses triangle ranging)
- Sample Rate: 3 kHz (X2 standard)
- Frequency: 6.0 Hz (optimal for X2)
"""

import sys
import time
import math
import logging
import threading
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from collections import deque

try:
    import ydlidar
    YDLIDAR_AVAILABLE = True
    print("✓ YDLidar library found and imported successfully")
except ImportError as e:
    YDLIDAR_AVAILABLE = False
    print(f"⚠ Warning: ydlidar module not found: {e}")
    print("Running in simulation mode.")


@dataclass
class LidarPoint:
    """Represents a single LIDAR measurement point"""
    angle: float        # Angle in radians
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


@dataclass
class LidarConfig:
    """Optimized configuration for YDLidar X2"""
    port: str = "/dev/ttyUSB1"
    baudrate: int = 115200
    lidar_type: str = "TYPE_TRIANGLE"  # X2 uses triangle ranging
    device_type: str = "YDLIDAR_TYPE_SERIAL"
    scan_frequency: float = 6.0  # Optimal for X2 (4-8 Hz range)
    sample_rate: int = 3  # X2 standard sample rate
    single_channel: bool = True  # X2 is single channel
    max_angle: float = 180.0
    min_angle: float = -180.0
    max_range: float = 8.0  # X2 max range
    min_range: float = 0.10  # X2 min range
    intensity: bool = False  # X2 doesn't support intensity
    auto_reconnect: bool = True
    fixed_resolution: bool = False
    reversion: bool = False
    inverted: bool = False
    support_motor_dtr_ctrl: bool = True  # X2 supports DTR control


class YDLidarX2Optimized:
    """
    Optimized YDLidar X2 interface based on GUI best practices
    """
    
    def __init__(self, config: Optional[LidarConfig] = None):
        """Initialize with optimized configuration"""
        self.config = config or LidarConfig()
        self.laser = None
        self.scan = None
        self.is_connected = False
        self.is_scanning = False
        self.simulation_mode = not YDLIDAR_AVAILABLE
        
        # Data management
        self.current_scan: Optional[LidarScan] = None
        self.scan_history = deque(maxlen=10)
        self.total_scans = 0
        self.scan_frequency = 0.0
        self.last_scan_time = time.time()
        
        # Threading
        self.scan_thread: Optional[threading.Thread] = None
        self.data_lock = threading.Lock()
        self.running = False
        
        # Setup logging
        self.logger = logging.getLogger('YDLidarX2Optimized')
        self.logger.setLevel(logging.INFO)
        
        if not self.simulation_mode:
            # Initialize SDK
            ydlidar.os_init()
    
    def auto_detect_port(self) -> str:
        """Auto-detect LIDAR port using optimized detection"""
        if self.simulation_mode:
            return "/dev/ttyUSB1"  # Default for simulation
        
        try:
            # Try SDK port detection first
            ports = ydlidar.lidarPortList()
            if ports:
                # Prefer USB ports (more reliable for actual hardware)
                usb_ports = [value for key, value in ports.items() if 'USB' in value]
                if usb_ports:
                    # Prefer ttyUSB1 if available
                    for port in usb_ports:
                        if 'USB1' in port:
                            return port
                    return usb_ports[0]  # Use first USB port
                else:
                    # Fall back to any available port
                    return list(ports.values())[0]
            
            # Manual fallback to common ports
            import os
            common_ports = ["/dev/ttyUSB1", "/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyACM1"]
            for test_port in common_ports:
                if os.path.exists(test_port):
                    return test_port
            
            return "/dev/ttyUSB1"  # Final fallback
            
        except Exception as e:
            self.logger.error(f"Port detection error: {e}")
            return "/dev/ttyUSB1"
    
    def connect(self) -> bool:
        """Connect to YDLidar X2 with optimized settings"""
        try:
            if self.simulation_mode:
                self.logger.info("🔌 Running in simulation mode")
                self.is_connected = True
                return True
            
            # Auto-detect port if needed
            if self.config.port == "auto" or self.config.port == "/dev/ttyUSB1":
                detected_port = self.auto_detect_port()
                self.logger.info(f"🔍 Using port: {detected_port}")
                actual_port = detected_port
            else:
                actual_port = self.config.port
            
            self.logger.info(f"🔌 Connecting to YDLidar X2 on {actual_port} at {self.config.baudrate} baud")
            
            # Create and configure laser with optimized settings
            self.laser = ydlidar.CYdLidar()
            
            # Basic connection settings
            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, actual_port)
            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.config.baudrate)
            
            # X2-specific optimized settings
            lidar_type = getattr(ydlidar, self.config.lidar_type)
            self.laser.setlidaropt(ydlidar.LidarPropLidarType, lidar_type)
            
            device_type = getattr(ydlidar, self.config.device_type)
            self.laser.setlidaropt(ydlidar.LidarPropDeviceType, device_type)
            
            # Performance settings
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, self.config.scan_frequency)
            self.laser.setlidaropt(ydlidar.LidarPropSampleRate, self.config.sample_rate)
            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, self.config.single_channel)
            
            # Range and angle settings
            self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, self.config.max_angle)
            self.laser.setlidaropt(ydlidar.LidarPropMinAngle, self.config.min_angle)
            self.laser.setlidaropt(ydlidar.LidarPropMaxRange, self.config.max_range)
            self.laser.setlidaropt(ydlidar.LidarPropMinRange, self.config.min_range)
            
            # X2 capabilities
            self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, self.config.intensity)
            self.laser.setlidaropt(ydlidar.LidarPropAutoReconnect, self.config.auto_reconnect)
            self.laser.setlidaropt(ydlidar.LidarPropFixedResolution, self.config.fixed_resolution)
            self.laser.setlidaropt(ydlidar.LidarPropReversion, self.config.reversion)
            self.laser.setlidaropt(ydlidar.LidarPropInverted, self.config.inverted)
            self.laser.setlidaropt(ydlidar.LidarPropSupportMotorDtrCtrl, self.config.support_motor_dtr_ctrl)
            
            # Initialize
            self.logger.info("⚙️  Initializing LIDAR...")
            ret = self.laser.initialize()
            if ret:
                self.logger.info("🚀 Starting LIDAR motor...")
                ret = self.laser.turnOn()
                if ret:
                    self.scan = ydlidar.LaserScan()
                    self.is_connected = True
                    self.logger.info("✅ YDLidar X2 connected and running successfully!")
                    return True
                else:
                    self.logger.error("❌ Failed to start LIDAR motor")
                    return False
            else:
                error_msg = "Unknown error"
                try:
                    error_msg = self.laser.DescribeError()
                except:
                    pass
                self.logger.error(f"❌ Failed to initialize LIDAR: {error_msg}")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ Connection failed: {e}")
            return False
    
    def start_scanning(self) -> bool:
        """Start continuous LIDAR scanning"""
        if self.is_scanning:
            self.logger.warning("⚠️  Scanning already in progress")
            return True
        
        if not self.is_connected:
            self.logger.error("❌ Not connected to LIDAR")
            return False
        
        try:
            self.running = True
            self.is_scanning = True
            self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self.scan_thread.start()
            
            self.logger.info("🚀 LIDAR scanning started")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to start scanning: {e}")
            self.is_scanning = False
            self.running = False
            return False
    
    def stop_scanning(self):
        """Stop LIDAR scanning"""
        if not self.is_scanning:
            return
        
        self.running = False
        self.is_scanning = False
        
        # Wait for thread to finish
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=2.0)
        
        self.logger.info("🛑 LIDAR scanning stopped")
    
    def _scan_loop(self):
        """Main scanning loop optimized for performance"""
        self.logger.info("🔄 Optimized scan loop started")
        frame_count = 0
        
        while self.running and self.is_connected:
            try:
                if self.simulation_mode:
                    # Generate realistic simulation data
                    angles = []
                    ranges = []
                    
                    # 360-degree scan with realistic patterns
                    for i in range(360):
                        angle_rad = math.radians(i - 180)  # -180 to +180 degrees
                        
                        # Create some realistic obstacles and walls
                        base_range = 3.0 + 1.0 * math.sin(angle_rad * 2)
                        
                        # Add obstacles at specific angles
                        if abs(angle_rad) < 0.3:  # Front obstacle
                            range_val = 1.5
                        elif abs(angle_rad - 1.5) < 0.2:  # Side obstacle
                            range_val = 2.0
                        else:
                            range_val = base_range
                        
                        # Add noise
                        range_val += 0.05 * (2 * (hash(i + frame_count) % 100) / 100 - 1)
                        range_val = max(self.config.min_range, min(self.config.max_range, range_val))
                        
                        angles.append(angle_rad)
                        ranges.append(range_val)
                    
                    # Convert to LidarPoints
                    points = []
                    for angle, range_val in zip(angles, ranges):
                        point = LidarPoint(
                            angle=angle,
                            distance=range_val,
                            quality=255,
                            timestamp=time.time()
                        )
                        points.append(point)
                    
                    # Create scan
                    scan_obj = LidarScan(
                        points=points,
                        scan_time=0.1,
                        scan_frequency=self.config.scan_frequency,
                        is_complete=True
                    )
                    
                    # Update data
                    with self.data_lock:
                        self.current_scan = scan_obj
                        self.scan_history.append(scan_obj)
                        self.total_scans += 1
                        frame_count += 1
                    
                    time.sleep(1.0 / self.config.scan_frequency)  # Maintain frequency
                    
                else:
                    # Real LIDAR data acquisition
                    if self.laser and self.scan:
                        r = self.laser.doProcessSimple(self.scan)
                        if r:
                            points = []
                            for point in self.scan.points:
                                lidar_point = LidarPoint(
                                    angle=point.angle,
                                    distance=point.range,
                                    quality=int(point.intensity) if hasattr(point, 'intensity') else 255,
                                    timestamp=time.time()
                                )
                                points.append(lidar_point)
                            
                            if points:  # Only process if we have data
                                scan_obj = LidarScan(
                                    points=points,
                                    scan_time=self.scan.config.scan_time if hasattr(self.scan, 'config') else 0.1,
                                    scan_frequency=self.config.scan_frequency,
                                    is_complete=True
                                )
                                
                                with self.data_lock:
                                    self.current_scan = scan_obj
                                    self.scan_history.append(scan_obj)
                                    self.total_scans += 1
                                    
                                    # Update frequency calculation
                                    current_time = time.time()
                                    if self.last_scan_time > 0:
                                        time_diff = current_time - self.last_scan_time
                                        if time_diff > 0:
                                            self.scan_frequency = 1.0 / time_diff
                                    self.last_scan_time = current_time
                        
                        time.sleep(0.02)  # 50 Hz polling for real hardware
                    
            except Exception as e:
                self.logger.error(f"❌ Scan loop error: {e}")
                time.sleep(0.1)
        
        self.logger.info("🔄 Scan loop ended")
    
    def get_latest_scan(self) -> Optional[LidarScan]:
        """Get the most recent complete scan"""
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
                'configured_frequency': self.config.scan_frequency,
                'is_scanning': self.is_scanning,
                'connected': self.is_connected,
                'simulation_mode': self.simulation_mode,
                'history_length': len(self.scan_history),
                'port': self.config.port,
                'baudrate': self.config.baudrate
            }
    
    def is_connected_status(self) -> bool:
        """Check connection status"""
        return self.is_connected
    
    def disconnect(self):
        """Disconnect from LIDAR"""
        self.stop_scanning()
        
        if self.laser and not self.simulation_mode:
            try:
                self.laser.turnOff()
                self.laser.disconnecting()
                self.logger.info("🔌 LIDAR disconnected")
            except Exception as e:
                self.logger.error(f"❌ Disconnect error: {e}")
        
        self.is_connected = False
    
    def cleanup(self):
        """Cleanup resources"""
        self.disconnect()
        self.logger.info("🧹 Cleanup completed")


# Test function optimized for X2
def test_optimized_interface():
    """Test the optimized YDLidar X2 interface"""
    logging.basicConfig(level=logging.INFO)
    
    print("🚀 Testing YDLidar X2 Optimized Interface")
    print("=" * 50)
    
    # Create optimized configuration
    config = LidarConfig(
        port="/dev/ttyUSB1",
        baudrate=115200,
        scan_frequency=6.0,  # Optimal for X2
        sample_rate=3,       # X2 standard
        single_channel=True, # X2 characteristic
        support_motor_dtr_ctrl=True
    )
    
    lidar = YDLidarX2Optimized(config)
    
    try:
        print(f"🔧 Configuration: {config.port} @ {config.baudrate} baud, {config.scan_frequency} Hz")
        
        if lidar.connect():
            print("✅ Connected successfully!")
            
            if lidar.start_scanning():
                print("✅ Scanning started! Monitoring for 10 seconds...")
                
                start_time = time.time()
                last_report = start_time
                
                while time.time() - start_time < 10:
                    scan = lidar.get_latest_scan()
                    stats = lidar.get_statistics()
                    
                    # Report every 2 seconds
                    if time.time() - last_report >= 2.0:
                        if scan:
                            print(f"📊 Scan #{stats['total_scans']}: "
                                  f"{len(scan.points)} points, "
                                  f"Actual: {stats['scan_frequency']:.1f} Hz, "
                                  f"Target: {stats['configured_frequency']} Hz")
                        else:
                            print("⏳ Waiting for scan data...")
                        
                        last_report = time.time()
                    
                    time.sleep(0.1)
                
                print("✅ Test completed successfully!")
            else:
                print("❌ Failed to start scanning")
                if not lidar.simulation_mode:
                    print("💡 Check LIDAR power supply (5V) and USB connection")
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
        print("✅ Test completed")


if __name__ == "__main__":
    test_optimized_interface()

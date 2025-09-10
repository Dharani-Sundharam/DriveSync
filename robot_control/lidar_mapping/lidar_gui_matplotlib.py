#!/usr/bin/env python3
"""
LIDAR GUI with Matplotlib Plotting
==================================

Simple GUI for YDLidar X2 using matplotlib for proper plotting,
based on the comprehensive GUI reference code provided.
"""

import sys
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import logging
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass

# Handle both relative and absolute imports
try:
    from .ydlidar_x2_optimized import YDLidarX2Optimized, LidarScan, LidarConfig
except ImportError:
    from ydlidar_x2_optimized import YDLidarX2Optimized, LidarScan, LidarConfig

try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QPushButton, QLabel, QSlider, QCheckBox
    )
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtGui import QFont
    PYQT_AVAILABLE = True
except ImportError:
    print("⚠️ PyQt5 not available, using matplotlib-only mode")
    PYQT_AVAILABLE = False


class LidarVisualizationWidget:
    """Simple matplotlib-based LIDAR visualization"""
    
    def __init__(self):
        self.angles = []
        self.ranges = []
        self.max_points = 1000
        
        # Create matplotlib figure
        self.figure = Figure(figsize=(10, 8))
        self.ax = None
        self.init_plot()
        
    def init_plot(self):
        """Initialize the plot"""
        self.figure.clear()
        self.ax = self.figure.add_subplot(111, projection='polar')
        self.ax.set_title('YDLidar X2 - Real-time Data', fontsize=14, fontweight='bold')
        self.ax.set_rmax(8.0)  # X2 max range
        self.ax.grid(True)
        self.ax.set_theta_zero_location('N')  # 0 degrees at top
        self.ax.set_theta_direction(-1)  # Clockwise
        
    def update_data(self, angles, ranges):
        """Update with new LIDAR data"""
        # Keep only recent data for performance
        self.angles = angles[-self.max_points:]
        self.ranges = ranges[-self.max_points:]
        self.update_plot()
    
    def update_plot(self):
        """Update the plot with current data"""
        if not self.angles or not self.ranges:
            return
        
        self.ax.clear()
        self.ax.set_title('YDLidar X2 - Real-time Data', fontsize=14, fontweight='bold')
        self.ax.set_rmax(8.0)
        self.ax.grid(True)
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        
        # Filter data by range
        filtered_angles = []
        filtered_ranges = []
        
        for angle, range_val in zip(self.angles, self.ranges):
            if 0.05 <= range_val <= 8.0:  # Valid range for X2
                filtered_angles.append(angle)
                filtered_ranges.append(range_val)
        
        if filtered_angles:
            # Plot as scatter points
            self.ax.scatter(filtered_angles, filtered_ranges, 
                           c=filtered_ranges, cmap='plasma', 
                           alpha=0.8, s=2)


class SimpleLidarGUI:
    """Simple LIDAR GUI using matplotlib"""
    
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        
        # LIDAR interface
        config = LidarConfig(port=port, baudrate=baudrate)
        self.lidar = YDLidarX2Optimized(config)
        
        # Visualization
        self.viz = LidarVisualizationWidget()
        
        # Control variables
        self.running = False
        self.connected = False
        
        # Setup logging
        self.logger = logging.getLogger('SimpleLidarGUI')
        self.logger.setLevel(logging.INFO)
        
        # Setup matplotlib for interactive mode
        plt.ion()  # Turn on interactive mode
        
    def connect_lidar(self) -> bool:
        """Connect to LIDAR"""
        try:
            self.logger.info("🔌 Connecting to LIDAR...")
            if self.lidar.connect():
                if self.lidar.start_scanning():
                    self.connected = True
                    self.logger.info("✅ LIDAR connected and scanning")
                    return True
                else:
                    self.logger.error("❌ Failed to start scanning")
            else:
                self.logger.error("❌ Failed to connect to LIDAR")
        except Exception as e:
            self.logger.error(f"❌ Connection error: {e}")
        
        return False
    
    def disconnect_lidar(self):
        """Disconnect from LIDAR"""
        if self.lidar:
            self.lidar.cleanup()
        self.connected = False
        self.logger.info("🔌 LIDAR disconnected")
    
    def update_plot(self):
        """Update the plot with latest LIDAR data"""
        if not self.connected:
            return
        
        try:
            scan = self.lidar.get_latest_scan()
            if scan and scan.points:
                angles = [point.angle for point in scan.points]
                ranges = [point.distance for point in scan.points]
                
                self.viz.update_data(angles, ranges)
                
                # Force matplotlib to update
                self.viz.figure.canvas.draw()
                self.viz.figure.canvas.flush_events()
                
        except Exception as e:
            self.logger.error(f"❌ Plot update error: {e}")
    
    def run_simple(self):
        """Run simple matplotlib-only version"""
        self.logger.info("🚀 Starting Simple LIDAR GUI (matplotlib-only)")
        
        if not self.connect_lidar():
            self.logger.error("❌ Failed to connect to LIDAR")
            return
        
        # Show the plot
        plt.figure(self.viz.figure.number)
        plt.show()
        
        self.running = True
        update_count = 0
        
        try:
            while self.running:
                self.update_plot()
                
                # Print status every 50 updates
                update_count += 1
                if update_count % 50 == 0:
                    scan = self.lidar.get_latest_scan()
                    if scan:
                        stats = self.lidar.get_statistics()
                        self.logger.info(f"📊 Update {update_count}: "
                                       f"{len(scan.points)} points, "
                                       f"{stats['scan_frequency']:.1f} Hz")
                
                # Check if plot window is still open
                if not plt.fignum_exists(self.viz.figure.number):
                    self.logger.info("👋 Plot window closed")
                    break
                
                time.sleep(0.1)  # 10 Hz update rate
                
        except KeyboardInterrupt:
            self.logger.info("👋 Interrupted by user")
        except Exception as e:
            self.logger.error(f"❌ Runtime error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        self.disconnect_lidar()
        plt.close('all')
        self.logger.info("🧹 Cleanup completed")


if PYQT_AVAILABLE:
    class LidarMainWindow(QMainWindow):
        """Main window with PyQt5 controls and matplotlib plot"""
        
        def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
            super().__init__()
            self.port = port
            self.baudrate = baudrate
            
            # LIDAR interface
            config = LidarConfig(port=port, baudrate=baudrate)
            self.lidar = YDLidarX2Optimized(config)
            
            # Setup UI
            self.setup_ui()
            
            # Control variables
            self.connected = False
            
            # Setup timer for updates
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_plot)
            
            # Setup logging
            self.logger = logging.getLogger('LidarMainWindow')
            self.logger.setLevel(logging.INFO)
        
        def setup_ui(self):
            """Setup the user interface"""
            self.setWindowTitle("YDLidar X2 GUI - Real-time Visualization")
            self.setGeometry(100, 100, 1000, 700)
            
            # Central widget
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            
            # Main layout
            layout = QVBoxLayout()
            
            # Control panel
            control_layout = QHBoxLayout()
            
            self.connect_btn = QPushButton("Connect to LIDAR")
            self.connect_btn.clicked.connect(self.toggle_connection)
            control_layout.addWidget(self.connect_btn)
            
            self.status_label = QLabel("Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            control_layout.addWidget(self.status_label)
            
            control_layout.addStretch()
            
            # Range control
            control_layout.addWidget(QLabel("Max Range:"))
            self.range_slider = QSlider(Qt.Horizontal)
            self.range_slider.setMinimum(1)
            self.range_slider.setMaximum(8)
            self.range_slider.setValue(8)
            self.range_slider.valueChanged.connect(self.update_range)
            control_layout.addWidget(self.range_slider)
            
            self.range_label = QLabel("8.0m")
            control_layout.addWidget(self.range_label)
            
            layout.addLayout(control_layout)
            
            # Visualization widget
            self.viz = LidarVisualizationWidget()
            self.canvas = FigureCanvas(self.viz.figure)
            layout.addWidget(self.canvas)
            
            central_widget.setLayout(layout)
        
        def toggle_connection(self):
            """Toggle LIDAR connection"""
            if not self.connected:
                self.connect_lidar()
            else:
                self.disconnect_lidar()
        
        def connect_lidar(self):
            """Connect to LIDAR"""
            try:
                self.logger.info("🔌 Connecting to LIDAR...")
                self.connect_btn.setText("Connecting...")
                self.connect_btn.setEnabled(False)
                
                if self.lidar.connect():
                    if self.lidar.start_scanning():
                        self.connected = True
                        self.connect_btn.setText("Disconnect")
                        self.connect_btn.setEnabled(True)
                        self.status_label.setText("Connected")
                        self.status_label.setStyleSheet("color: green; font-weight: bold;")
                        
                        # Start update timer
                        self.timer.start(100)  # 10 Hz
                        
                        self.logger.info("✅ LIDAR connected and scanning")
                    else:
                        self.logger.error("❌ Failed to start scanning")
                        self.reset_connection_ui()
                else:
                    self.logger.error("❌ Failed to connect to LIDAR")
                    self.reset_connection_ui()
            except Exception as e:
                self.logger.error(f"❌ Connection error: {e}")
                self.reset_connection_ui()
        
        def disconnect_lidar(self):
            """Disconnect from LIDAR"""
            self.timer.stop()
            if self.lidar:
                self.lidar.cleanup()
            self.connected = False
            self.reset_connection_ui()
            self.logger.info("🔌 LIDAR disconnected")
        
        def reset_connection_ui(self):
            """Reset connection UI to disconnected state"""
            self.connect_btn.setText("Connect to LIDAR")
            self.connect_btn.setEnabled(True)
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
        
        def update_range(self):
            """Update the maximum range"""
            range_val = self.range_slider.value()
            self.range_label.setText(f"{range_val}.0m")
            self.viz.ax.set_rmax(range_val)
            self.canvas.draw()
        
        def update_plot(self):
            """Update the plot with latest LIDAR data"""
            if not self.connected:
                return
            
            try:
                scan = self.lidar.get_latest_scan()
                if scan and scan.points:
                    angles = [point.angle for point in scan.points]
                    ranges = [point.distance for point in scan.points]
                    
                    self.viz.update_data(angles, ranges)
                    self.canvas.draw()
                    
            except Exception as e:
                self.logger.error(f"❌ Plot update error: {e}")
        
        def closeEvent(self, event):
            """Handle window close"""
            self.disconnect_lidar()
            event.accept()


def main():
    """Main application entry point"""
    logging.basicConfig(level=logging.INFO, 
                       format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    
    if PYQT_AVAILABLE:
        # Use PyQt5 version with controls
        app = QApplication(sys.argv)
        window = LidarMainWindow('/dev/ttyUSB1', 115200)
        window.show()
        sys.exit(app.exec_())
    else:
        # Use simple matplotlib version
        gui = SimpleLidarGUI('/dev/ttyUSB1', 115200)
        gui.run_simple()


if __name__ == "__main__":
    main()

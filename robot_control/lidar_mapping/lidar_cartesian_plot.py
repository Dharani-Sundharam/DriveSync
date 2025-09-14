#!/usr/bin/env python3
"""
Simple LIDAR Cartesian Plot
==========================

Creates a real-time cartesian plot of LIDAR data points.
Robot is centered at origin, obstacles plotted around it.
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
import math
from ydlidar_x2_optimized import YDLidarX2Optimized, LidarConfig

class LidarCartesianPlot:
    def __init__(self, lidar_port='/dev/ttyUSB1', max_range=6.0, 
                 front_distance=0.30, side_distance=0.25, back_distance=0.20):
        # Initialize LIDAR
        config = LidarConfig(port=lidar_port, baudrate=115200)
        self.lidar = YDLidarX2Optimized(config)
        self.max_range = max_range
        
        # Directional safety distances
        self.front_distance = front_distance
        self.side_distance = side_distance
        self.back_distance = back_distance
        self.obstacle_detected = False
        
        # Setup plot for 480x320 screen (4.8x3.2 inches at 100 DPI)
        self.fig, self.ax = plt.subplots(figsize=(4.8, 3.2), dpi=100)
        self.ax.set_xlim(-max_range, max_range)
        self.ax.set_ylim(-max_range, max_range)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3, linewidth=0.5)
        self.ax.set_xlabel('X (m)', fontsize=8)
        self.ax.set_ylabel('Y (m)', fontsize=8)
        self.ax.set_title('LIDAR Cartesian Plot', fontsize=10)
        self.ax.tick_params(labelsize=6)
        
        # Tight layout for small screen
        self.fig.tight_layout(pad=0.5)
        
        # Plot elements - all points with no compromise in quality
        self.scatter = self.ax.scatter([], [], c='red', s=1, alpha=0.8, marker='.')
        self.robot_marker = self.ax.scatter([0], [0], c='blue', s=20, marker='+', zorder=10)
        
        # Robot front direction arrow (pointing to 0° = front)
        arrow_length = self.front_distance * 0.8
        self.front_arrow = self.ax.annotate('', xy=(arrow_length, 0), xytext=(0, 0),
                                           arrowprops=dict(arrowstyle='->', color='blue', 
                                                         lw=2, alpha=0.8), zorder=12)
        
        # Safety zones - rectangular areas for different directions
        from matplotlib.patches import Rectangle
        # Front zone (right side of robot)
        self.front_zone = Rectangle((0, -self.side_distance/2), self.front_distance, self.side_distance, 
                                   fill=False, color='green', linewidth=2, alpha=0.7, zorder=5)
        # Back zone (left side of robot)  
        self.back_zone = Rectangle((-self.back_distance, -self.side_distance/2), self.back_distance, self.side_distance,
                                  fill=False, color='green', linewidth=2, alpha=0.7, zorder=5)
        # Left zone (top)
        self.left_zone = Rectangle((-self.back_distance, 0), self.front_distance + self.back_distance, self.side_distance,
                                  fill=False, color='green', linewidth=2, alpha=0.7, zorder=5)
        # Right zone (bottom)
        self.right_zone = Rectangle((-self.back_distance, -self.side_distance), self.front_distance + self.back_distance, self.side_distance,
                                   fill=False, color='green', linewidth=2, alpha=0.7, zorder=5)
        
        self.ax.add_patch(self.front_zone)
        self.ax.add_patch(self.back_zone)
        self.ax.add_patch(self.left_zone)
        self.ax.add_patch(self.right_zone)
        
        # Obstacle warning text
        self.warning_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes, 
                                        fontsize=10, fontweight='bold', 
                                        verticalalignment='top', zorder=15)
        
        # Data storage
        self.x_data = []
        self.y_data = []
        
    def connect_and_start(self):
        """Connect to LIDAR and start scanning"""
        if self.lidar.connect():
            if self.lidar.start_scanning():
                print("✅ LIDAR connected and scanning")
                return True
            else:
                print("❌ Failed to start scanning")
                return False
        else:
            print("❌ Failed to connect to LIDAR")
            return False
    
    def update_plot(self, frame):
        """Update plot with latest LIDAR data - ALL points, no filtering"""
        scan = self.lidar.get_latest_scan()
        
        if scan and scan.points:
            # Convert ALL polar to cartesian coordinates - no quality compromise
            x_points = []
            y_points = []
            obstacles_in_radius = []
            
            for point in scan.points:
                # Only filter by basic range limits, plot ALL valid points
                if 0.05 <= point.distance <= self.max_range:
                    x = point.distance * math.cos(point.angle)
                    y = -point.distance * math.sin(point.angle)  # Flip Y to correct left/right
                    x_points.append(x)
                    y_points.append(y)
                    
                    # Check if point is within directional safety zones
                    if self.is_point_in_safety_zone(x, y):
                        obstacles_in_radius.append((x, y, point.distance))
            
            # Update scatter plot with ALL points
            if x_points:
                self.scatter.set_offsets(np.column_stack([x_points, y_points]))
            
            # Update obstacle detection
            self.obstacle_detected = len(obstacles_in_radius) > 0
            
            # Update safety zones color and warning
            if self.obstacle_detected:
                self.front_zone.set_color('red')
                self.back_zone.set_color('red')
                self.left_zone.set_color('red')
                self.right_zone.set_color('red')
                for zone in [self.front_zone, self.back_zone, self.left_zone, self.right_zone]:
                    zone.set_linewidth(3)
                closest_obstacle = min(obstacles_in_radius, key=lambda p: p[2])
                self.warning_text.set_text(f'⚠ OBSTACLE DETECTED\nDist: {closest_obstacle[2]:.2f}m\nF:{self.front_distance:.2f} S:{self.side_distance:.2f} B:{self.back_distance:.2f}')
                self.warning_text.set_color('red')
            else:
                self.front_zone.set_color('green')
                self.back_zone.set_color('green')
                self.left_zone.set_color('green')
                self.right_zone.set_color('green')
                for zone in [self.front_zone, self.back_zone, self.left_zone, self.right_zone]:
                    zone.set_linewidth(2)
                self.warning_text.set_text(f'✓ CLEAR\nF:{self.front_distance:.2f} S:{self.side_distance:.2f} B:{self.back_distance:.2f}')
                self.warning_text.set_color('green')
            
            # Compact title for small screen
            stats = self.lidar.get_statistics()
            status = "⚠" if self.obstacle_detected else "✓"
            self.ax.set_title(f'{status} LIDAR - {len(x_points)}pts - {stats["scan_frequency"]:.1f}Hz', fontsize=8)
        
        return self.scatter, self.robot_marker
    
    def run(self):
        """Start real-time plotting"""
        if self.connect_and_start():
            # Start animation
            ani = animation.FuncAnimation(
                self.fig, self.update_plot, interval=100, blit=False, cache_frame_data=False
            )
            plt.show()
        else:
            print("Failed to start LIDAR system")
    
    def is_point_in_safety_zone(self, x, y):
        """Check if point is within any directional safety zone"""
        # Front zone (x > 0, within side distance)
        if x > 0 and x <= self.front_distance and abs(y) <= self.side_distance:
            return True
        # Back zone (x < 0, within side distance)  
        if x < 0 and x >= -self.back_distance and abs(y) <= self.side_distance:
            return True
        # Left zone (y > 0, within front/back range)
        if y > 0 and y <= self.side_distance and -self.back_distance <= x <= self.front_distance:
            return True
        # Right zone (y < 0, within front/back range)
        if y < 0 and y >= -self.side_distance and -self.back_distance <= x <= self.front_distance:
            return True
        return False
    
    def set_safety_distances(self, front=None, side=None, back=None):
        """Change safety distances for obstacle detection"""
        if front is not None:
            self.front_distance = front
        if side is not None:
            self.side_distance = side
        if back is not None:
            self.back_distance = back
        print(f"Safety distances updated - Front: {self.front_distance:.2f}m, Side: {self.side_distance:.2f}m, Back: {self.back_distance:.2f}m")
    
    def cleanup(self):
        """Cleanup resources"""
        self.lidar.cleanup()

def main():
    # Directional safety distances
    front_distance = 0.30   # 30cm front
    side_distance = 0.25    # 25cm left/right
    back_distance = 0.20    # 20cm back
    
    plotter = LidarCartesianPlot(front_distance=front_distance, 
                                side_distance=side_distance, 
                                back_distance=back_distance)
    
    print(f"Starting LIDAR plot with safety distances:")
    print(f"Front: {front_distance}m, Sides: {side_distance}m, Back: {back_distance}m")
    print("Adjust distance parameters in main() to change detection areas")
    
    try:
        plotter.run()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        plotter.cleanup()

if __name__ == "__main__":
    main()

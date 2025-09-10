"""
LIDAR Mapping System for YDLIDAR X2
====================================

This module provides LIDAR-based mapping functionality using the YDLIDAR X2 sensor.
Integrates with the pathfinding robot controller for real-time mapping and visualization.
"""

__version__ = "1.0.0"
__author__ = "Robot Control System"

try:
    # Try to use official SDK interface first
    from .ydlidar_interface_sdk import YDLidarX2Interface
except ImportError:
    # Fallback to custom interface if SDK not available
    from .ydlidar_interface import YDLidarX2Interface
from .lidar_mapper import LidarMapper
from .lidar_gui import LidarMappingGUI

__all__ = [
    'YDLidarX2Interface',
    'LidarMapper', 
    'LidarMappingGUI'
]

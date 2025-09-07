#!/usr/bin/env python3
"""
Headless YOLO Object Detection - CLI Only
- Ultra-fast detection without GUI display
- Continuously prints detected objects to CLI
- Real-time object names, confidence scores, and counts
- Optimized for maximum performance on Raspberry Pi
"""

import cv2
import numpy as np
import time
from ultralytics import YOLO
import logging
import argparse
import sys
from datetime import datetime
import os

class YOLOHeadlessDetector:
    """Ultra-fast headless object detection using YOLO model"""
    
    def __init__(self, model_name='yolov8n.pt', camera_id=0, confidence_threshold=0.3, print_interval=0.1):
        """
        Initialize headless YOLO detector
        
        Args:
            model_name: YOLO model to use (yolov8n.pt for maximum speed)
            camera_id: Camera device ID (0 for default camera)
            confidence_threshold: Minimum confidence for detection
            print_interval: How often to print results (seconds)
        """
        self.model_name = model_name
        self.camera_id = camera_id
        self.confidence_threshold = confidence_threshold
        self.print_interval = print_interval
        
        # Setup logging
        self.setup_logging()
        self.logger = logging.getLogger('YOLOHeadless')
        
        # Expanded useful object classes (COCO dataset IDs)
        self.useful_classes = {
            # Vehicles
            2: 'car',           
            3: 'motorcycle',    
            5: 'bus',           
            7: 'truck',         
            
            # Road infrastructure
            9: 'traffic light', 
            11: 'stop sign',    
            12: 'parking meter',
            13: 'bench',
            
            # People and mobility
            0: 'person',        
            1: 'bicycle',       
            
            # Animals (safety)
            16: 'bird',
            17: 'cat',
            18: 'dog',
            
            # Common objects
            32: 'sports ball',
            39: 'bottle',
            41: 'cup',
            64: 'potted plant',
            65: 'bed',
            67: 'dining table',
            72: 'tv',
            73: 'laptop',
            74: 'mouse',
            76: 'keyboard',
        }
        
        # Performance tracking
        self.fps_counter = 0
        self.fps_time = time.time()
        self.current_fps = 0
        self.last_print_time = time.time()
        self.total_detections = 0
        self.session_start_time = time.time()
        
        # Detection storage for CLI output
        self.current_detections = []
        
        # Initialize camera and YOLO model
        self.init_camera()
        self.init_yolo_model()
        
    def setup_logging(self):
        """Setup logging configuration"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler(sys.stdout)
            ]
        )
    
    def init_camera(self):
        """Initialize camera capture with maximum performance settings"""
        self.logger.info(f"🎥 Initializing ultra-high-speed camera {self.camera_id}...")
        
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            self.logger.error(f"❌ Failed to open camera {self.camera_id}")
            sys.exit(1)
        
        # MAXIMUM PERFORMANCE camera settings
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)   # Small = ultra fast
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Small = ultra fast
        self.cap.set(cv2.CAP_PROP_FPS, 60)            # Maximum FPS
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # Zero lag
        
        # Speed optimizations
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        
        # Get actual properties
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.logger.info(f"✅ Camera: {width}x{height} @ {fps} FPS (HEADLESS MODE)")
        self.logger.info(f"⚡ Maximum performance - No GUI overhead!")
        
    def init_yolo_model(self):
        """Initialize YOLO model with speed optimizations"""
        self.logger.info(f"🤖 Loading YOLO model: {self.model_name}")
        
        try:
            self.model = YOLO(self.model_name)
            self.logger.info(f"✅ YOLO model loaded - {len(self.useful_classes)} object types")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to load YOLO model: {e}")
            sys.exit(1)
    
    def process_detections(self, detections):
        """Process detections and store for CLI output"""
        if detections is None:
            self.current_detections = []
            return
        
        detected_objects = []
        
        for detection in detections:
            # Extract detection information
            confidence = float(detection.conf[0])
            class_id = int(detection.cls[0])
            
            # Filter useful objects only
            if class_id not in self.useful_classes:
                continue
                
            # Skip low confidence
            if confidence < self.confidence_threshold:
                continue
            
            class_name = self.useful_classes[class_id]
            
            # Get bounding box for size info
            x1, y1, x2, y2 = map(int, detection.xyxy[0])
            width = x2 - x1
            height = y2 - y1
            area = width * height
            
            detected_objects.append({
                'name': class_name,
                'confidence': confidence,
                'bbox': (x1, y1, x2, y2),
                'size': f"{width}x{height}",
                'area': area
            })
        
        # Sort by confidence (highest first)
        detected_objects.sort(key=lambda x: x['confidence'], reverse=True)
        self.current_detections = detected_objects
        self.total_detections += len(detected_objects)
    
    def update_fps(self):
        """Update FPS counter"""
        self.fps_counter += 1
        current_time = time.time()
        
        if current_time - self.fps_time >= 1.0:
            self.current_fps = self.fps_counter / (current_time - self.fps_time)
            self.fps_counter = 0
            self.fps_time = current_time
    
    def print_detections(self):
        """Print current detections to CLI"""
        current_time = time.time()
        
        # Only print at specified intervals
        if current_time - self.last_print_time < self.print_interval:
            return
            
        # Clear screen for clean output
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Header
        runtime = current_time - self.session_start_time
        print("=" * 80)
        print(f"⚡ YOLO HEADLESS DETECTION - ULTRA HIGH SPEED")
        print(f"🕒 Runtime: {runtime:.1f}s | 📊 FPS: {self.current_fps:.1f} | 🎯 Total Detected: {self.total_detections}")
        print("=" * 80)
        
        if not self.current_detections:
            print("🔍 No objects detected")
        else:
            print(f"📋 DETECTED OBJECTS ({len(self.current_detections)}):")
            print("-" * 80)
            
            # Group by object type
            object_counts = {}
            for obj in self.current_detections:
                name = obj['name']
                if name not in object_counts:
                    object_counts[name] = []
                object_counts[name].append(obj)
            
            # Print grouped objects
            for obj_type, objects in object_counts.items():
                count = len(objects)
                avg_conf = sum(obj['confidence'] for obj in objects) / count
                max_conf = max(obj['confidence'] for obj in objects)
                
                print(f"🎯 {obj_type.upper()}: {count} detected")
                print(f"   📊 Confidence: avg={avg_conf:.2f}, max={max_conf:.2f}")
                
                # Show individual detections if few objects
                if count <= 3:
                    for i, obj in enumerate(objects, 1):
                        print(f"   #{i}: {obj['confidence']:.2f} confidence, size={obj['size']}")
                
                print()
        
        print("=" * 80)
        print("🎮 Controls: Ctrl+C to quit")
        print("=" * 80)
        
        self.last_print_time = current_time
    
    def run(self):
        """Main headless detection loop"""
        self.logger.info("🚀 Starting ULTRA-HIGH-SPEED headless detection...")
        self.logger.info(f"🎯 Detecting {len(self.useful_classes)} object types")
        self.logger.info(f"📊 Print interval: {self.print_interval}s")
        self.logger.info(f"🎮 Press Ctrl+C to quit")
        
        try:
            while True:
                # Read frame from camera
                ret, frame = self.cap.read()
                if not ret:
                    self.logger.error("❌ Failed to read frame from camera")
                    break
                
                # ULTRA-HIGH-SPEED YOLO detection
                results = self.model(frame, 
                                   conf=self.confidence_threshold, 
                                   verbose=False,
                                   imgsz=320,  # Maximum speed
                                   classes=list(self.useful_classes.keys()),
                                   device='cpu',
                                   half=False,
                                   max_det=30)  # Limit for speed
                
                detections = results[0].boxes if len(results) > 0 and results[0].boxes is not None else None
                
                # Process detections
                self.process_detections(detections)
                
                # Update FPS
                self.update_fps()
                
                # Print results to CLI
                self.print_detections()
                
                # Small delay to prevent CPU overload
                time.sleep(0.01)
                    
        except KeyboardInterrupt:
            self.logger.info("🛑 Detection stopped by user")
        except Exception as e:
            self.logger.error(f"❌ Error during detection: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.logger.info("🧹 Cleaning up...")
        
        if hasattr(self, 'cap'):
            self.cap.release()
        
        # Final statistics
        runtime = time.time() - self.session_start_time
        avg_fps = self.total_detections / runtime if runtime > 0 else 0
        
        print("\n" + "=" * 60)
        print("📊 FINAL STATISTICS")
        print("=" * 60)
        print(f"⏱️  Total Runtime: {runtime:.1f} seconds")
        print(f"🎯 Total Objects Detected: {self.total_detections}")
        print(f"📊 Average Detection Rate: {avg_fps:.1f} objects/second")
        print(f"🚀 Final FPS: {self.current_fps:.1f}")
        print("=" * 60)
        
        self.logger.info("✅ Cleanup complete")

def main():
    """Main function with command line arguments"""
    parser = argparse.ArgumentParser(description='Ultra-Fast Headless YOLO Object Detection')
    parser.add_argument('--model', default='yolov8n.pt', 
                       help='YOLO model (yolov8n.pt for speed)')
    parser.add_argument('--camera', type=int, default=0, 
                       help='Camera device ID (default: 0)')
    parser.add_argument('--confidence', type=float, default=0.3, 
                       help='Confidence threshold (default: 0.3)')
    parser.add_argument('--interval', type=float, default=0.1,
                       help='Print interval in seconds (default: 0.1)')
    parser.add_argument('--list-cameras', action='store_true',
                       help='List available cameras and exit')
    
    args = parser.parse_args()
    
    # List cameras if requested
    if args.list_cameras:
        print("🎥 Available cameras:")
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"  Camera {i}: Available")
                cap.release()
            else:
                break
        return
    
    # Create and run headless detector
    detector = YOLOHeadlessDetector(
        model_name=args.model,
        camera_id=args.camera,
        confidence_threshold=args.confidence,
        print_interval=args.interval
    )
    
    detector.run()

if __name__ == "__main__":
    main()

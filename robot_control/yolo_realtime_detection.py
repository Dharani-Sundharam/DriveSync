#!/usr/bin/env python3
"""
Real-time Object Detection using YOLO
- Detects all objects in real-time from camera feed
- Shows bounding boxes, labels, and confidence scores
- Displays FPS counter and object count
- Keyboard controls for settings and exit
"""

import cv2
import numpy as np
import time
from ultralytics import YOLO
import logging
from datetime import datetime
import argparse
import sys

class YOLORealtimeDetector:
    """High-performance road object detection using YOLO model"""
    
    def __init__(self, model_name='yolov8n.pt', camera_id=0, confidence_threshold=0.4):
        """
        Initialize YOLO detector optimized for road objects
        
        Args:
            model_name: YOLO model to use (yolov8n.pt for speed, yolov8s.pt for balance)
            camera_id: Camera device ID (0 for default camera)
            confidence_threshold: Minimum confidence for detection (higher = faster)
        """
        self.model_name = model_name
        self.camera_id = camera_id
        self.confidence_threshold = confidence_threshold
        
        # Setup logging
        self.setup_logging()
        self.logger = logging.getLogger('YOLORoadDetector')
        
        # Detection settings - optimized for speed
        self.show_confidence = True
        self.show_fps = True
        self.show_count = True
        self.save_detections = False
        
        # Expanded useful object classes (COCO dataset IDs)
        # Include more objects that are commonly encountered
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
            0: 'person',        # Added back - useful for detection
            1: 'bicycle',       # Added back - common on roads
            
            # Animals (safety)
            16: 'bird',
            17: 'cat',
            18: 'dog',
            
            # Common objects
            32: 'sports ball',  # balls on road
            39: 'bottle',       # litter/objects
            41: 'cup',          # common objects
            64: 'potted plant', # street furniture
            65: 'bed',          # furniture (moving)
            67: 'dining table', # furniture (moving)
            72: 'tv',           # electronics
            73: 'laptop',       # electronics
            74: 'mouse',        # electronics
            76: 'keyboard',     # electronics
        }
        
        # FPS tracking
        self.fps_counter = 0
        self.fps_time = time.time()
        self.current_fps = 0
        
        # Colors for useful object classes (BGR format)
        self.object_colors = {
            # Vehicles (bright colors)
            2: (0, 255, 0),      # car - Green
            3: (255, 0, 0),      # motorcycle - Blue  
            5: (0, 255, 255),    # bus - Yellow
            7: (0, 0, 255),      # truck - Red
            
            # Infrastructure (cyan/purple tones)
            9: (255, 255, 0),    # traffic light - Cyan
            11: (0, 0, 255),     # stop sign - Red
            12: (128, 0, 128),   # parking meter - Purple
            13: (255, 165, 0),   # bench - Orange
            
            # People/mobility (warm colors)
            0: (0, 255, 255),    # person - Yellow
            1: (255, 0, 255),    # bicycle - Magenta
            
            # Animals (natural colors)
            16: (0, 128, 255),   # bird - Light Blue
            17: (255, 128, 0),   # cat - Orange
            18: (128, 255, 0),   # dog - Light Green
            
            # Objects (cool colors)
            32: (255, 192, 203), # sports ball - Pink
            39: (0, 255, 127),   # bottle - Spring Green
            41: (255, 20, 147),  # cup - Deep Pink
            64: (34, 139, 34),   # potted plant - Forest Green
            65: (139, 69, 19),   # bed - Saddle Brown
            67: (160, 82, 45),   # dining table - Saddle Brown
            72: (25, 25, 112),   # tv - Midnight Blue
            73: (72, 61, 139),   # laptop - Dark Slate Blue
            74: (128, 128, 128), # mouse - Gray
            76: (105, 105, 105), # keyboard - Dim Gray
        }
        
        # Performance optimizations
        self.skip_frames = 0  # Process every frame for max FPS
        self.frame_count = 0
        
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
        """Initialize camera capture with high-performance settings"""
        self.logger.info(f"🎥 Initializing high-speed camera {self.camera_id}...")
        
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            self.logger.error(f"❌ Failed to open camera {self.camera_id}")
            sys.exit(1)
        
        # ULTRA HIGH-PERFORMANCE camera settings
        # Even smaller resolution = maximum FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)   # Smaller = faster
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # 4:3 aspect ratio
        self.cap.set(cv2.CAP_PROP_FPS, 60)            # Request higher FPS
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # Minimize buffer lag
        
        # Aggressive performance optimizations
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Faster exposure
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)         # Disable autofocus
        
        # Get actual camera properties
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.logger.info(f"✅ ULTRA high-speed camera initialized: {width}x{height} @ {fps} FPS")
        self.logger.info(f"⚡ MAXIMUM PERFORMANCE MODE - Expanded object detection")
        
    def init_yolo_model(self):
        """Initialize YOLO model"""
        self.logger.info(f"🤖 Loading YOLO model: {self.model_name}")
        
        try:
            self.model = YOLO(self.model_name)
            self.logger.info(f"✅ YOLO model loaded successfully")
            
            # Get class names
            self.class_names = self.model.names
            self.logger.info(f"📋 Model can detect {len(self.class_names)} object classes")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to load YOLO model: {e}")
            sys.exit(1)
    
    def get_color_for_class(self, class_id):
        """Get color for specific useful object class"""
        return self.object_colors.get(class_id, (128, 128, 128))  # Gray for unknown classes
    
    def draw_detection_info(self, frame, detections):
        """Draw detection information on frame"""
        height, width = frame.shape[:2]
        
        # Draw header background
        cv2.rectangle(frame, (0, 0), (width, 80), (0, 0, 0), -1)
        
        # Title
        title = "⚡ YOLO Ultra-Fast Detection"
        cv2.putText(frame, title, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # FPS counter
        if self.show_fps:
            fps_text = f"FPS: {self.current_fps:.1f}"
            cv2.putText(frame, fps_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Detected objects count
        if self.show_count:
            object_count = getattr(self, 'detected_objects_count', 0)
            count_text = f"Objects: {object_count}"
            cv2.putText(frame, count_text, (150, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Confidence threshold
        conf_text = f"Confidence: {self.confidence_threshold:.2f}"
        cv2.putText(frame, conf_text, (300, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        # Controls help
        help_text = "Controls: Q=Quit, C=Toggle Confidence, F=Toggle FPS, S=Save Frame"
        cv2.putText(frame, help_text, (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    
    def draw_detections(self, frame, detections):
        """Draw bounding boxes and labels for useful objects"""
        if detections is None:
            return frame
        
        detected_objects_count = 0
        
        for detection in detections:
            # Extract detection information
            x1, y1, x2, y2 = map(int, detection.xyxy[0])
            confidence = float(detection.conf[0])
            class_id = int(detection.cls[0])
            
            # FILTER: Only show useful objects
            if class_id not in self.useful_classes:
                continue
                
            # Skip if confidence is below threshold
            if confidence < self.confidence_threshold:
                continue
            
            detected_objects_count += 1
            class_name = self.useful_classes[class_id]
            
            # Get color for this object class
            color = self.get_color_for_class(class_id)
            
            # Draw bounding box (optimized thickness)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Prepare label text (simplified for speed)
            if self.show_confidence:
                label = f"{class_name}: {confidence:.1f}"  # Less precision = faster
            else:
                label = class_name
            
            # Calculate label background size (smaller font for speed)
            (label_width, label_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1  # Smaller font = faster
            )
            
            # Draw label background
            cv2.rectangle(
                frame, 
                (x1, y1 - label_height - 8), 
                (x1 + label_width, y1), 
                color, 
                -1
            )
            
            # Draw label text (optimized for speed)
            cv2.putText(
                frame, 
                label, 
                (x1, y1 - 3), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5,  # Smaller font
                (255, 255, 255), 
                1     # Thinner text
            )
        
        # Store detected objects count for display
        self.detected_objects_count = detected_objects_count
        return frame
    
    def update_fps(self):
        """Update FPS counter"""
        self.fps_counter += 1
        current_time = time.time()
        
        if current_time - self.fps_time >= 1.0:
            self.current_fps = self.fps_counter / (current_time - self.fps_time)
            self.fps_counter = 0
            self.fps_time = current_time
    
    def save_frame(self, frame):
        """Save current frame with detections"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"detection_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        self.logger.info(f"💾 Frame saved as {filename}")
    
    def handle_keyboard_input(self, key, frame):
        """Handle keyboard input for controls"""
        if key == ord('q') or key == ord('Q'):
            return False  # Quit
        elif key == ord('c') or key == ord('C'):
            self.show_confidence = not self.show_confidence
            self.logger.info(f"Confidence display: {'ON' if self.show_confidence else 'OFF'}")
        elif key == ord('f') or key == ord('F'):
            self.show_fps = not self.show_fps
            self.logger.info(f"FPS display: {'ON' if self.show_fps else 'OFF'}")
        elif key == ord('s') or key == ord('S'):
            self.save_frame(frame)
        elif key == ord('+') or key == ord('='):
            self.confidence_threshold = min(1.0, self.confidence_threshold + 0.05)
            self.logger.info(f"Confidence threshold: {self.confidence_threshold:.2f}")
        elif key == ord('-') or key == ord('_'):
            self.confidence_threshold = max(0.0, self.confidence_threshold - 0.05)
            self.logger.info(f"Confidence threshold: {self.confidence_threshold:.2f}")
        
        return True  # Continue
    
    def run(self):
        """Main ULTRA-HIGH-SPEED object detection loop"""
        self.logger.info("⚡ Starting ULTRA-HIGH-SPEED object detection...")
        self.logger.info("🎯 Detecting: vehicles, people, animals, electronics, furniture, and more!")
        self.logger.info("📋 Controls:")
        self.logger.info("   Q - Quit")
        self.logger.info("   C - Toggle confidence display")
        self.logger.info("   F - Toggle FPS display")
        self.logger.info("   S - Save current frame")
        self.logger.info("   +/- - Adjust confidence threshold")
        
        try:
            while True:
                # Read frame from camera
                ret, frame = self.cap.read()
                if not ret:
                    self.logger.error("❌ Failed to read frame from camera")
                    break
                
                # ULTRA-HIGH-SPEED YOLO detection with aggressive optimizations
                results = self.model(frame, 
                                   conf=self.confidence_threshold, 
                                   verbose=False,
                                   imgsz=320,  # Even smaller input = maximum speed
                                   classes=list(self.useful_classes.keys()),  # Useful objects
                                   device='cpu',  # Use CPU for consistency
                                   half=False,  # Disable FP16 for stability on CPU
                                   max_det=50)  # Limit detections for speed
                detections = results[0].boxes if len(results) > 0 and results[0].boxes is not None else None
                
                # Draw detections on frame
                frame = self.draw_detections(frame, detections)
                
                # Draw UI information
                self.draw_detection_info(frame, detections)
                
                # Update FPS
                self.update_fps()
                
                # Display frame
                cv2.imshow('YOLO Ultra-Fast Detection', frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if not self.handle_keyboard_input(key, frame):
                    break
                    
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
        
        cv2.destroyAllWindows()
        self.logger.info("✅ Cleanup complete")

def main():
    """Main function with command line argument parsing"""
    parser = argparse.ArgumentParser(description='High-Speed Road Object Detection using YOLO')
    parser.add_argument('--model', default='yolov8n.pt', 
                       help='YOLO model to use (yolov8n.pt for speed, yolov8s.pt for accuracy)')
    parser.add_argument('--camera', type=int, default=0, 
                       help='Camera device ID (default: 0)')
    parser.add_argument('--confidence', type=float, default=0.4, 
                       help='Confidence threshold for detection (default: 0.4, higher = faster)')
    parser.add_argument('--list-cameras', action='store_true',
                       help='List available cameras and exit')
    
    args = parser.parse_args()
    
    # List cameras if requested
    if args.list_cameras:
        print("🎥 Available cameras:")
        for i in range(10):  # Check first 10 camera indices
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"  Camera {i}: Available")
                cap.release()
            else:
                break
        return
    
    # Create and run detector
    detector = YOLORealtimeDetector(
        model_name=args.model,
        camera_id=args.camera,
        confidence_threshold=args.confidence
    )
    
    detector.run()

if __name__ == "__main__":
    main()

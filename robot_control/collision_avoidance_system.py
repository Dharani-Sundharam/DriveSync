#!/usr/bin/env python3
"""
Collision Avoidance System for Robot Navigation
- Real-time object detection for safety
- Immediate stop when objects detected
- Multi-check verification before resuming
- Integrates with all robot control modes
"""

import cv2
import time
import threading
import queue
from ultralytics import YOLO
import logging
from typing import List, Dict, Optional, Tuple
from enum import Enum

class SafetyState(Enum):
    """Robot safety states"""
    SAFE = "safe"           # No objects detected - can move
    DANGER = "danger"       # Objects detected - must stop
    CHECKING = "checking"   # Verifying if path is clear
    EMERGENCY = "emergency" # Critical safety stop

class CollisionAvoidanceSystem:
    """Real-time collision avoidance using YOLO object detection"""
    
    def __init__(self, camera_id=0, confidence_threshold=0.4, model_name='yolov8n.pt'):
        """
        Initialize collision avoidance system
        
        Args:
            camera_id: Camera device ID
            confidence_threshold: Detection confidence threshold
            model_name: YOLO model to use
        """
        self.camera_id = camera_id
        self.confidence_threshold = confidence_threshold
        self.model_name = model_name
        
        # Setup logging
        self.logger = logging.getLogger('CollisionAvoidance')
        
        # Safety critical object classes that require immediate stop
        self.critical_objects = {
            # People and animals - highest priority
            0: 'person',
            16: 'bird',
            17: 'cat', 
            18: 'dog',
            
            # Vehicles - high priority
            2: 'car',
            3: 'motorcycle',
            5: 'bus',
            7: 'truck',
            1: 'bicycle',
            
            # Large obstacles
            64: 'potted plant',
            65: 'bed',
            67: 'dining table',
            
            # Road infrastructure
            11: 'stop sign',
            9: 'traffic light',
        }
        
        # Safety state management
        self.current_state = SafetyState.SAFE
        self.last_detection_time = 0
        self.clear_check_count = 0
        self.required_clear_checks = 5  # Number of consecutive clear checks needed
        self.check_interval = 0.1  # 100ms between checks
        
        # Detection tracking
        self.detection_history = []
        self.max_history = 10
        
        # Threading for real-time detection
        self.detection_thread = None
        self.detection_queue = queue.Queue(maxsize=5)
        self.running = False
        
        # Performance tracking
        self.total_detections = 0
        self.emergency_stops = 0
        self.false_alarms = 0
        
        # Initialize camera and model
        self.init_camera()
        self.init_yolo_model()
        
        # Force immediate danger state if objects detected
        self.force_immediate_stop = True
        
    def init_camera(self):
        """Initialize camera with ultra-fast settings"""
        self.logger.info(f"🎥 Initializing safety camera {self.camera_id}...")
        
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.logger.error(f"❌ Failed to open camera {self.camera_id}")
            raise RuntimeError(f"Camera {self.camera_id} not available")
        
        # Ultra-fast settings for real-time safety
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        self.logger.info(f"✅ Safety camera initialized for collision avoidance")
        
    def init_yolo_model(self):
        """Initialize YOLO model for object detection"""
        self.logger.info(f"🤖 Loading YOLO safety model: {self.model_name}")
        
        try:
            self.model = YOLO(self.model_name)
            self.logger.info(f"✅ Safety model loaded - monitoring {len(self.critical_objects)} object types")
        except Exception as e:
            self.logger.error(f"❌ Failed to load YOLO model: {e}")
            raise RuntimeError(f"YOLO model loading failed: {e}")
    
    def detect_objects(self, frame) -> List[Dict]:
        """Detect critical objects in frame"""
        try:
            # Ultra-fast detection for safety
            results = self.model(frame,
                               conf=self.confidence_threshold,
                               verbose=False,
                               imgsz=320,
                               classes=list(self.critical_objects.keys()),
                               device='cpu',
                               max_det=20)  # Limit for speed
            
            detections = results[0].boxes if len(results) > 0 and results[0].boxes is not None else None
            
            if detections is None:
                return []
            
            detected_objects = []
            for detection in detections:
                confidence = float(detection.conf[0])
                class_id = int(detection.cls[0])
                
                if class_id in self.critical_objects and confidence >= self.confidence_threshold:
                    x1, y1, x2, y2 = map(int, detection.xyxy[0])
                    
                    detected_objects.append({
                        'name': self.critical_objects[class_id],
                        'confidence': confidence,
                        'bbox': (x1, y1, x2, y2),
                        'class_id': class_id,
                        'area': (x2-x1) * (y2-y1)
                    })
            
            return detected_objects
            
        except Exception as e:
            self.logger.error(f"❌ Detection error: {e}")
            return []
    
    def update_safety_state(self, detected_objects: List[Dict]):
        """Update safety state based on detections"""
        current_time = time.time()
        
        if detected_objects:
            # Objects detected - immediate danger
            self.current_state = SafetyState.DANGER
            self.last_detection_time = current_time
            self.clear_check_count = 0
            self.total_detections += len(detected_objects)
            
            # Log critical detections
            object_names = [obj['name'] for obj in detected_objects]
            self.logger.warning(f"⚠️  OBJECTS DETECTED: {', '.join(object_names)} - STOPPING ROBOT")
            
        else:
            # No objects detected
            if self.current_state == SafetyState.DANGER:
                # Start checking if path is clear
                self.current_state = SafetyState.CHECKING
                self.clear_check_count = 1
                self.logger.info(f"🔍 Path checking... ({self.clear_check_count}/{self.required_clear_checks})")
                
            elif self.current_state == SafetyState.CHECKING:
                # Continue checking
                self.clear_check_count += 1
                self.logger.info(f"🔍 Path checking... ({self.clear_check_count}/{self.required_clear_checks})")
                
                if self.clear_check_count >= self.required_clear_checks:
                    # Path confirmed clear
                    self.current_state = SafetyState.SAFE
                    self.logger.info("✅ Path clear - robot can resume movement")
        
        # Update detection history
        self.detection_history.append({
            'time': current_time,
            'objects': detected_objects,
            'state': self.current_state
        })
        
        # Keep history manageable
        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)
    
    def detection_loop(self):
        """Main detection loop running in separate thread"""
        self.logger.info("🚀 Starting collision avoidance detection loop...")
        
        consecutive_errors = 0
        max_errors = 10
        
        while self.running:
            try:
                # Read frame
                ret, frame = self.cap.read()
                if not ret:
                    self.logger.warning("⚠️  Failed to read camera frame")
                    consecutive_errors += 1
                    if consecutive_errors >= max_errors:
                        self.logger.error("❌ Too many camera errors - stopping detection")
                        break
                    time.sleep(0.1)
                    continue
                
                consecutive_errors = 0  # Reset error counter on success
                
                # Detect objects
                detected_objects = self.detect_objects(frame)
                
                # Update safety state
                self.update_safety_state(detected_objects)
                
                # Put result in queue for main thread (non-blocking)
                try:
                    self.detection_queue.put({
                        'objects': detected_objects,
                        'state': self.current_state,
                        'timestamp': time.time()
                    }, block=False)
                except queue.Full:
                    # Queue full, remove old item and add new one
                    try:
                        self.detection_queue.get_nowait()  # Remove old
                        self.detection_queue.put({
                            'objects': detected_objects,
                            'state': self.current_state,
                            'timestamp': time.time()
                        }, block=False)
                    except queue.Empty:
                        pass
                
                # Control detection rate
                time.sleep(self.check_interval)
                
            except Exception as e:
                consecutive_errors += 1
                self.logger.error(f"❌ Detection loop error: {e}")
                if consecutive_errors >= max_errors:
                    self.logger.error("❌ Too many consecutive errors - stopping detection")
                    break
                time.sleep(0.5)
    
    def start(self):
        """Start the collision avoidance system"""
        if self.running:
            self.logger.warning("⚠️  Collision avoidance already running")
            return
        
        self.running = True
        self.detection_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detection_thread.start()
        
        self.logger.info("✅ Collision avoidance system started")
    
    def stop(self):
        """Stop the collision avoidance system"""
        self.running = False
        
        if self.detection_thread and self.detection_thread.is_alive():
            self.detection_thread.join(timeout=2.0)
        
        if hasattr(self, 'cap'):
            self.cap.release()
        
        self.logger.info("🛑 Collision avoidance system stopped")
    
    def is_safe_to_move(self) -> bool:
        """Check if it's safe for robot to move"""
        return self.current_state == SafetyState.SAFE
    
    def get_latest_detection(self) -> Optional[Dict]:
        """Get latest detection result"""
        try:
            return self.detection_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_safety_status(self) -> Dict:
        """Get comprehensive safety status"""
        return {
            'state': self.current_state.value,
            'safe_to_move': self.is_safe_to_move(),
            'last_detection_time': self.last_detection_time,
            'clear_check_count': self.clear_check_count,
            'required_checks': self.required_clear_checks,
            'total_detections': self.total_detections,
            'emergency_stops': self.emergency_stops,
            'detection_history_length': len(self.detection_history)
        }
    
    def emergency_stop(self):
        """Trigger emergency stop state"""
        self.current_state = SafetyState.EMERGENCY
        self.emergency_stops += 1
        self.logger.critical("🚨 EMERGENCY STOP ACTIVATED")
    
    def reset_safety_state(self):
        """Reset to safe state (use carefully!)"""
        self.current_state = SafetyState.SAFE
        self.clear_check_count = 0
        self.logger.warning("⚠️  Safety state manually reset to SAFE")

# Test the collision avoidance system
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Test Collision Avoidance System')
    parser.add_argument('--camera', type=int, default=0, help='Camera ID')
    parser.add_argument('--confidence', type=float, default=0.4, help='Detection confidence')
    parser.add_argument('--duration', type=int, default=30, help='Test duration in seconds')
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    
    # Create and test collision avoidance system
    cas = CollisionAvoidanceSystem(camera_id=args.camera, confidence_threshold=args.confidence)
    
    try:
        cas.start()
        
        print("🚀 Testing collision avoidance system...")
        print(f"⏱️  Test duration: {args.duration} seconds")
        print("🎮 Press Ctrl+C to stop early")
        
        start_time = time.time()
        while time.time() - start_time < args.duration:
            status = cas.get_safety_status()
            latest = cas.get_latest_detection()
            
            if latest:
                objects = latest.get('objects', [])
                state = latest.get('state', 'unknown')
                
                # Handle SafetyState enum
                if hasattr(state, 'value'):
                    state_str = state.value.upper()
                else:
                    state_str = str(state).upper()
                
                print(f"🛡️  Safety: {state_str} | Objects: {len(objects)} | Safe to move: {status['safe_to_move']}")
                
                if objects:
                    for obj in objects:
                        print(f"   🎯 {obj['name']}: {obj['confidence']:.2f}")
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    finally:
        cas.stop()
        print("✅ Test complete")

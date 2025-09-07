# 🤖 YOLO Real-time Object Detection Guide

## 📋 Overview

This script provides real-time object detection using YOLO (You Only Look Once) model with live camera feed. It can detect and identify various objects in real-time with bounding boxes, confidence scores, and labels.

## 🚀 Features

- **Real-time Detection**: Live camera feed with instant object detection
- **Multiple Object Classes**: Detects 80+ different object classes (people, vehicles, animals, objects, etc.)
- **Visual Feedback**: Bounding boxes, labels, and confidence scores
- **Performance Monitoring**: Real-time FPS counter and object count
- **Interactive Controls**: Keyboard shortcuts for various settings
- **Frame Saving**: Save detection results as images
- **Adjustable Settings**: Confidence threshold, display options

## 🎯 Detected Object Classes

The YOLO model can detect 80+ object classes including:
- **People**: person
- **Vehicles**: car, motorcycle, airplane, bus, train, truck, boat
- **Animals**: bird, cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe
- **Objects**: bicycle, traffic light, fire hydrant, stop sign, parking meter, bench
- **Sports**: baseball bat, baseball glove, skateboard, surfboard, tennis racket
- **Electronics**: laptop, mouse, remote, keyboard, cell phone, microwave, oven, toaster
- **Food**: banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake
- **Furniture**: chair, couch, potted plant, bed, dining table, toilet
- **And many more...

## 💻 Installation

### Prerequisites
Make sure you have Python 3.7+ installed.

### Install Dependencies
```bash
cd /home/dharani/Desktop/DriveSync/robot_control
pip3 install -r requirements.txt
```

This will install:
- `opencv-python` - Computer vision library
- `ultralytics` - YOLO implementation
- `numpy` - Numerical computing
- `pillow` - Image processing

## 🎮 Usage

### Basic Usage
```bash
# Run with default settings (camera 0, YOLOv8n model)
python3 yolo_realtime_detection.py
```

### Advanced Usage
```bash
# Use different camera
python3 yolo_realtime_detection.py --camera 1

# Use different YOLO model (more accurate but slower)
python3 yolo_realtime_detection.py --model yolov8s.pt

# Set custom confidence threshold
python3 yolo_realtime_detection.py --confidence 0.5

# List available cameras
python3 yolo_realtime_detection.py --list-cameras
```

### Command Line Options
- `--model`: YOLO model to use
  - `yolov8n.pt` - Nano (fastest, least accurate)
  - `yolov8s.pt` - Small
  - `yolov8m.pt` - Medium
  - `yolov8l.pt` - Large
  - `yolov8x.pt` - Extra Large (slowest, most accurate)
- `--camera`: Camera device ID (0, 1, 2, etc.)
- `--confidence`: Minimum confidence threshold (0.0 to 1.0)
- `--list-cameras`: Show available cameras

## 🎮 Interactive Controls

While the detection is running, use these keyboard shortcuts:

| Key | Action |
|-----|--------|
| `Q` | Quit the application |
| `C` | Toggle confidence score display |
| `F` | Toggle FPS counter display |
| `S` | Save current frame as image |
| `+` or `=` | Increase confidence threshold |
| `-` or `_` | Decrease confidence threshold |

## 📊 Display Information

The interface shows:
- **Title**: Application name
- **FPS**: Current frames per second
- **Objects**: Number of detected objects
- **Confidence**: Current confidence threshold
- **Bounding Boxes**: Colored rectangles around detected objects
- **Labels**: Object name and confidence score
- **Controls**: Help text at the bottom

## 🎨 Visual Features

- **Color-coded Detection**: Different colors for different object classes
- **Confidence Scores**: Shows detection certainty (0.0 to 1.0)
- **Real-time Stats**: Live FPS and object count
- **Clean Interface**: Non-intrusive overlay information

## 🔧 Troubleshooting

### Camera Issues
```bash
# Check available cameras
python3 yolo_realtime_detection.py --list-cameras

# Try different camera ID
python3 yolo_realtime_detection.py --camera 1
```

### Performance Issues
```bash
# Use faster model
python3 yolo_realtime_detection.py --model yolov8n.pt

# Lower confidence threshold for fewer detections
python3 yolo_realtime_detection.py --confidence 0.5
```

### Model Download
The first time you run the script, it will automatically download the YOLO model (~6MB for yolov8n.pt). This requires internet connection.

## 📁 Output Files

When you press `S` to save frames, they are saved as:
- `detection_YYYYMMDD_HHMMSS.jpg`
- Located in the same directory as the script

## 🚀 Example Commands

```bash
# Quick start with default settings
python3 yolo_realtime_detection.py

# High accuracy mode
python3 yolo_realtime_detection.py --model yolov8x.pt --confidence 0.3

# Fast mode for real-time applications
python3 yolo_realtime_detection.py --model yolov8n.pt --confidence 0.6

# USB webcam
python3 yolo_realtime_detection.py --camera 1

# Check what cameras are available
python3 yolo_realtime_detection.py --list-cameras
```

## 🎯 Tips for Best Results

1. **Good Lighting**: Ensure adequate lighting for better detection
2. **Stable Camera**: Reduce camera shake for clearer results
3. **Appropriate Distance**: Objects should be clearly visible, not too small or too large
4. **Model Selection**: Balance between speed and accuracy based on your needs
5. **Confidence Tuning**: Adjust threshold to reduce false positives/negatives

## 🔍 Technical Details

- **Framework**: YOLOv8 from Ultralytics
- **Input Resolution**: 640x480 (adjustable)
- **Detection Classes**: 80 COCO dataset classes
- **Real-time Performance**: 15-30 FPS depending on hardware
- **Memory Usage**: ~1-2GB depending on model size

Enjoy real-time object detection! 🎉

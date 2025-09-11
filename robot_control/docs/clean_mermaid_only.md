# Clean Mermaid Diagrams - Copy Only the Code Blocks

## 1. Simple System Architecture (Recommended)

```mermaid
graph TB
    subgraph UI ["🖥️ USER INTERFACE"]
        GUI[Interactive GUI<br/>Auto-Resolution Display]
        LIDAR_VIZ[LIDAR Visualization<br/>480x320 Cartesian Plot]
    end

    subgraph CTRL ["🎛️ CONTROL LAYER"]
        MAIN[PathfindingRobotController<br/>Main Orchestrator]
        SAFETY[Safety Manager<br/>Multi-Sensor Fusion]
    end

    subgraph ALGO ["🧠 ALGORITHMS"]
        PATHFIND[A* Pathfinding<br/>Route Planning]
        NAV[PID Navigation<br/>Path Following]
        LIDAR_MAP[LIDAR Mapping<br/>Real-time Occupancy]
        VISION[YOLO Detection<br/>Object Recognition]
    end

    subgraph HW ["⚙️ HARDWARE"]
        RPI[Raspberry Pi 4<br/>Main Computer]
        LIDAR[YDLIDAR X2<br/>360° Laser Scanner]
        CAMERA[USB Camera<br/>Computer Vision]
        ARDUINO[Arduino Uno<br/>Motor Controller]
        ROBOT[Differential Drive<br/>210mm Chassis]
    end

    UI --> CTRL
    CTRL --> ALGO
    ALGO --> HW

    MAIN --> SAFETY
    PATHFIND --> NAV
    LIDAR_MAP --> PATHFIND
    VISION --> SAFETY
    
    RPI --> ARDUINO
    RPI --> LIDAR
    RPI --> CAMERA
    ARDUINO --> ROBOT

    LIDAR -.->|Scan Data| LIDAR_MAP
    CAMERA -.->|Video Stream| VISION
    ROBOT -.->|Encoder Data| NAV
    NAV -.->|Motor Commands| ARDUINO

    classDef uiStyle fill:#e3f2fd,stroke:#1976d2,stroke-width:2px,color:#000
    classDef ctrlStyle fill:#e8f5e8,stroke:#388e3c,stroke-width:2px,color:#000
    classDef algoStyle fill:#fff3e0,stroke:#f57c00,stroke-width:2px,color:#000
    classDef hwStyle fill:#fce4ec,stroke:#c2185b,stroke-width:2px,color:#000

    class GUI,LIDAR_VIZ uiStyle
    class MAIN,SAFETY ctrlStyle
    class PATHFIND,NAV,LIDAR_MAP,VISION algoStyle
    class RPI,LIDAR,CAMERA,ARDUINO,ROBOT hwStyle
```

## 2. Data Flow Diagram

```mermaid
flowchart TD
    USER_INPUT[👤 User Input<br/>Mouse & Keyboard]
    LIDAR_SENSOR[📡 LIDAR Sensor<br/>360° Scans]
    CAMERA_SENSOR[📷 Camera Sensor<br/>Video Stream]
    ENCODER_FB[⚙️ Encoder Feedback<br/>Position Data]

    subgraph FUSION ["🔄 DATA FUSION"]
        EVENT_PROC[Event Processing]
        LIDAR_PROC[LIDAR Processing]
        VISION_PROC[Vision Processing]
        ODOM_PROC[Odometry Processing]
    end

    subgraph DECISION ["🧠 DECISION MAKING"]
        PATH_PLAN[Path Planning<br/>A* Algorithm]
        NAV_CONTROL[Navigation Control<br/>PID Controller]
        SAFETY_CHECK[Safety Assessment<br/>Collision Detection]
        MODE_MGR[Mode Management]
    end

    subgraph OUTPUT ["📤 CONTROL OUTPUT"]
        MOTOR_CMD[Motor Commands]
        DISPLAY_UPD[Display Updates]
        STATUS_IND[Status Indicators]
        DATA_LOG[Data Logging]
    end

    MOTORS[🔧 Motor Hardware]
    DISPLAY[🖥️ Display Hardware]
    STORAGE[💾 File System]

    USER_INPUT --> EVENT_PROC
    LIDAR_SENSOR --> LIDAR_PROC
    CAMERA_SENSOR --> VISION_PROC
    ENCODER_FB --> ODOM_PROC

    EVENT_PROC --> PATH_PLAN
    LIDAR_PROC --> SAFETY_CHECK
    LIDAR_PROC --> PATH_PLAN
    VISION_PROC --> SAFETY_CHECK
    ODOM_PROC --> NAV_CONTROL

    PATH_PLAN --> NAV_CONTROL
    NAV_CONTROL --> MOTOR_CMD
    SAFETY_CHECK --> MODE_MGR
    MODE_MGR --> MOTOR_CMD
    MODE_MGR --> DISPLAY_UPD

    MOTOR_CMD --> MOTORS
    DISPLAY_UPD --> DISPLAY
    STATUS_IND --> DISPLAY
    DATA_LOG --> STORAGE

    MOTORS --> ENCODER_FB

    classDef inputStyle fill:#e0f2fe,stroke:#0277bd,stroke-width:2px
    classDef processStyle fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    classDef decisionStyle fill:#fff3e0,stroke:#ef6c00,stroke-width:2px
    classDef outputStyle fill:#e8f5e8,stroke:#2e7d32,stroke-width:2px
    classDef hardwareStyle fill:#ffebee,stroke:#c62828,stroke-width:2px

    class USER_INPUT,LIDAR_SENSOR,CAMERA_SENSOR,ENCODER_FB inputStyle
    class EVENT_PROC,LIDAR_PROC,VISION_PROC,ODOM_PROC processStyle
    class PATH_PLAN,NAV_CONTROL,SAFETY_CHECK,MODE_MGR decisionStyle
    class MOTOR_CMD,DISPLAY_UPD,STATUS_IND,DATA_LOG outputStyle
    class MOTORS,DISPLAY,STORAGE hardwareStyle
```

## 3. Control Loop (Simplified)

```mermaid
flowchart LR
    subgraph LOOP ["🔄 MAIN CONTROL LOOP (20 Hz)"]
        COLLECT[📊 Data Collection<br/>LIDAR, Camera, Encoders]
        PROCESS[⚡ Processing<br/>Mapping & Detection]
        DECIDE[🎯 Decision Making<br/>Path Planning & Safety]
        EXECUTE[🚀 Execution<br/>Motor Control & Display]
    end

    SENSORS[🔍 Sensors]
    ACTUATORS[⚙️ Actuators]

    COLLECT --> PROCESS
    PROCESS --> DECIDE
    DECIDE --> EXECUTE
    EXECUTE --> COLLECT

    SENSORS --> COLLECT
    EXECUTE --> ACTUATORS
    ACTUATORS -.->|Feedback| COLLECT

    classDef loopStyle fill:#e3f2fd,stroke:#1976d2,stroke-width:3px,color:#000
    classDef externalStyle fill:#f1f8e9,stroke:#388e3c,stroke-width:2px,color:#000

    class COLLECT,PROCESS,DECIDE,EXECUTE loopStyle
    class SENSORS,ACTUATORS externalStyle
```

## Instructions:

1. **Copy ONLY the code between** ```mermaid and ```
2. **Don't include** the markdown headers (# titles)
3. **Don't include** the ```mermaid and ``` markers
4. **Paste directly** into Mermaid Live Editor
5. **Use Diagram #1** for your presentation - it's the clearest and most comprehensive

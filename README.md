# Drift Hackathon - GalaxyRVR Autonomous Navigation

## 🎯 Project Overview

This project implements an autonomous navigation system for the GalaxyRVR robot using computer vision and sensor fusion. The system combines overhead camera vision for global path planning with onboard sensors for local obstacle avoidance.

## 🤖 System Architecture

### Hardware Components

- **GalaxyRVR Robot**
  - Differential drive motors (left/right)
  - ESP32-CAM for onboard vision
  - Ultrasonic distance sensor (forward)
  - IR obstacle sensors (left/right)
  - Servo-controlled camera pan
  - Battery voltage monitoring

- **Overhead Vision System**
  - USB webcam positioned above arena
  - Real-time perspective transformation
  - Red corner marker detection for calibration

### Software Components

```
drift-hackathon/
├── robot_code/
│   ├── arduino/              # Robot firmware
│   └── python_client/        # Robot control library
├── webcam_code/              # Overhead camera system
│   ├── arena_transform.py   # Perspective transformation
│   ├── webcam_stream.py     # Camera streaming server
│   └── webcam_client.py     # Camera client
└── autonomous_navigation/    # Main navigation system (to be implemented)
```

## 📋 Challenge Analysis

Based on the codebase analysis, the challenge likely involves:

1. **Autonomous Navigation**: Robot must navigate to targets/waypoints in an arena
2. **Computer Vision**: Detect and track objects/targets using vision
3. **Obstacle Avoidance**: Navigate around obstacles using sensors
4. **Precision Control**: Accurate positioning and path following

## 🚀 Implementation Plan

### Phase 1: System Integration & Calibration ✅

- [x] Repository setup and initial code commit
- [ ] Arena setup and red corner marker verification
- [ ] World coordinate system calibration
- [ ] Perspective transformation accuracy testing
- [ ] Robot connection and sensor validation

### Phase 2: Core Navigation System 🚧

#### 2.1 Target Detection Module
- [ ] Implement color-based target detection (HSV)
- [ ] Shape detection using contours
- [ ] Multiple target tracking
- [ ] Coordinate transformation (pixel → world coordinates)

#### 2.2 Robot Localization
- [ ] Robot position tracking in overhead view
- [ ] Marker-based or tracking-based localization
- [ ] World coordinate mapping
- [ ] Position update loop

#### 2.3 Path Planning
- [ ] Simple straight-line navigation
- [ ] Obstacle-aware path planning
- [ ] Waypoint following
- [ ] Dynamic path adjustment

#### 2.4 Control System
- [ ] PID controller for heading
- [ ] Speed control based on distance
- [ ] Sensor fusion (ultrasonic + IR + vision)
- [ ] State machine for navigation modes

### Phase 3: Advanced Features 🎯

- [ ] Hybrid vision system (overhead + onboard)
- [ ] Obstacle avoidance integration
- [ ] Error recovery mechanisms
- [ ] Performance optimization
- [ ] Real-time visualization

### Phase 4: Testing & Refinement 🧪

- [ ] Unit testing for modules
- [ ] Integration testing
- [ ] Arena testing and calibration
- [ ] Performance tuning
- [ ] Documentation

## 🏗️ Technical Approach

### Strategy: Hybrid Vision System (Recommended)

**Primary: Overhead Vision**
- Global view of entire arena
- Easier target detection
- Better path planning
- Accurate robot positioning

**Secondary: Onboard Vision**
- Local navigation refinement
- Target verification
- Obstacle detection

**Tertiary: Sensor Fusion**
- Ultrasonic for forward obstacles
- IR sensors for side obstacles
- Vision for target tracking

### Navigation Flow

```
┌─────────────────┐
│  Overhead Camera│
│  (Global View)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Target Detection│
│ & Localization  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Path Planning │
┌──────────────┐
└────────┬────────┘ │ Robot Camera │
         │          │ (Local View) │
         ▼          └───────┬───────┘
┌─────────────────┐        │
│  Robot Control  │◄───────┘
│  (Motor Commands)│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Sensor Feedback │
│ (Ultrasonic/IR) │
└────────┬────────┘
         │
         └─────────┐
                   ▼
         ┌─────────────────┐
         │ Adjust Path     │
         └─────────────────┘
```

### State Machine

```
IDLE
  │
  ▼
SEARCHING (detect targets)
  │
  ▼
TRACKING (lock onto target)
  │
  ▼
NAVIGATING (move toward target)
  │
  ├─► OBSTACLE_DETECTED ──► AVOIDING ──► NAVIGATING
  │
  ▼
ARRIVED (at target)
  │
  ▼
RETURNING (optional: return to start)
```

## 🛠️ Setup Instructions

### Prerequisites

```bash
# Python dependencies
pip install -r robot_code/python_client/requirements.txt
pip install -r webcam_code/requirements.txt

# Additional dependencies for navigation
pip install scipy matplotlib
```

### Robot Configuration

1. **Arduino Setup**
   - Upload `galaxy-rvr.ino` to Arduino
   - Configure WiFi credentials in the code
   - Verify WebSocket connection on port 8765

2. **Network Configuration**
   - Ensure robot and computer are on same network
   - Note robot IP address (default: 192.168.1.216)
   - Test connection with `simple_control.py`

### Arena Setup

1. **Physical Setup**
   - Place red corner markers at arena corners
   - Position overhead webcam above arena
   - Ensure good lighting conditions

2. **Calibration**
   ```bash
   # Run arena transformation calibration
   cd webcam_code
   python arena_transform.py
   ```

3. **Start Camera Stream**
   ```bash
   # Start webcam server with transformation
   python webcam_stream.py --transform --camera 0
   ```

## 📁 Project Structure

```
drift-hackathon/
├── README.md                          # This file
├── robot_code/
│   ├── arduino/
│   │   └── galaxy-rvr/                # Arduino firmware
│   └── python_client/
│       ├── galaxyrvr.py              # Robot control library
│       ├── galaxyrvr_camera.py       # Camera stream helper
│       ├── galaxyrvr_keyboard.py     # Keyboard control
│       ├── simple_control.py         # Simple control example
│       └── requirements.txt
├── webcam_code/
│   ├── arena_transform.py            # Perspective transformation
│   ├── webcam_stream.py              # Camera streaming server
│   ├── webcam_client.py              # Camera client
│   └── requirements.txt
└── autonomous_navigation/            # Main navigation system (TODO)
    ├── target_detection.py           # Target detection module
    ├── robot_localization.py         # Position tracking
    ├── path_planner.py               # Path planning
    ├── navigation_controller.py      # Control system
    └── main.py                       # Main entry point
```

## 🔧 Key Modules to Implement

### 1. Target Detection (`target_detection.py`)
- Color-based detection using HSV
- Contour analysis for shape detection
- Multiple target tracking
- Coordinate conversion utilities

### 2. Robot Localization (`robot_localization.py`)
- Marker-based position detection
- Tracking-based localization
- World coordinate transformation
- Position filtering and smoothing

### 3. Path Planner (`path_planner.py`)
- Straight-line path calculation
- Obstacle-aware path planning
- Waypoint generation
- Dynamic path adjustment

### 4. Navigation Controller (`navigation_controller.py`)
- PID control for heading
- Speed control
- Sensor fusion
- State machine management

### 5. Main Navigation Loop (`main.py`)
- Integration of all modules
- Real-time control loop
- Error handling and recovery
- Visualization and logging

## 🎮 Usage

### Basic Robot Control
```bash
cd robot_code/python_client
python simple_control.py
```

### Camera Streaming
```bash
cd webcam_code
python webcam_stream.py --transform
```

### Autonomous Navigation (Once Implemented)
```bash
cd autonomous_navigation
python main.py --robot-ip 192.168.1.216 --camera-url http://localhost:8000/
```

## 📊 Performance Metrics

- **Target Detection Accuracy**: >95%
- **Position Accuracy**: <5cm error
- **Navigation Success Rate**: >90%
- **Obstacle Avoidance**: 100% (no collisions)
- **Response Time**: <100ms control loop

## 🐛 Troubleshooting

### Robot Connection Issues
- Verify WiFi credentials in Arduino code
- Check robot IP address
- Ensure WebSocket port 8765 is accessible

### Camera Issues
- Verify camera index (try 0, 1, 2...)
- Check red corner marker visibility
- Adjust HSV thresholds for lighting

### Navigation Issues
- Recalibrate arena transformation
- Verify world coordinates
- Check sensor readings

## 📝 Notes

- The arena uses red corner markers for calibration
- World coordinates are defined in `arena_transform.py`
- Robot communicates via WebSocket on port 8765
- Camera streams on port 9000 (robot) and 8000 (webcam)

## 🤝 Contributing

This is a hackathon project. Key areas for contribution:
- Target detection algorithms
- Path planning improvements
- Sensor fusion techniques
- Performance optimization

## 📄 License

Initial hackathon code provided by Drift.

---

**Status**: 🚧 In Development
**Last Updated**: Initial commit
**Next Steps**: Implement Phase 2 modules


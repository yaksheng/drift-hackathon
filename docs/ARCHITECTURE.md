# System Architecture

## 🤖 Hardware Components

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

## 📁 Project Structure

```
drift-hackathon/
├── scripts/                  # Main executable scripts
│   ├── edge_follow_to_strip.py  # Edge following and strip selection
│   ├── run_line_following.py    # Line following navigation runner
│   ├── simulate.py              # Main simulation entry point
│   ├── simulate_edge_follow.py  # Edge following simulation
│   └── plan_z.py                # Robot movement planning script
├── puzzle/                   # Puzzle solving tools
│   ├── puzzlesolver.py          # AI puzzle solver using OpenAI Vision
│   ├── test.py                  # Image download utility
│   ├── test_puzzle_simple.py    # Simple puzzle solver test
│   └── quiz/                    # Quiz images
├── tests/                     # Test runners
│   └── run_virtual_tests.py     # Virtual test runner
├── robot_code/
│   ├── arduino/              # Robot firmware
│   └── python_client/        # Robot control library
├── webcam_code/              # Overhead camera system
│   ├── arena_transform.py   # Perspective transformation
│   ├── webcam_stream.py     # Camera streaming server
│   └── webcam_client.py     # Camera client
└── autonomous_navigation/    # Main navigation system ✅
    ├── __init__.py           # Package initialization
    ├── target_detection.py   # Target detection module
    ├── robot_localization.py # Position tracking
    ├── path_planner.py       # Path planning
    ├── navigation_controller.py # Control system
    ├── dead_reckoning.py     # Odometry for camera delay handling
    ├── path_visualization.py # Path overlay on camera feed
    ├── line_detection.py     # Line detection module
    ├── obstacle_detection.py # Obstacle detection from camera feed
    ├── line_following_navigation.py # Line following navigation system
    ├── simulate_line_following.py # Simulation with real camera feed
    ├── path_visualizer_camera.py # Path visualization on camera feed
    ├── main.py              # Main entry point
    ├── example_usage.py     # Usage examples
    └── requirements.txt     # Dependencies
```

## 🔄 Module Integration

The modules work together in a coordinated pipeline:

1. **Overhead Camera** → Provides global view of arena
2. **Target Detection** → Identifies targets and converts to world coordinates
3. **Robot Localization** → Tracks robot position in world coordinates
4. **Dead Reckoning** → Estimates position during camera delays using odometry
5. **Path Planner** → Generates waypoints from robot to target (avoiding obstacles)
6. **Navigation Controller** → Converts waypoints to motor commands using PID control
7. **Path Visualization** → Overlays path and navigation info on camera feed
8. **Robot** → Executes commands and provides sensor feedback
9. **Loop** → Repeats at ~10Hz for real-time navigation

**Data Flow**:
```
Camera Frame → [Target Detection + Localization] → [Dead Reckoning] → 
[Path Planning] → [Navigation Controller] → Motor Commands → Robot → 
[Dead Reckoning Update] → Sensor Feedback → [Path Visualization] → Loop
```

**Dead Reckoning Integration**:
- Camera updates provide ground truth position (when available)
- Motor commands continuously update dead reckoning estimate
- Navigation uses dead reckoning position when camera is delayed (>2s)
- Automatic correction when camera updates arrive


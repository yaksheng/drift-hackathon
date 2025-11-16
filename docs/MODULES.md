# Module Documentation

## 🔧 Phase 1: Calibration Tools Implementation

### Calibration Tools (`calibration_tools.py`) ✅

**Purpose**: Comprehensive tools for arena setup, calibration, and system validation.

**Implementation Details**:

- **ArenaCalibrator Class**: Main calibration tool with methods for:
  - `verify_red_corners()`: Detects and verifies 4 red corner markers
    - Uses HSV color detection (same as `arena_transform.py`)
    - Validates exactly 4 corners are detected
    - Creates visualization images showing detected corners
    - Provides troubleshooting guidance if detection fails
  
  - `calibrate_world_coordinates()`: Calibrates world coordinate system
    - Takes pixel coordinates (from corner detection) and world coordinates (measured)
    - Calculates two transformation matrices:
      * `auto_transform_matrix.npy`: Pixel to normalized coordinates
      * `world_transform_matrix.npy`: Pixel to world coordinates (meters)
    - Uses `cv2.getPerspectiveTransform()` and `cv2.findHomography()`
    - Saves matrices for use by navigation system
    - Tests transformation with center point
  
  - `test_perspective_transform()`: Tests transformation accuracy
    - Loads saved transformation matrix
    - Applies transformation to test image
    - Analyzes corner angles (should be ~90° for rectangle)
    - Calculates angle error: `|average_angle - 90°|`
    - Provides quality assessment (Excellent/Good/Poor)

- **RobotValidator Class**: Robot connection and sensor validation
  - `validate_connection()`: Tests WebSocket connection
    - Connects to robot via `GalaxyRVR` class
    - Tests communication by sending stop command
    - Provides troubleshooting for connection failures
  
  - `validate_sensors()`: Tests all robot sensors
    - Reads sensors for specified duration (default 5 seconds)
    - Validates:
      * Battery voltage (should be >6V)
      * Ultrasonic sensor (distance readings in cm)
      * IR left sensor (0 or 1)
      * IR right sensor (0 or 1)
    - Displays statistics and ranges
    - Provides real-time feedback during test

- **Command-Line Interface**: Supports 6 commands:
  1. `verify-corners`: Verify red corner detection
  2. `calibrate-world`: Calibrate world coordinates
  3. `test-transform`: Test transformation accuracy
  4. `validate-robot`: Validate robot connection
  5. `validate-sensors`: Validate robot sensors
  6. `full-calibration`: Run all steps automatically

**Usage Examples**:
```bash
# Verify corners
python3 calibrate.py verify-corners --image arena_image.png

# Calibrate with custom coordinates
python3 calibrate.py calibrate-world --image arena_image.png \
    --world-corners "0,3.85 2.35,3.95 1.7,0.05 0.45,0"

# Full calibration
python3 calibrate.py full-calibration --image arena_image.png
```

---

### Image Capture Tool (`capture_calibration_image.py`) ✅

**Purpose**: Capture images from overhead camera for calibration.

**Implementation Details**:

- **URL Capture**: Captures from MJPEG stream
  - Connects to camera stream URL (e.g., `http://192.168.0.21:8000/`)
  - Parses MJPEG stream (finds JPEG boundaries: `\xff\xd8` to `\xff\xd9`)
  - Decodes first complete frame
  - Saves with timestamped filename

- **Webcam Capture**: Captures from local webcam
  - Uses `cv2.VideoCapture()` with camera index
  - Captures single frame
  - Saves with timestamped filename

- **Automatic Filenames**: Uses timestamp format: `calibration_image_YYYYMMDD_HHMMSS.png`

**Usage**:
```bash
# From camera URL
python3 autonomous_navigation/capture_calibration_image.py --url http://192.168.0.21:8000/

# From local webcam
python3 autonomous_navigation/capture_calibration_image.py --camera 0
```

---

## 🔧 Key Modules Implementation

### 1. Target Detection (`target_detection.py`) ✅

**Purpose**: Detect and track targets/objects in the arena using computer vision.

**Implementation Details**:

- **Color-Based Detection**: Uses HSV color space for robust color detection under varying lighting conditions. Supports multiple colors (blue, green, yellow, red, orange) with configurable HSV ranges. Red color detection handles the HSV wraparound issue by using two separate ranges (0-10° and 170-180°).

- **Shape Detection**: Implements contour analysis to detect geometric shapes (circles, squares, triangles). Uses `cv2.approxPolyDP()` for polygon approximation and calculates circularity metric: `4π × area / perimeter²` to identify circular targets.

- **Multi-Target Tracking**: Implements a simple distance-based tracking algorithm. Each detected target is assigned a unique ID. When new targets are detected, they are matched to existing tracked targets based on minimum Euclidean distance (within a threshold). Unmatched targets receive new IDs, and unmatched old targets are removed after one frame.

- **Coordinate Transformation**: Converts pixel coordinates to world coordinates using a 3×3 homography transformation matrix. The `convert_to_world_coords()` method applies the transformation: `world_point = transform_matrix @ [px, py, 1]`.

- **Confidence Scoring**: Calculates detection confidence based on:
  - Circularity (for shape-based detection)
  - Area ratio (target size relative to expected size)
  - Contour quality

- **Visualization**: Provides `visualize_targets()` method that draws circles, center points, and labels on detected targets for debugging.

**Key Classes**:
- `Target`: Dataclass representing a detected target with center, world position, area, color, confidence, and tracking ID.
- `TargetDetector`: Main class with methods for detection, tracking, and coordinate transformation.

---

### 2. Robot Localization (`robot_localization.py`) ✅

**Purpose**: Track robot position in the arena using overhead camera and coordinate transformation.

**Implementation Details**:

- **Marker-Based Detection**: Detects robot using a colored marker (configurable: blue, green, or red). Uses HSV color space with morphological operations (closing and opening) to clean up the detection mask. Filters detections by area to remove noise and false positives.

- **Position Smoothing**: Implements exponential smoothing to reduce position jitter:
  ```
  smoothed_pos = α × new_pos + (1-α) × last_pos
  ```
  Where α = 0.7 (configurable). Also maintains a position history buffer (last 10 positions) for additional smoothing if needed.

- **World Coordinate Transformation**: Uses the same homography matrix as target detection to convert pixel coordinates to real-world meters. The transformation matrix is loaded from `auto_transform_matrix.npy` (generated by `arena_transform.py`).

- **Orientation Estimation**: Placeholder implementation that returns 0 (facing positive X). Can be enhanced with:
  - Directional marker detection (front/back markers)
  - Optical flow analysis
  - IMU integration (if available)

- **Confidence Management**: Tracks localization confidence:
  - High confidence (0.9) when robot is detected
  - Low confidence (0.3) when using last known position (robot not visible)
  - Returns None if robot hasn't been seen recently

- **Robustness**: Handles cases where robot is temporarily occluded by maintaining `last_pose` and using it with reduced confidence.

**Key Classes**:
- `RobotPose`: Dataclass with x, y, theta (orientation), and confidence.
- `RobotLocalizer`: Main class with detection, localization, and smoothing methods.

---

### 3. Path Planner (`path_planner.py`) ✅

**Purpose**: Plan paths from current position to target, with obstacle avoidance.

**Implementation Details**:

- **Straight-Line Planning**: Simple direct path from start to goal. Used when no obstacles are detected.

- **Obstacle-Aware Planning**: When obstacles block the path:
  1. Identifies blocking obstacles using `get_blocking_obstacles()`
  2. Calculates perpendicular offset to go around the first blocking obstacle
  3. Generates waypoints: [waypoint_around_obstacle, goal]
  4. Validates waypoints are within arena bounds

- **Obstacle Detection**: Uses point-to-line-segment distance calculation to check if obstacles intersect the path. Distance formula:
  ```
  distance = |(point - line_start) × (line_end - line_start)| / |line_end - line_start|
  ```
  Obstacle blocks path if: `distance < (obstacle_radius + clearance)`

- **Waypoint Management**: 
  - Tracks which waypoints have been reached
  - Provides `get_next_waypoint()` to get next unreached waypoint
  - Supports dynamic replanning with `replan_path()`

- **Path Validation**: Checks if paths are clear using `is_path_clear()`, which tests all obstacles against the line segment.

- **Arena Bounds**: Optional arena boundary constraints to keep waypoints within valid area.

- **Pre-Mapped Obstacles**: Obstacles are pre-mapped (known positions) but can have variable properties:
  - **Known Positions**: Obstacle locations are pre-mapped from arena setup
  - **Variable Sizes**: Obstacle radii can vary (0.10m - 0.25m)
  - **Variable Colors**: Obstacles can have different colors (red, blue, green, yellow, orange) for identification
  - **Confidence Levels**: High-confidence obstacles (pre-mapped) vs. low-confidence sensor-detected obstacles

**Key Classes**:
- `Waypoint`: Dataclass with x, y coordinates and reached flag.
- `Obstacle`: Dataclass with position, radius, confidence, and color.
- `PathPlanner`: Main class with planning, validation, and waypoint management.

**Pre-Mapped Obstacle System** (`obstacle_map.py`):
- `ObstacleMap`: Manages pre-mapped obstacles with known positions
- Default map includes 3 obstacles at typical arena positions
- Supports loading/saving obstacle configurations from JSON files
- Can apply variations to simulate real-world property differences

**Future Enhancements**: Could implement A* or RRT for more sophisticated path planning in complex environments.

---

### 3.5 Hybrid Vision System (`hybrid_vision.py`) ✅

**Purpose**: Integrate overhead camera (global view) with onboard camera (local view) for improved navigation.

**Implementation Details**:

- **Vision Fusion**: Combines data from overhead and onboard cameras
  - Overhead camera: Global target detection, robot localization, path planning
  - Onboard camera: Local obstacle detection, target verification, fine navigation
  - Weighted fusion: Overhead (70%) + Onboard (30%)

- **Target Detection**:
  - Overhead: Detects all targets in arena
  - Onboard: Verifies targets detected by overhead camera
  - Duplicate removal: Filters same targets seen from different views

- **Obstacle Detection**:
  - Onboard camera uses edge detection and contour analysis
  - Detects obstacles in robot's local view
  - Converts to world coordinates for path planning

- **Confidence Management**: Tracks confidence in fused results based on:
  - Overhead camera availability and quality
  - Onboard camera availability and quality
  - Number of targets/obstacles detected

**Key Classes**:
- `VisionFusion`: Dataclass containing fused vision results
- `HybridVisionSystem`: Main class for vision fusion

---

### 3.6 Error Recovery System (`error_recovery.py`) ✅

**Purpose**: Detect navigation errors and implement recovery behaviors.

**Implementation Details**:

- **Error Detection**:
  - **Stuck Detection**: Monitors position history, detects lack of progress
  - **Lost Localization**: Detects when robot position is unknown
  - **Obstacle Blocked**: Detects when path is completely blocked
  - **Timeout**: Monitors operation duration
  - **Sensor Failure**: Detects invalid sensor readings

- **Recovery Behaviors**:
  - **Stuck**: Back up, turn left/right alternately
  - **Lost**: Rotate in place to help camera find robot
  - **Blocked**: Back up and try different direction
  - **Timeout**: Stop and reassess
  - **Sensor Failure**: Stop for safety

- **Position Tracking**: Maintains position history for stuck detection
  - Tracks last 50 positions with timestamps
  - Calculates movement over time windows
  - Detects stuck if movement < threshold over time period

**Key Classes**:
- `ErrorType`: Enum for error types
- `ErrorState`: Dataclass for error state tracking
- `ErrorRecovery`: Main class for error detection and recovery

---

### 3.7 Performance Optimizer (`performance_optimizer.py`) ✅

**Purpose**: Optimize navigation system performance.

**Implementation Details**:

- **Adaptive Frame Skipping**: Maintains target FPS (default 10 FPS)
  - Skips frames if processing is too slow
  - Adaptively adjusts skip rate based on performance
  - Prevents system overload

- **Result Caching**: Caches expensive computations
  - Configurable cache size (default 10 entries)
  - Tracks cache hit/miss rates
  - FIFO eviction policy

- **Processing Time Tracking**: Monitors frame processing time
  - Tracks last 30 frames
  - Calculates average processing time
  - Adjusts frame skipping based on performance

- **Performance Metrics**:
  - Frame rate (FPS)
  - Processing time per frame
  - Cache hit rate
  - Skipped frames count

- **Optimization Modes**:
  - **Low Power**: Reduced FPS (5 FPS), increased frame skipping
  - **Performance**: Maximum FPS (15 FPS), minimal skipping

**Key Classes**:
- `PerformanceMetrics`: Dataclass for performance metrics
- `PerformanceOptimizer`: Main class for performance optimization

---

### 4. Navigation Controller (`navigation_controller.py`) ✅

**Purpose**: Control robot movement using PID control and state machine.

**Implementation Details**:

- **State Machine**: Implements 8 navigation states:
  - `IDLE`: Robot stopped, waiting for command
  - `SEARCHING`: Rotating in place to find targets
  - `TRACKING`: Target detected, preparing to navigate
  - `NAVIGATING`: Moving toward waypoint/target
  - `AVOIDING`: Obstacle detected, executing avoidance maneuver
  - `ARRIVED`: Reached target, stopped
  - `RETURNING`: Returning to start position
  - `ERROR`: Error state for recovery

- **PID Control**: Two PID controllers:
  - **Heading PID**: Controls robot orientation to face target
    - Error: `target_angle - current_theta` (normalized to [-π, π])
    - Output: Differential drive correction
  - **Distance PID**: Controls speed based on distance (currently simple proportional)
  
  PID formula: `output = Kp×error + Ki×∫error + Kd×d(error)/dt`

- **Differential Drive Control**: Converts heading correction to left/right motor speeds:
  ```
  left_speed = base_speed - heading_correction
  right_speed = base_speed + heading_correction
  ```
  Base speed scales with distance to target (closer = slower for precision).

- **Obstacle Avoidance**: Reactive avoidance using sensor fusion:
  - **Ultrasonic sensor**: Detects forward obstacles (< 20cm threshold)
  - **IR sensors**: Detect left/right obstacles (binary: 1 = obstacle)
  - **Avoidance behavior**:
    - Both sides blocked: Back up
    - Left blocked: Turn right
    - Right blocked: Turn left
    - Front only: Turn right (default)

- **Arrival Detection**: Checks if robot is within `arrival_threshold` (15cm) of target. When arrived, transitions to `ARRIVED` state and stops motors.

- **Speed Control**: Dynamic speed based on distance:
  - Maximum speed at far distances
  - Minimum speed when close to target
  - Scales linearly: `speed = min(max_speed, distance × scale_factor), min_speed)`

**Key Classes**:
- `PIDController`: Generic PID controller with reset capability.
- `NavigationState`: Enum for state machine states.
- `ControlCommand`: Dataclass for motor and servo commands.
- `NavigationController`: Main class integrating state machine, PID, and obstacle avoidance.

---

### 5. Main Navigation Loop (`main.py`) ✅

**Purpose**: Integrate all modules for autonomous navigation.

**Implementation Details**:

- **System Initialization**: 
  - Connects to robot via WebSocket (port 8765)
  - Starts robot camera stream (port 9000)
  - Loads world transformation matrix if available
  - Initializes all navigation modules

- **Asynchronous Architecture**: Uses Python `asyncio` for non-blocking operations:
  - Robot communication (WebSocket send/receive)
  - Camera frame processing
  - Navigation loop execution

- **Overhead Camera Integration**: 
  - Connects to overhead camera stream (Flask server)
  - Parses MJPEG stream (finds JPEG boundaries: `\xff\xd8` to `\xff\xd9`)
  - Updates frames in real-time for processing

- **Main Control Loop** (runs at ~10Hz):
  1. **Target Detection**: Detects targets in overhead frame using `TargetDetector`
  2. **Robot Localization**: Localizes robot position using `RobotLocalizer`
  3. **Target Selection**: Selects closest target to current position
  4. **Path Planning**: Plans path from current position to target using `PathPlanner`
  5. **Waypoint Management**: Gets next waypoint and sets it in navigation controller
  6. **Control Update**: Updates navigation controller, gets motor commands
  7. **Robot Control**: Sends commands to robot via WebSocket
  8. **Status Display**: Prints current state, position, and target count

- **Error Handling**: 
  - Handles connection failures with reconnection logic
  - Graceful shutdown on keyboard interrupt
  - Exception handling with traceback for debugging

- **Modular Design**: Each module can be used independently or together:
  - Can run with or without overhead camera
  - Supports different target colors and robot markers
  - Configurable via command-line arguments

**Key Features**:
- Real-time visualization capability (can be extended)
- Configurable target and robot marker colors
- Supports both overhead and onboard camera modes
- Clean shutdown with proper resource cleanup

**Usage Example**:
```bash
python main.py --robot-ip 192.168.1.216 \
               --camera-url http://192.168.1.109:5000/ \
               --target-color blue \
               --robot-marker-color green \
               --save-path-images \
               --display-path-images
```

**New Features**:
- **Dead Reckoning**: Handles webcam delay by estimating robot position using motor commands and odometry
- **Path Visualization**: Overlays robot path, waypoints, and goal on camera feed images
  - Use `--save-path-images` to save visualization images to `path_images/` directory
  - Use `--display-path-images` to show real-time visualization window

**Phase 3 Features**:
- **Hybrid Vision**: Integrates overhead and onboard cameras for improved navigation
- **Error Recovery**: Automatic error detection and recovery behaviors
- **Performance Optimization**: Adaptive frame skipping, caching, performance monitoring
- **Enhanced Visualization**: Performance metrics, error status, vision status overlays


# Setup Instructions

## Prerequisites

```bash
# Python dependencies
pip install -r robot_code/python_client/requirements.txt
pip install -r webcam_code/requirements.txt

# Additional dependencies for navigation
pip install scipy matplotlib
```

## Robot Configuration

1. **Arduino Setup**
   - Upload `galaxy-rvr.ino` to Arduino
   - Configure WiFi credentials in the code
   - Verify WebSocket connection on port 8765

2. **Network Configuration**
   - Ensure robot and computer are on same network
   - Note robot IP address (default: 192.168.1.216)
   - Test connection with `simple_control.py`

## Arena Setup

1. **Physical Setup**
   - Place red corner markers at arena corners
   - Position overhead webcam above arena
   - Ensure good lighting conditions

2. **Calibration** (Phase 1 - Required Before Navigation)
   
   **Quick Calibration:**
   ```bash
   # Capture calibration image
   python3 autonomous_navigation/capture_calibration_image.py --url http://192.168.0.21:8000/
   
   # Run full calibration
   python3 calibrate.py full-calibration --image calibration_image_*.png
   ```
   
   **Step-by-Step:**
   ```bash
   # 1. Verify red corners
   python3 calibrate.py verify-corners --image calibration_image.png
   
   # 2. Calibrate world coordinates
   python3 calibrate.py calibrate-world --image calibration_image.png
   
   # 3. Test transformation
   python3 calibrate.py test-transform --image calibration_image.png
   
   # 4. Validate robot
   python3 calibrate.py validate-robot --robot-ip 192.168.1.216
   
   # 5. Validate sensors
   python3 calibrate.py validate-sensors --robot-ip 192.168.1.216
   ```
   
   See `autonomous_navigation/CALIBRATION_GUIDE.md` for detailed instructions.

3. **Start Camera Stream**
   ```bash
   # Start webcam server with transformation
   cd webcam_code
   python3 webcam_stream.py --transform --camera 0
   ```

## 📡 Dead Reckoning & Camera Delay Handling

### Problem
The overhead webcam feed experiences significant delays (10+ seconds), which causes the robot to:
- Stop and wait for camera updates
- Make decisions based on stale position data
- Miss waypoints or overshoot targets

### Solution: Dead Reckoning / Odometry

Instead of waiting for camera updates, the system uses **dead reckoning** to continuously estimate the robot's position:

1. **Initialization**: When camera provides first position, dead reckoning is initialized
2. **Continuous Updates**: Motor commands are used to estimate position using differential drive kinematics
3. **Camera Correction**: When camera updates arrive, dead reckoning is corrected with ground truth
4. **Seamless Navigation**: Robot continues moving even during camera delays

**Implementation** (`dead_reckoning.py`):
- Uses differential drive kinematics model
- Tracks wheel speeds from motor commands
- Estimates position and orientation continuously
- Confidence decreases over time without camera updates
- Automatically corrects when camera data arrives

**Benefits**:
- ✅ Robot never stops waiting for camera
- ✅ Smooth, continuous navigation
- ✅ Handles delays up to 20+ seconds
- ✅ Automatic correction when camera updates

## 📊 Path Visualization

The system can overlay navigation information directly on the camera feed:

**Visualization Elements**:
- **Green Line**: Robot's actual path/trajectory
- **Blue Circle**: Current robot position
- **Orange Circles**: Planned waypoints (W1, W2, ...)
- **Red Line**: Path to next waypoint
- **Yellow Circle**: Goal position (blue line at top)

**Usage**:
```bash
# Save path images to disk
python main.py --save-path-images ...

# Display real-time visualization window
python main.py --display-path-images ...

# Both
python main.py --save-path-images --display-path-images ...
```

**Implementation** (`path_visualization.py`):
- Converts world coordinates to pixel coordinates using transform matrix
- Draws path history, waypoints, and goal on camera frames
- Maintains last 100 path points for visualization
- Can save images or display in real-time window

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


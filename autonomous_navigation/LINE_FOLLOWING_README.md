# Line Following Navigation System

## Overview

This system uses the overhead camera to detect lines on the floor and guides the robot to follow those lines using path planning. This is a revamped approach that focuses on line detection and following rather than target-based navigation.

## Key Features

- **Overhead Camera Integration**: Uses overhead camera to capture floor images
- **Line Detection**: Detects colored lines (blue, green, or red) on the floor
- **Path Planning**: Creates waypoints along detected lines for the robot to follow
- **Smart Line Selection**: Selects the best line to follow based on:
  - Distance from robot
  - Alignment with robot's forward direction
  - Line length
- **Continuous Following**: Maintains a following distance from the line while navigating

## Architecture

### Components

1. **LineFollowingNavigation** (`line_following_navigation.py`)
   - Main navigation system for real robot
   - Integrates overhead camera, line detection, and navigation control

2. **SimulatedLineFollowing** (`simulate_line_following.py`)
   - Simulation version for testing without physical robot
   - Creates synthetic overhead images with lines

3. **LineDetector** (`line_detection.py`)
   - Detects colored lines in images using HSV color space and Hough line detection
   - Converts pixel coordinates to world coordinates

4. **NavigationController** (`navigation_controller.py`)
   - Controls robot movement with PID control
   - Includes fixes for preventing spinning and ensuring straight movement

## Usage

### Real Robot

```bash
# Run with overhead camera
python3 autonomous_navigation/line_following_navigation.py \
    --robot-ip 192.168.0.113 \
    --camera-url http://192.168.0.21:8000/ \
    --line-color blue \
    --robot-marker-color green
```

### Simulation

```bash
# Run simulated line following
python3 autonomous_navigation/simulate_line_following.py \
    --initial-x 0.5 \
    --initial-y 0.5 \
    --initial-theta 0.0 \
    --max-iterations 500 \
    --no-wait
```

## How It Works

1. **Image Capture**: Overhead camera captures current floor image
2. **Line Detection**: Detects colored lines in the image using HSV color filtering and Hough line detection
3. **Robot Localization**: Determines robot position and orientation from overhead view
4. **Line Selection**: Selects the best line to follow based on proximity, alignment, and length
5. **Path Generation**: Creates waypoints along the selected line
6. **Navigation**: Robot follows waypoints using PID control, maintaining a following distance from the line

## Line Following Algorithm

### Waypoint Generation

For a detected line, the system:
1. Projects robot position onto the line to find the closest point
2. Creates waypoints ahead along the line (lookahead distance)
3. Offsets waypoints perpendicular to the line to maintain following distance
4. Updates waypoints as robot moves

### Line Selection Scoring

Each detected line is scored based on:
- **Distance Score** (30%): Closer lines are preferred
- **Alignment Score** (40%): Lines aligned with robot's forward direction are preferred
- **Length Score** (30%): Longer lines are preferred

## Parameters

### Line Following Parameters

- `line_follow_distance`: Distance to maintain from line (default: 0.15m)
- `max_lookahead`: Maximum lookahead distance along line (default: 0.5m)

### Navigation Controller Parameters

- `max_speed`: Maximum motor speed (default: 60)
- `min_speed`: Minimum motor speed (default: 30)
- `heading_dead_zone`: Dead zone for heading error (default: 0.1 rad ≈ 6°)
- `max_correction`: Maximum heading correction (default: 20)

## Navigation Controller Fixes

The navigation controller includes fixes to prevent spinning:

1. **Reduced PID gains**: kp reduced from 2.0 to 1.0
2. **Heading dead zone**: Small heading errors (< 0.1 rad) are ignored
3. **Limited correction**: Maximum heading correction capped at 20
4. **Motor compensation**: Framework for compensating motor differences
5. **Search timeout**: Prevents getting stuck in SEARCHING state

## Testing

### Virtual Tests

Run the simulation to test line following:

```bash
cd autonomous_navigation
python3 simulate_line_following.py --no-wait
```

### Real Robot Tests

1. Ensure overhead camera is running and accessible
2. Ensure world transform matrix is calibrated (`auto_transform_matrix.npy`)
3. Run the line following navigation script
4. Monitor robot behavior - it should follow detected lines smoothly

## Troubleshooting

### Robot Spinning

- Check motor compensation values in navigation controller
- Verify PID gains are not too high
- Ensure heading dead zone is working

### Lines Not Detected

- Check camera image quality
- Verify line color matches `--line-color` parameter
- Adjust HSV color ranges in `LineDetector` if needed
- Check lighting conditions

### Robot Not Following Lines

- Verify robot localization is working
- Check that waypoints are being generated
- Ensure navigation controller state is NAVIGATING
- Check that motor commands are being sent

## Future Improvements

- Dynamic line color detection (detect multiple colors)
- Adaptive following distance based on line curvature
- Line intersection handling
- Obstacle avoidance while line following
- Multi-line path planning (switch between lines)


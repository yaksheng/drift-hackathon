# Simulation Environment

This simulation environment allows you to test the autonomous navigation system without physical hardware. It's designed to work when you don't have access to Drift AI or enough time for physical testing.

## Features

- **Full Robot Simulation**: Simulates GalaxyRVR robot with differential drive kinematics
- **Arena Visualization**: Real-time visualization of robot, targets, obstacles, and paths
- **Line Stopping Challenge**: Supports stopping at 1st, 2nd, or 3rd line
- **Obstacle Avoidance**: Simulated ultrasonic and IR sensors
- **Path Planning**: Visualizes planned paths and waypoints
- **No Hardware Required**: Test navigation algorithms without robot

## Quick Start

### Installation

```bash
# Install dependencies
pip install -r requirements.txt
```

### Basic Usage

**From project root (recommended):**
```bash
# Stop at first line (default)
python3 scripts/simulate.py --stop-at-line 1

# Stop at second line
python3 scripts/simulate.py --stop-at-line 2

# Stop at third line
python3 scripts/simulate.py --stop-at-line 3
```

**From autonomous_navigation directory:**
```bash
cd autonomous_navigation

# Stop at first line (default)
python3 simulate_navigation.py --stop-at-line 1

# Stop at second line
python3 simulate_navigation.py --stop-at-line 2

# Stop at third line
python3 simulate_navigation.py --stop-at-line 3
```

### Advanced Options

```bash
# Custom starting position (from project root)
python3 scripts/simulate.py \
    --stop-at-line 2 \
    --initial-x 0.3 \
    --initial-y 0.5 \
    --initial-theta 0.0

# Limit iterations
python3 scripts/simulate.py --stop-at-line 1 --max-iterations 500
```

**Note**: On macOS, use `python3` instead of `python`.

## Command Line Arguments

- `--stop-at-line`: Which line to stop at (1, 2, or 3) - **Required for challenge**
- `--initial-x`: Starting X position in meters (default: 0.5)
- `--initial-y`: Starting Y position in meters (default: 0.5)
- `--initial-theta`: Starting orientation in radians (default: 0.0)
- `--max-iterations`: Maximum navigation iterations (default: 1000)

## Simulation Components

### 1. SimulatedRobot (`simulator.py`)
- Implements differential drive kinematics
- Simulates ultrasonic and IR sensors
- Handles obstacle collisions
- Tracks line crossings

### 2. ArenaVisualizer (`simulator_visualization.py`)
- Real-time matplotlib visualization
- Shows robot, targets, obstacles, lines, and paths
- Updates at ~10Hz

### 3. MockGalaxyRVR (`mock_robot.py`)
- Compatible interface with real GalaxyRVR
- Allows existing navigation code to work unchanged
- Translates commands to simulator

### 4. SimulatedNavigation (`simulate_navigation.py`)
- Main simulation script
- Integrates all modules
- Handles line stopping logic

## Default Arena Setup

- **Arena Size**: 2.5m × 4.0m
- **Lines**: 3 horizontal lines at y = 1.0, 2.0, 3.0 meters
- **Obstacles**: 2 obstacles at (1.0, 2.0) and (1.5, 1.5)
- **Targets**: 3 blue targets at various positions

## Line Stopping Challenge

The simulation tracks when the robot crosses each line. When the robot crosses the target line (1st, 2nd, or 3rd), it will:
1. Stop immediately
2. Display success message
3. Highlight the target line in green

## Visualization

The visualization shows:
- **Blue circle**: Robot position and orientation (arrow shows heading)
- **Colored circles**: Targets
- **Red circles**: Obstacles
- **Blue lines**: Regular lines
- **Green dashed line**: Target line to stop at
- **Green dashed path**: Planned navigation path
- **Status box**: Current state, position, and line crossing status

## Customization

You can customize the simulation by modifying `simulate_navigation.py`:

```python
# Custom targets
target_positions = [
    (2.0, 3.5, 'blue'),
    (1.5, 3.0, 'green'),
]

# Custom obstacles
obstacles = [
    (1.0, 2.0, 0.2),  # (x, y, radius)
    (1.5, 1.5, 0.15),
]

# Custom lines
lines = [
    (0.0, 1.0, 2.5, 1.0),  # (x1, y1, x2, y2)
    (0.0, 2.0, 2.5, 2.0),
    (0.0, 3.0, 2.5, 3.0),
]
```

## Integration with Real Robot

The simulation uses the same navigation modules as the real robot:
- `target_detection.py`
- `robot_localization.py`
- `path_planner.py`
- `navigation_controller.py`

This means code tested in simulation should work with minimal changes on the real robot (just switch from `MockGalaxyRVR` to real `GalaxyRVR`).

## Troubleshooting

### Visualization not updating
- Make sure matplotlib is in interactive mode (`plt.ion()`)
- Check that the window is not minimized

### Robot not moving
- Check that navigation controller state is not `IDLE` or `ARRIVED`
- Verify targets are detected
- Ensure robot is not stuck at arena boundaries

### Line not detected
- Verify line coordinates are correct
- Check that robot is actually crossing the line (not just near it)
- Increase line detection sensitivity if needed

## Performance

- Simulation runs at ~10Hz (10 updates per second)
- Visualization updates in real-time
- Suitable for algorithm testing and debugging

## Next Steps

1. Test different line stopping scenarios (1st, 2nd, 3rd)
2. Adjust PID controller parameters
3. Test obstacle avoidance behavior
4. Customize arena layout for your challenge
5. Once satisfied, test on real robot

---

**Note**: This simulation is a simplified model. Real robot behavior may differ due to:
- Motor characteristics
- Sensor noise
- Surface friction
- Battery voltage
- Environmental factors


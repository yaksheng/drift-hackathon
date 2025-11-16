# Visual Robot Tests

This document describes how to run visual tests to see the robot in action.

## Quick Start

Run the full navigation simulation with visualization:
```bash
python3 simulate.py --stop-at-line 1 --max-iterations 200
```

## Test Scenarios

### Scenario 1: Basic Robot Movement
Tests basic robot movement and rotation:
```bash
python3 autonomous_navigation/test_visual_scenarios.py --scenario 1
```

**What you'll see:**
- Robot moving forward
- Robot rotating
- Path history (green line)

### Scenario 2: Path Planning
Visualizes path planning from start to goal:
```bash
python3 autonomous_navigation/test_visual_scenarios.py --scenario 2
```

**What you'll see:**
- Start position (robot)
- Goal position (blue target)
- Planned path (green line)

### Scenario 3: Obstacle Avoidance
Shows path planning with obstacles:
```bash
python3 autonomous_navigation/test_visual_scenarios.py --scenario 3
```

**What you'll see:**
- Obstacles (red circles)
- Planned path avoiding obstacles
- Robot at start position

### Scenario 4: Full Navigation
Complete navigation to goal with real-time updates:
```bash
python3 autonomous_navigation/test_visual_scenarios.py --scenario 4
```

**What you'll see:**
- Robot moving in real-time
- Path being drawn as robot moves
- Waypoints (orange squares)
- Obstacles
- Goal line at top (blue)
- Status updates

### Run All Scenarios
```bash
python3 autonomous_navigation/test_visual_scenarios.py --all
```

## Visualization Elements

The visualization shows:

- **Blue Circle**: Robot position
- **Blue Arrow**: Robot orientation
- **Green Line**: Robot's path/trajectory
- **Orange Squares**: Planned waypoints
- **Red Circles**: Static obstacles
- **Orange Circles (dashed)**: Sensor-detected obstacles
- **Blue Target**: Goal position
- **Blue Line (thick)**: Goal line at top of arena
- **Status Text**: Current state, position, distance to goal

## Using the Existing Simulation

The main simulation script also provides visualization:

```bash
# Basic run
python3 simulate.py

# With custom parameters
python3 simulate.py --stop-at-line 1 --max-iterations 300 \
                    --initial-x 0.5 --initial-y 0.5

# Parameters:
#   --stop-at-line: Which line to track (1, 2, or 3)
#   --max-iterations: Maximum navigation iterations
#   --initial-x: Starting X position (meters)
#   --initial-y: Starting Y position (meters)
#   --initial-theta: Starting orientation (radians)
```

## What to Observe

When running visual tests, observe:

1. **Path Planning**: Does the robot plan a reasonable path?
2. **Obstacle Avoidance**: Does it avoid obstacles correctly?
3. **Goal Reaching**: Does it reach the goal efficiently?
4. **Smooth Movement**: Is the movement smooth and controlled?
5. **Waypoint Following**: Does it follow waypoints accurately?

## Troubleshooting

### Window doesn't appear
- Ensure matplotlib is installed: `pip3 install matplotlib`
- Try different backends if needed
- Check if running in headless mode

### Window closes immediately
- Add `plt.pause()` or `input()` to keep window open
- Check if script completes too quickly

### Performance issues
- Reduce `--max-iterations`
- Increase `plt.pause()` delay
- Reduce visualization update frequency


# Pre-Mapped Obstacles System

## Overview

The navigation system now uses **pre-mapped obstacles** that reflect real-world conditions. Obstacles have known positions (pre-mapped) but can have variable sizes, positions, and colors, simulating real-world scenarios where obstacle locations are known but their exact properties may vary.

## Key Changes

### 1. Pathfinding Logic
- Uses the improved pathfinding logic from commit `b7de67ab496a21d656cbb9c4d97d9e8b2f60013d`
- Better obstacle avoidance with `plan_obstacle_avoidance_path()`
- Path clearance checking with `is_path_clear()`
- Blocking obstacle detection with `get_blocking_obstacles()`

### 2. Pre-Mapped Obstacles (`obstacle_map.py`)
- **ObstacleMap** class manages pre-mapped obstacles
- Obstacles have known positions but variable properties:
  - **Position**: Known but can vary slightly (±0.1m)
  - **Size (radius)**: Variable (0.10m - 0.25m)
  - **Color**: Variable (red, blue, green, yellow, orange)
- Default map includes 3 pre-mapped obstacles:
  1. (1.0, 1.5) - radius 0.20m - red
  2. (1.5, 2.0) - radius 0.15m - blue
  3. (2.0, 1.0) - radius 0.18m - green

### 3. Obstacle Data Structure
- `Obstacle` dataclass now includes:
  - `x`, `y`: Position (meters)
  - `radius`: Size (meters)
  - `confidence`: Detection confidence (0-1)
  - `color`: Obstacle color (for visualization/identification)

### 4. Visualization Updates
- Obstacles are drawn with their assigned colors
- Color mapping:
  - Red → darkred edge
  - Blue → darkblue edge
  - Green → darkgreen edge
  - Yellow → gold edge
  - Orange → darkorange edge

## Usage

### Using Default Pre-Mapped Obstacles

```python
from obstacle_map import get_default_obstacle_map
from path_planner import PathPlanner

# Get pre-mapped obstacles
obstacle_map = get_default_obstacle_map()
obstacles = obstacle_map.get_obstacles()

# Add to path planner
planner = PathPlanner()
for obs in obstacles:
    planner.add_obstacle(obs)

# Plan path (will use obstacle avoidance)
path = planner.plan_obstacle_avoidance_path((0.5, 0.5), (2.0, 3.0))
```

### Custom Obstacle Map

```python
from obstacle_map import ObstacleMap
from path_planner import Obstacle

# Create custom map
obstacle_map = ObstacleMap()

# Add obstacles with known positions
obstacle_map.add_obstacle(1.0, 1.5, 0.2, color='red')
obstacle_map.add_obstacle(1.5, 2.0, 0.15, color='blue')

# Save to file
obstacle_map.save_to_file('custom_obstacles.json')

# Load from file
obstacle_map = ObstacleMap('custom_obstacles.json')
```

### Applying Variations

To simulate real-world variations in obstacle properties:

```python
from obstacle_map import get_default_obstacle_map

obstacle_map = get_default_obstacle_map()

# Apply variations (simulates real-world differences)
obstacle_map.apply_variations(
    position_variance=0.1,  # ±10cm position variation
    radius_variance=0.05,  # ±5cm radius variation
    color_options=['red', 'blue', 'green', 'yellow']
)
```

## Integration with Simulation

The simulation (`simulate_navigation.py`) now automatically:
1. Loads pre-mapped obstacles on initialization
2. Adds them to both the robot simulator and path planner
3. Visualizes them with their assigned colors
4. Uses the improved pathfinding logic for navigation

## Real-World Conditions

This system reflects real-world conditions where:
- ✅ Obstacle **positions are known** (pre-mapped from arena setup)
- ✅ Obstacle **sizes can vary** (different objects)
- ✅ Obstacle **colors can vary** (different colored objects)
- ✅ Pathfinding uses **better obstacle avoidance** logic
- ✅ Tests and simulations use **consistent obstacle maps**

## Benefits

1. **Realistic Testing**: Tests reflect real-world conditions with known obstacle positions
2. **Better Pathfinding**: Uses improved obstacle avoidance algorithm
3. **Visual Clarity**: Color-coded obstacles for better visualization
4. **Flexibility**: Easy to customize obstacle maps for different scenarios
5. **Consistency**: Same obstacle map used across tests and simulations

## Files Modified

- `path_planner.py`: Added `color` field to `Obstacle` dataclass
- `obstacle_map.py`: New module for managing pre-mapped obstacles
- `simulate_navigation.py`: Updated to use pre-mapped obstacles
- `simulator_visualization.py`: Updated to support obstacle colors
- `test_visual_scenarios.py`: Updated to use pre-mapped obstacles
- `__init__.py`: Added exports for `ObstacleMap` and `get_default_obstacle_map`


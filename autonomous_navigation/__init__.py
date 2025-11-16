"""
Autonomous Navigation Package

Main modules:
- target_detection: Target detection and tracking
- robot_localization: Robot position tracking
- path_planner: Path planning and obstacle avoidance
- navigation_controller: PID control and state machine
- main: Main integration and control loop
"""

from .target_detection import TargetDetector, Target
from .robot_localization import RobotLocalizer, RobotPose
from .path_planner import PathPlanner, Waypoint, Obstacle
from .navigation_controller import NavigationController, NavigationState, ControlCommand
from .line_detection import LineDetector, DetectedLine
from .dead_reckoning import DeadReckoning, OdometryState
from .path_visualization import PathVisualizer
from .obstacle_map import ObstacleMap, get_default_obstacle_map

__all__ = [
    'TargetDetector',
    'Target',
    'RobotLocalizer',
    'RobotPose',
    'PathPlanner',
    'Waypoint',
    'Obstacle',
    'NavigationController',
    'NavigationState',
    'ControlCommand',
    'LineDetector',
    'DetectedLine',
    'DeadReckoning',
    'OdometryState',
    'PathVisualizer',
    'ObstacleMap',
    'get_default_obstacle_map',
]


"""
Path Planning Module

Plans paths from current position to target, with obstacle avoidance.
Supports straight-line navigation, waypoint following, and dynamic path adjustment.
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import math


@dataclass
class Waypoint:
    """Represents a waypoint in the path"""
    x: float  # X position in world coordinates
    y: float  # Y position in world coordinates
    reached: bool = False


@dataclass
class Obstacle:
    """Represents an obstacle in the arena"""
    x: float  # X position in world coordinates
    y: float  # Y position in world coordinates
    radius: float  # Obstacle radius in meters
    confidence: float = 1.0  # Detection confidence


class PathPlanner:
    """Plans and manages navigation paths"""
    
    def __init__(self, 
                 arena_bounds: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None,
                 obstacle_clearance: float = 0.2):  # meters
        """
        Initialize path planner
        
        Args:
            arena_bounds: ((min_x, min_y), (max_x, max_y)) arena boundaries
            obstacle_clearance: Minimum clearance from obstacles (meters)
        """
        self.arena_bounds = arena_bounds
        self.obstacle_clearance = obstacle_clearance
        self.obstacles: List[Obstacle] = []
        self.current_path: List[Waypoint] = []
        
    def add_obstacle(self, obstacle: Obstacle):
        """Add an obstacle to the map"""
        self.obstacles.append(obstacle)
    
    def update_obstacle_from_sensors(self, 
                                    robot_pos: Tuple[float, float],
                                    robot_theta: float,
                                    ultrasonic_distance: Optional[float],
                                    ir_left: Optional[int],
                                    ir_right: Optional[int]):
        """
        Update obstacles dynamically from sensor readings
        
        Args:
            robot_pos: Current robot position (x, y)
            robot_theta: Current robot orientation
            ultrasonic_distance: Forward distance in cm (None if no reading)
            ir_left: Left IR sensor (1=obstacle, 0=clear)
            ir_right: Right IR sensor (1=obstacle, 0=clear)
        """
        # Clear old sensor-based obstacles
        self.obstacles = [obs for obs in self.obstacles if obs.confidence >= 0.9]  # Keep high-confidence obstacles
        
        # Add forward obstacle from ultrasonic
        if ultrasonic_distance is not None and ultrasonic_distance < 50:  # Less than 50cm
            # Convert to meters
            dist_m = ultrasonic_distance / 100.0
            
            # Calculate obstacle position in front of robot
            obs_x = robot_pos[0] + dist_m * np.cos(robot_theta)
            obs_y = robot_pos[1] + dist_m * np.sin(robot_theta)
            
            # Add obstacle (smaller confidence since it's from sensors)
            obstacle = Obstacle(
                x=obs_x,
                y=obs_y,
                radius=0.15,  # Assume 15cm radius
                confidence=0.7  # Lower confidence for sensor-based obstacles
            )
            self.obstacles.append(obstacle)
        
        # Add side obstacles from IR sensors
        side_offset = 0.15  # 15cm to the side
        if ir_left == 1:
            # Obstacle on left side
            obs_x = robot_pos[0] + side_offset * np.cos(robot_theta + np.pi/2)
            obs_y = robot_pos[1] + side_offset * np.sin(robot_theta + np.pi/2)
            obstacle = Obstacle(x=obs_x, y=obs_y, radius=0.10, confidence=0.6)
            self.obstacles.append(obstacle)
        
        if ir_right == 1:
            # Obstacle on right side
            obs_x = robot_pos[0] + side_offset * np.cos(robot_theta - np.pi/2)
            obs_y = robot_pos[1] + side_offset * np.sin(robot_theta - np.pi/2)
            obstacle = Obstacle(x=obs_x, y=obs_y, radius=0.10, confidence=0.6)
            self.obstacles.append(obstacle)
    
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles = []
    
    def plan_straight_path(self, 
                          start: Tuple[float, float],
                          goal: Tuple[float, float]) -> List[Waypoint]:
        """
        Plan a straight-line path from start to goal
        
        Args:
            start: (x, y) start position
            goal: (x, y) goal position
            
        Returns:
            List of waypoints
        """
        # Simple straight-line path
        waypoints = [Waypoint(x=goal[0], y=goal[1])]
        self.current_path = waypoints
        return waypoints
    
    def plan_obstacle_avoidance_path(self,
                                    start: Tuple[float, float],
                                    goal: Tuple[float, float]) -> List[Waypoint]:
        """
        Plan path avoiding obstacles
        
        Args:
            start: (x, y) start position
            goal: (x, y) goal position
            
        Returns:
            List of waypoints avoiding obstacles
        """
        # Check if straight path is clear
        if self.is_path_clear(start, goal):
            return self.plan_straight_path(start, goal)
        
        # Simple obstacle avoidance: go around nearest obstacle
        # More sophisticated: A* or RRT could be used here
        waypoints = []
        
        # Find obstacles in the way
        blocking_obstacles = self.get_blocking_obstacles(start, goal)
        
        if not blocking_obstacles:
            # No obstacles, straight path
            waypoints = [Waypoint(x=goal[0], y=goal[1])]
        else:
            # Go around first blocking obstacle
            obstacle = blocking_obstacles[0]
            
            # Calculate waypoint to go around obstacle
            # Simple approach: offset perpendicular to path
            dx = goal[0] - start[0]
            dy = goal[1] - start[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > 0:
                # Unit vector along path
                ux = dx / dist
                uy = dy / dist
                
                # Perpendicular vector (rotate 90 degrees)
                px = -uy
                py = ux
                
                # Offset distance
                offset = obstacle.radius + self.obstacle_clearance
                
                # Waypoint around obstacle
                waypoint_x = obstacle.x + px * offset
                waypoint_y = obstacle.y + py * offset
                
                # Check if waypoint is in bounds
                if self.arena_bounds:
                    min_x, min_y = self.arena_bounds[0]
                    max_x, max_y = self.arena_bounds[1]
                    waypoint_x = max(min_x, min(max_x, waypoint_x))
                    waypoint_y = max(min_y, min(max_y, waypoint_y))
                
                waypoints = [
                    Waypoint(x=waypoint_x, y=waypoint_y),
                    Waypoint(x=goal[0], y=goal[1])
                ]
            else:
                waypoints = [Waypoint(x=goal[0], y=goal[1])]
        
        self.current_path = waypoints
        return waypoints
    
    def is_path_clear(self, 
                     start: Tuple[float, float],
                     goal: Tuple[float, float]) -> bool:
        """
        Check if straight path from start to goal is clear of obstacles
        
        Args:
            start: (x, y) start position
            goal: (x, y) goal position
            
        Returns:
            True if path is clear
        """
        for obstacle in self.obstacles:
            # Calculate distance from line segment to obstacle
            dist = self.point_to_line_distance(
                start, goal, (obstacle.x, obstacle.y)
            )
            
            if dist < (obstacle.radius + self.obstacle_clearance):
                return False
        
        return True
    
    def get_blocking_obstacles(self,
                              start: Tuple[float, float],
                              goal: Tuple[float, float]) -> List[Obstacle]:
        """
        Get obstacles blocking the path
        
        Args:
            start: (x, y) start position
            goal: (x, y) goal position
            
        Returns:
            List of blocking obstacles
        """
        blocking = []
        
        for obstacle in self.obstacles:
            dist = self.point_to_line_distance(
                start, goal, (obstacle.x, obstacle.y)
            )
            
            if dist < (obstacle.radius + self.obstacle_clearance):
                blocking.append(obstacle)
        
        # Sort by distance from start
        blocking.sort(key=lambda o: self.distance(start, (o.x, o.y)))
        
        return blocking
    
    def point_to_line_distance(self,
                               line_start: Tuple[float, float],
                               line_end: Tuple[float, float],
                               point: Tuple[float, float]) -> float:
        """
        Calculate distance from point to line segment
        
        Args:
            line_start: Start of line segment
            line_end: End of line segment
            point: Point to measure distance to
            
        Returns:
            Distance in meters
        """
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # Vector from start to end
        dx = x2 - x1
        dy = y2 - y1
        
        # Length squared
        length_sq = dx*dx + dy*dy
        
        if length_sq == 0:
            # Line is a point
            return self.distance(line_start, point)
        
        # Parameter t for closest point on line
        t = max(0, min(1, ((x0 - x1) * dx + (y0 - y1) * dy) / length_sq))
        
        # Closest point on line
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        # Distance to closest point
        return self.distance(point, (closest_x, closest_y))
    
    def distance(self, p1: Tuple[float, float], 
                p2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def get_next_waypoint(self) -> Optional[Waypoint]:
        """
        Get next unreached waypoint in current path
        
        Returns:
            Next waypoint, or None if all reached
        """
        for waypoint in self.current_path:
            if not waypoint.reached:
                return waypoint
        return None
    
    def mark_waypoint_reached(self, waypoint: Waypoint):
        """Mark a waypoint as reached"""
        waypoint.reached = True
    
    def is_path_complete(self) -> bool:
        """Check if all waypoints in current path are reached"""
        return all(wp.reached for wp in self.current_path)
    
    def replan_path(self,
                   current_pos: Tuple[float, float],
                   goal: Tuple[float, float]):
        """
        Replan path from current position to goal
        
        Args:
            current_pos: Current robot position
            goal: Goal position
        """
        # Clear old path
        self.current_path = []
        
        # Plan new path
        if self.obstacles:
            self.plan_obstacle_avoidance_path(current_pos, goal)
        else:
            self.plan_straight_path(current_pos, goal)
    
    def get_path_length(self) -> float:
        """Calculate total path length"""
        if not self.current_path:
            return 0.0
        
        total = 0.0
        prev = None
        
        for waypoint in self.current_path:
            if prev is not None:
                total += self.distance(prev, (waypoint.x, waypoint.y))
            prev = (waypoint.x, waypoint.y)
        
        return total


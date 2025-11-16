"""
Robot Simulator

Simulates the GalaxyRVR robot and arena environment for testing navigation
without physical hardware. Supports visualization and line detection.
"""

import numpy as np
import math
from typing import Tuple, Optional, List
from dataclasses import dataclass
import time


@dataclass
class SimulatedRobotState:
    """Simulated robot state"""
    x: float  # X position in meters
    y: float  # Y position in meters
    theta: float  # Orientation in radians
    left_motor: int = 0  # -100 to 100
    right_motor: int = 0  # -100 to 100
    servo_angle: int = 90  # 0 to 180
    battery_voltage: float = 7.4
    ultrasonic_distance: Optional[float] = None  # cm
    ir_left: int = 0  # 0 or 1
    ir_right: int = 0  # 0 or 1


class SimulatedRobot:
    """Simulates GalaxyRVR robot behavior"""
    
    def __init__(self, 
                 initial_pos: Tuple[float, float] = (0.5, 0.5),
                 initial_theta: float = 0.0,
                 arena_bounds: Tuple[Tuple[float, float], Tuple[float, float]] = None,
                 robot_radius: float = 0.15,  # meters
                 max_speed: float = 0.5):  # m/s
        """
        Initialize simulated robot
        
        Args:
            initial_pos: Starting position (x, y) in meters
            initial_theta: Starting orientation in radians
            arena_bounds: ((min_x, min_y), (max_x, max_y)) arena boundaries
            robot_radius: Robot radius in meters
            max_speed: Maximum linear speed in m/s
        """
        self.state = SimulatedRobotState(
            x=initial_pos[0],
            y=initial_pos[1],
            theta=initial_theta
        )
        
        self.arena_bounds = arena_bounds or ((0, 0), (2.5, 4.0))
        self.robot_radius = robot_radius
        self.max_speed = max_speed
        
        # Robot physical parameters
        self.wheel_base = 0.12  # meters (distance between wheels)
        self.wheel_radius = 0.03  # meters
        
        # Obstacles in the arena
        self.obstacles: List[Tuple[float, float, float]] = []  # (x, y, radius)
        
        # Lines in the arena (for stopping challenge)
        self.lines: List[Tuple[float, float, float, float]] = []  # (x1, y1, x2, y2)
        
        # Last update time
        self.last_update_time = time.time()
        
        # Previous position for line crossing detection
        self.prev_x = initial_pos[0]
        self.prev_y = initial_pos[1]
        
    def add_obstacle(self, x: float, y: float, radius: float):
        """Add an obstacle to the arena"""
        self.obstacles.append((x, y, radius))
    
    def add_line(self, x1: float, y1: float, x2: float, y2: float):
        """Add a line to the arena (for stopping challenge)"""
        self.lines.append((x1, y1, x2, y2))
    
    def set_motors(self, left: int, right: int):
        """Set motor speeds (-100 to 100)"""
        self.state.left_motor = max(-100, min(100, int(left)))
        self.state.right_motor = max(-100, min(100, int(right)))
    
    def set_servo(self, angle: int):
        """Set servo angle (0-180)"""
        self.state.servo_angle = max(0, min(180, int(angle)))
    
    def update(self, dt: float = None):
        """
        Update robot physics
        
        Args:
            dt: Time step in seconds (if None, uses actual elapsed time)
        """
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
        
        # Convert motor speeds (-100 to 100) to wheel speeds (m/s)
        left_speed = (self.state.left_motor / 100.0) * self.max_speed
        right_speed = (self.state.right_motor / 100.0) * self.max_speed
        
        # Differential drive kinematics
        # Linear velocity
        v = (left_speed + right_speed) / 2.0
        
        # Angular velocity
        omega = (right_speed - left_speed) / self.wheel_base
        
        # Update position
        if abs(omega) < 0.001:  # Straight line motion
            dx = v * math.cos(self.state.theta) * dt
            dy = v * math.sin(self.state.theta) * dt
        else:  # Curved motion
            # ICC (Instantaneous Center of Curvature)
            radius = v / omega
            icc_x = self.state.x - radius * math.sin(self.state.theta)
            icc_y = self.state.y + radius * math.cos(self.state.theta)
            
            # Rotation matrix
            cos_omega_dt = math.cos(omega * dt)
            sin_omega_dt = math.sin(omega * dt)
            
            # New position
            dx = (self.state.x - icc_x) * cos_omega_dt - (self.state.y - icc_y) * sin_omega_dt + icc_x
            dy = (self.state.x - icc_x) * sin_omega_dt + (self.state.y - icc_y) * cos_omega_dt + icc_y
            
            dx -= self.state.x
            dy -= self.state.y
        
        # Update state
        new_x = self.state.x + dx
        new_y = self.state.y + dy
        new_theta = self.state.theta + omega * dt
        
        # Normalize theta to [-pi, pi]
        while new_theta > math.pi:
            new_theta -= 2 * math.pi
        while new_theta < -math.pi:
            new_theta += 2 * math.pi
        
        # Check arena bounds
        min_x, min_y = self.arena_bounds[0]
        max_x, max_y = self.arena_bounds[1]
        
        # Keep robot within bounds (simple collision)
        if new_x < min_x + self.robot_radius:
            new_x = min_x + self.robot_radius
        elif new_x > max_x - self.robot_radius:
            new_x = max_x - self.robot_radius
        
        if new_y < min_y + self.robot_radius:
            new_y = min_y + self.robot_radius
        elif new_y > max_y - self.robot_radius:
            new_y = max_y - self.robot_radius
        
        # Store previous position before updating
        self.prev_x = self.state.x
        self.prev_y = self.state.y
        
        # Update state
        self.state.x = new_x
        self.state.y = new_y
        self.state.theta = new_theta
        
        # Update sensors
        self._update_sensors()
    
    def _update_sensors(self):
        """Update simulated sensor readings"""
        # Ultrasonic sensor (forward distance)
        forward_dist = self._get_forward_distance()
        self.state.ultrasonic_distance = forward_dist * 100  # Convert to cm
        
        # IR sensors (left and right)
        self.state.ir_left = 1 if self._check_obstacle_left() else 0
        self.state.ir_right = 1 if self._check_obstacle_right() else 0
    
    def _get_forward_distance(self) -> float:
        """Get distance to nearest obstacle in forward direction"""
        # Cast ray forward from robot
        ray_length = 2.0  # meters
        ray_end_x = self.state.x + ray_length * math.cos(self.state.theta)
        ray_end_y = self.state.y + ray_length * math.sin(self.state.theta)
        
        min_dist = ray_length
        
        for obs_x, obs_y, obs_radius in self.obstacles:
            # Distance from line segment to circle
            dist = self._point_to_line_distance(
                (self.state.x, self.state.y),
                (ray_end_x, ray_end_y),
                (obs_x, obs_y)
            )
            
            if dist < obs_radius + self.robot_radius:
                # Calculate actual distance along ray
                dx = obs_x - self.state.x
                dy = obs_y - self.state.y
                dist_to_center = math.sqrt(dx*dx + dy*dy)
                dist_along_ray = dist_to_center - obs_radius - self.robot_radius
                min_dist = min(min_dist, max(0, dist_along_ray))
        
        return min_dist
    
    def _check_obstacle_left(self) -> bool:
        """Check if obstacle on left side"""
        left_angle = self.state.theta + math.pi / 2
        check_dist = 0.15  # meters
        check_x = self.state.x + check_dist * math.cos(left_angle)
        check_y = self.state.y + check_dist * math.sin(left_angle)
        
        for obs_x, obs_y, obs_radius in self.obstacles:
            dist = math.sqrt((check_x - obs_x)**2 + (check_y - obs_y)**2)
            if dist < obs_radius + self.robot_radius:
                return True
        return False
    
    def _check_obstacle_right(self) -> bool:
        """Check if obstacle on right side"""
        right_angle = self.state.theta - math.pi / 2
        check_dist = 0.15  # meters
        check_x = self.state.x + check_dist * math.cos(right_angle)
        check_y = self.state.y + check_dist * math.sin(right_angle)
        
        for obs_x, obs_y, obs_radius in self.obstacles:
            dist = math.sqrt((check_x - obs_x)**2 + (check_y - obs_y)**2)
            if dist < obs_radius + self.robot_radius:
                return True
        return False
    
    def _point_to_line_distance(self,
                                line_start: Tuple[float, float],
                                line_end: Tuple[float, float],
                                point: Tuple[float, float]) -> float:
        """Calculate distance from point to line segment"""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx*dx + dy*dy
        
        if length_sq == 0:
            return math.sqrt((x0-x1)**2 + (y0-y1)**2)
        
        t = max(0, min(1, ((x0-x1)*dx + (y0-y1)*dy) / length_sq))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return math.sqrt((x0-closest_x)**2 + (y0-closest_y)**2)
    
    def check_line_crossing(self, line_index: int) -> bool:
        """
        Check if robot has crossed a specific line
        
        Uses line segment intersection to detect actual crossing.
        
        Args:
            line_index: Index of line to check (0-based)
            
        Returns:
            True if robot has crossed the line
        """
        if line_index >= len(self.lines):
            return False
        
        x1, y1, x2, y2 = self.lines[line_index]
        
        # Check if robot path (from previous to current position) intersects the line
        # Using line segment intersection algorithm
        
        # Robot path segment
        r1_x, r1_y = self.prev_x, self.prev_y
        r2_x, r2_y = self.state.x, self.state.y
        
        # Line segment
        l1_x, l1_y = x1, y1
        l2_x, l2_y = x2, y2
        
        # Check for intersection
        def ccw(A, B, C):
            """Counter-clockwise test"""
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        
        def segments_intersect(p1, p2, p3, p4):
            """Check if two line segments intersect"""
            return (ccw(p1, p3, p4) != ccw(p2, p3, p4) and 
                   ccw(p1, p2, p3) != ccw(p1, p2, p4))
        
        # Check intersection
        if segments_intersect((r1_x, r1_y), (r2_x, r2_y), 
                              (l1_x, l1_y), (l2_x, l2_y)):
            return True
        
        # Also check if robot is very close to line (within threshold)
        # This handles cases where robot might "jump over" line in one update
        threshold = 0.05  # 5cm
        dist = self._point_to_line_distance(
            (l1_x, l1_y), (l2_x, l2_y),
            (self.state.x, self.state.y)
        )
        
        if dist < threshold:
            # Check if we're on the "target side" of the line
            # For horizontal lines, check if y is past line
            if abs(y2 - y1) > abs(x2 - x1):  # More vertical
                line_x = (x1 + x2) / 2
                if x1 < x2:  # Line goes left to right
                    return self.state.x > line_x
                else:
                    return self.state.x < line_x
            else:  # More horizontal
                line_y = (y1 + y2) / 2
                if y1 < y2:  # Line goes bottom to top
                    return self.state.y > line_y
                else:
                    return self.state.y < line_y
        
        return False
    
    def get_position(self) -> Tuple[float, float]:
        """Get current robot position"""
        return (self.state.x, self.state.y)
    
    def get_orientation(self) -> float:
        """Get current robot orientation"""
        return self.state.theta
    
    def get_state(self) -> SimulatedRobotState:
        """Get current robot state"""
        return self.state


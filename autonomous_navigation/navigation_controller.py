"""
Navigation Controller Module

Controls robot movement using PID control and state machine.
Manages navigation states and sensor fusion.
"""

import math
from typing import Optional, Tuple
from enum import Enum
from dataclasses import dataclass
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from robot_code.python_client.galaxyrvr import GalaxyRVR


class NavigationState(Enum):
    """Navigation state machine states"""
    IDLE = "idle"
    SEARCHING = "searching"
    TRACKING = "tracking"
    NAVIGATING = "navigating"
    AVOIDING = "avoiding"
    ARRIVED = "arrived"
    RETURNING = "returning"
    ERROR = "error"


@dataclass
class ControlCommand:
    """Robot control command"""
    left_motor: int  # -100 to 100
    right_motor: int  # -100 to 100
    servo_angle: int  # 0 to 180


class PIDController:
    """Simple PID controller for heading control"""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
    
    def compute(self, error: float, dt: float = 0.1) -> float:
        """
        Compute PID output
        
        Args:
            error: Current error
            dt: Time step
            
        Returns:
            Control output
        """
        # Proportional
        p = self.kp * error
        
        # Integral
        self.integral += error * dt
        i = self.ki * self.integral
        
        # Derivative
        d = self.kd * (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        
        return p + i + d
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.last_error = 0.0


class NavigationController:
    """Controls robot navigation with state machine and PID control"""
    
    def __init__(self, robot: GalaxyRVR,
                 max_speed: int = 60,
                 min_speed: int = 30,
                 arrival_threshold: float = 0.15,  # meters
                 obstacle_threshold: float = 0.20):  # meters
        """
        Initialize navigation controller
        
        Args:
            robot: GalaxyRVR robot instance
            max_speed: Maximum motor speed (0-100)
            min_speed: Minimum motor speed (0-100)
            arrival_threshold: Distance threshold for arrival (meters)
            obstacle_threshold: Distance threshold for obstacle avoidance (meters)
        """
        self.robot = robot
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.arrival_threshold = arrival_threshold
        self.obstacle_threshold = obstacle_threshold
        
        # State machine
        self.state = NavigationState.IDLE
        self.target_position: Optional[Tuple[float, float]] = None
        self.current_waypoint: Optional[Tuple[float, float]] = None
        
        # PID controllers
        self.heading_pid = PIDController(kp=2.0, ki=0.0, kd=0.5)
        self.distance_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
        
        # Sensor thresholds
        self.ultrasonic_threshold = 20.0  # cm
        self.ir_threshold = 1  # obstacle detected
        
    def set_target(self, target: Tuple[float, float]):
        """Set target position"""
        self.target_position = target
        if self.state == NavigationState.IDLE:
            self.state = NavigationState.SEARCHING
    
    def set_waypoint(self, waypoint: Tuple[float, float]):
        """Set current waypoint"""
        self.current_waypoint = waypoint
        if self.state == NavigationState.SEARCHING:
            self.state = NavigationState.TRACKING
    
    def update(self, 
              current_pos: Tuple[float, float],
              current_theta: float,
              dt: float = 0.1) -> ControlCommand:
        """
        Update navigation controller
        
        Args:
            current_pos: Current robot position (x, y)
            current_theta: Current robot orientation (radians)
            dt: Time step
            
        Returns:
            ControlCommand for robot
        """
        # Check for obstacles first
        if self.check_obstacles():
            if self.state != NavigationState.AVOIDING:
                self.state = NavigationState.AVOIDING
            return self.avoid_obstacle()
        
        # State machine logic
        if self.state == NavigationState.IDLE:
            return ControlCommand(0, 0, 90)
        
        elif self.state == NavigationState.SEARCHING:
            # Rotate in place to search for target
            return ControlCommand(-30, 30, 90)  # Turn left
        
        elif self.state == NavigationState.TRACKING:
            # Track target, prepare to navigate
            if self.current_waypoint:
                self.state = NavigationState.NAVIGATING
            return ControlCommand(0, 0, 90)
        
        elif self.state == NavigationState.NAVIGATING:
            if not self.current_waypoint:
                self.state = NavigationState.IDLE
                return ControlCommand(0, 0, 90)
            
            # Navigate to waypoint
            return self.navigate_to_waypoint(current_pos, current_theta, dt)
        
        elif self.state == NavigationState.AVOIDING:
            # Continue avoiding
            return self.avoid_obstacle()
        
        elif self.state == NavigationState.ARRIVED:
            # Stop at target
            return ControlCommand(0, 0, 90)
        
        elif self.state == NavigationState.RETURNING:
            # Return to start (similar to navigating)
            if self.current_waypoint:
                return self.navigate_to_waypoint(current_pos, current_theta, dt)
            return ControlCommand(0, 0, 90)
        
        else:  # ERROR
            return ControlCommand(0, 0, 90)
    
    def navigate_to_waypoint(self,
                            current_pos: Tuple[float, float],
                            current_theta: float,
                            dt: float) -> ControlCommand:
        """
        Navigate to current waypoint using PID control
        
        Args:
            current_pos: Current robot position
            current_theta: Current robot orientation
            dt: Time step
            
        Returns:
            ControlCommand
        """
        if not self.current_waypoint:
            return ControlCommand(0, 0, 90)
        
        # Calculate distance and angle to waypoint
        dx = self.current_waypoint[0] - current_pos[0]
        dy = self.current_waypoint[1] - current_pos[1]
        
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Check if arrived
        if distance < self.arrival_threshold:
            self.state = NavigationState.ARRIVED
            if self.target_position and distance < self.arrival_threshold:
                # Check if at final target
                target_dx = self.target_position[0] - current_pos[0]
                target_dy = self.target_position[1] - current_pos[1]
                target_dist = math.sqrt(target_dx*target_dx + target_dy*target_dy)
                if target_dist < self.arrival_threshold:
                    self.state = NavigationState.ARRIVED
            return ControlCommand(0, 0, 90)
        
        # Calculate heading error (normalize to [-pi, pi])
        heading_error = target_angle - current_theta
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # PID control for heading
        heading_correction = self.heading_pid.compute(heading_error, dt)
        
        # Speed control based on distance
        base_speed = min(self.max_speed, max(self.min_speed, 
                                            int(distance * 50)))  # Scale factor
        
        # Differential drive control
        # Positive heading_correction means turn right
        left_speed = int(base_speed - heading_correction)
        right_speed = int(base_speed + heading_correction)
        
        # Clamp speeds
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        return ControlCommand(left_speed, right_speed, 90)
    
    def check_obstacles(self) -> bool:
        """
        Check if obstacles are detected
        
        Returns:
            True if obstacle detected
        """
        # Check ultrasonic sensor
        if self.robot.ultrasonic_distance is not None:
            if self.robot.ultrasonic_distance < self.ultrasonic_threshold:
                return True
        
        # Check IR sensors
        if self.robot.ir_left == 1 or self.robot.ir_right == 1:
            return True
        
        return False
    
    def avoid_obstacle(self) -> ControlCommand:
        """
        Obstacle avoidance behavior
        
        Returns:
            ControlCommand for avoidance
        """
        # Simple obstacle avoidance: turn away from obstacle
        ir_left = self.robot.ir_left == 1 if self.robot.ir_left is not None else False
        ir_right = self.robot.ir_right == 1 if self.robot.ir_right is not None else False
        
        if ir_left and ir_right:
            # Obstacle on both sides, back up and turn
            return ControlCommand(-30, -30, 90)
        elif ir_left:
            # Obstacle on left, turn right
            return ControlCommand(40, -40, 90)
        elif ir_right:
            # Obstacle on right, turn left
            return ControlCommand(-40, 40, 90)
        else:
            # Obstacle ahead (ultrasonic), turn based on IR or default right
            if self.robot.ultrasonic_distance and self.robot.ultrasonic_distance < self.ultrasonic_threshold:
                return ControlCommand(40, -40, 90)  # Turn right
        
        # Default: stop
        return ControlCommand(0, 0, 90)
    
    def reset(self):
        """Reset controller state"""
        self.state = NavigationState.IDLE
        self.target_position = None
        self.current_waypoint = None
        self.heading_pid.reset()
        self.distance_pid.reset()
    
    def get_state(self) -> NavigationState:
        """Get current navigation state"""
        return self.state
    
    def is_arrived(self) -> bool:
        """Check if robot has arrived at target"""
        return self.state == NavigationState.ARRIVED


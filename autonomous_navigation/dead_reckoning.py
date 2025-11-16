"""
Dead Reckoning / Odometry Module

Estimates robot position between camera updates using motor commands and time.
Handles webcam delay by providing continuous position estimates.
"""

import numpy as np
import time
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class OdometryState:
    """Odometry state for dead reckoning"""
    x: float  # X position in meters
    y: float  # Y position in meters
    theta: float  # Orientation in radians
    last_update_time: float  # Last update timestamp
    last_left_motor: int = 0  # Last left motor command
    last_right_motor: int = 0  # Last right motor command


class DeadReckoning:
    """
    Dead reckoning / odometry estimator
    
    Estimates robot position using:
    - Motor commands (left/right speeds)
    - Time elapsed
    - Differential drive kinematics
    """
    
    def __init__(self,
                 wheel_base: float = 0.12,  # Distance between wheels (meters)
                 wheel_radius: float = 0.03,  # Wheel radius (meters)
                 max_speed: float = 0.5):  # Maximum speed (m/s)
        """
        Initialize dead reckoning
        
        Args:
            wheel_base: Distance between wheels in meters
            wheel_radius: Wheel radius in meters
            max_speed: Maximum linear speed in m/s
        """
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.max_speed = max_speed
        
        # Odometry state
        self.odometry: Optional[OdometryState] = None
        
        # Calibration from camera (for initialization and correction)
        self.camera_pose: Optional[Tuple[float, float, float]] = None
        self.last_camera_update: float = 0
        self.camera_update_interval: float = 10.0  # Expected camera update interval (seconds)
    
    def initialize(self, initial_pos: Tuple[float, float], 
                   initial_theta: float = 0.0):
        """
        Initialize odometry with known position
        
        Args:
            initial_pos: (x, y) initial position
            initial_theta: Initial orientation
        """
        self.odometry = OdometryState(
            x=initial_pos[0],
            y=initial_pos[1],
            theta=initial_theta,
            last_update_time=time.time()
        )
        self.camera_pose = (initial_pos[0], initial_pos[1], initial_theta)
        self.last_camera_update = time.time()
    
    def update_from_camera(self, camera_pos: Tuple[float, float],
                          camera_theta: float):
        """
        Update odometry with camera measurement (ground truth)
        
        Args:
            camera_pos: (x, y) position from camera
            camera_theta: Orientation from camera
        """
        current_time = time.time()
        
        if self.odometry is None:
            self.initialize(camera_pos, camera_theta)
            return
        
        # Update camera pose
        self.camera_pose = (camera_pos[0], camera_pos[1], camera_theta)
        self.last_camera_update = current_time
        
        # Correct odometry with camera measurement
        # Use weighted average: trust camera more if it's been a while
        time_since_update = current_time - self.last_camera_update
        camera_weight = min(1.0, time_since_update / self.camera_update_interval)
        
        # Blend odometry estimate with camera measurement
        self.odometry.x = (1 - camera_weight) * self.odometry.x + camera_weight * camera_pos[0]
        self.odometry.y = (1 - camera_weight) * self.odometry.y + camera_weight * camera_pos[1]
        self.odometry.theta = (1 - camera_weight) * self.odometry.theta + camera_weight * camera_theta
        
        # Reset update time
        self.odometry.last_update_time = current_time
    
    def update_from_motors(self, left_motor: int, right_motor: int):
        """
        Update odometry estimate based on motor commands
        
        Args:
            left_motor: Left motor speed (-100 to 100)
            right_motor: Right motor speed (-100 to 100)
        """
        if self.odometry is None:
            return
        
        current_time = time.time()
        dt = current_time - self.odometry.last_update_time
        
        if dt <= 0:
            return
        
        # Convert motor speeds to wheel speeds (m/s)
        left_speed = (left_motor / 100.0) * self.max_speed
        right_speed = (right_motor / 100.0) * self.max_speed
        
        # Differential drive kinematics
        # Linear velocity
        v = (left_speed + right_speed) / 2.0
        
        # Angular velocity
        omega = (right_speed - left_speed) / self.wheel_base
        
        # Update position using unicycle model
        if abs(omega) < 0.001:  # Straight line motion
            dx = v * np.cos(self.odometry.theta) * dt
            dy = v * np.sin(self.odometry.theta) * dt
            dtheta = 0
        else:  # Curved motion
            # ICC (Instantaneous Center of Curvature)
            radius = v / omega if omega != 0 else float('inf')
            icc_x = self.odometry.x - radius * np.sin(self.odometry.theta)
            icc_y = self.odometry.y + radius * np.cos(self.odometry.theta)
            
            # Rotation matrix
            cos_omega_dt = np.cos(omega * dt)
            sin_omega_dt = np.sin(omega * dt)
            
            # New position
            new_x = (self.odometry.x - icc_x) * cos_omega_dt - (self.odometry.y - icc_y) * sin_omega_dt + icc_x
            new_y = (self.odometry.x - icc_x) * sin_omega_dt + (self.odometry.y - icc_y) * cos_omega_dt + icc_y
            dx = new_x - self.odometry.x
            dy = new_y - self.odometry.y
            dtheta = omega * dt
        
        # Update state
        self.odometry.x += dx
        self.odometry.y += dy
        self.odometry.theta += dtheta
        
        # Normalize theta to [-pi, pi]
        while self.odometry.theta > np.pi:
            self.odometry.theta -= 2 * np.pi
        while self.odometry.theta < -np.pi:
            self.odometry.theta += 2 * np.pi
        
        # Update motor commands
        self.odometry.last_left_motor = left_motor
        self.odometry.last_right_motor = right_motor
        self.odometry.last_update_time = current_time
    
    def get_estimated_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        Get current estimated pose from dead reckoning
        
        Returns:
            (x, y, theta) estimated pose, or None if not initialized
        """
        if self.odometry is None:
            return None
        
        return (self.odometry.x, self.odometry.y, self.odometry.theta)
    
    def get_confidence(self) -> float:
        """
        Get confidence in odometry estimate
        
        Returns:
            Confidence value (0-1), lower if camera hasn't updated recently
        """
        if self.odometry is None:
            return 0.0
        
        time_since_camera = time.time() - self.last_camera_update
        
        # Confidence decreases over time without camera updates
        # After 10 seconds, confidence is 0.5
        # After 20 seconds, confidence is 0.25
        confidence = 1.0 / (1.0 + time_since_camera / 10.0)
        
        return max(0.1, min(1.0, confidence))
    
    def reset(self):
        """Reset odometry state"""
        self.odometry = None
        self.camera_pose = None
        self.last_camera_update = 0


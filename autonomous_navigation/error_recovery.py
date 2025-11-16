"""
Error Recovery System

Handles navigation errors and implements recovery behaviors.
"""

import time
import math
from typing import Optional, Tuple, List
from enum import Enum
from dataclasses import dataclass

# Use relative imports when used as module
try:
    from .navigation_controller import NavigationState
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(__file__)))
    from autonomous_navigation.navigation_controller import NavigationState


class ErrorType(Enum):
    """Types of navigation errors"""
    STUCK = "stuck"  # Robot not making progress
    LOST = "lost"    # Robot lost localization
    OBSTACLE_BLOCKED = "obstacle_blocked"  # Completely blocked by obstacles
    TIMEOUT = "timeout"  # Operation taking too long
    SENSOR_FAILURE = "sensor_failure"  # Sensor readings invalid


@dataclass
class ErrorState:
    """Current error state"""
    error_type: Optional[ErrorType] = None
    detected_at: float = 0.0
    recovery_attempts: int = 0
    last_position: Optional[Tuple[float, float]] = None
    stuck_threshold: float = 0.1  # meters - movement threshold
    stuck_time: float = 5.0  # seconds - time before considered stuck


class ErrorRecovery:
    """
    Error detection and recovery system
    
    Detects common navigation errors and implements recovery behaviors.
    """
    
    def __init__(self,
                 stuck_threshold: float = 0.1,  # meters
                 stuck_time: float = 5.0,  # seconds
                 timeout_threshold: float = 30.0):  # seconds
        """
        Initialize error recovery system
        
        Args:
            stuck_threshold: Minimum movement distance to not be considered stuck (meters)
            stuck_time: Time without movement before considered stuck (seconds)
            timeout_threshold: Maximum time for operation before timeout (seconds)
        """
        self.stuck_threshold = stuck_threshold
        self.stuck_time = stuck_time
        self.timeout_threshold = timeout_threshold
        
        self.error_state = ErrorState(
            stuck_threshold=stuck_threshold,
            stuck_time=stuck_time
        )
        
        # Position history for stuck detection
        self.position_history: List[Tuple[float, float, float]] = []  # (x, y, time)
        self.max_history = 50
        
        # Recovery behaviors
        self.recovery_behaviors = {
            ErrorType.STUCK: self._recover_from_stuck,
            ErrorType.LOST: self._recover_from_lost,
            ErrorType.OBSTACLE_BLOCKED: self._recover_from_blocked,
            ErrorType.TIMEOUT: self._recover_from_timeout,
            ErrorType.SENSOR_FAILURE: self._recover_from_sensor_failure
        }
    
    def update_position(self, position: Tuple[float, float]):
        """
        Update robot position for stuck detection
        
        Args:
            position: Current robot position (x, y)
        """
        current_time = time.time()
        self.position_history.append((position[0], position[1], current_time))
        
        # Keep history limited
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
        
        # Update last position in error state
        self.error_state.last_position = position
    
    def detect_errors(self,
                      current_position: Optional[Tuple[float, float]],
                      current_state: NavigationState,
                      operation_start_time: Optional[float] = None) -> Optional[ErrorType]:
        """
        Detect navigation errors
        
        Args:
            current_position: Current robot position
            current_state: Current navigation state
            operation_start_time: When current operation started
            
        Returns:
            Detected error type, or None if no error
        """
        current_time = time.time()
        
        # Check for lost localization
        if current_position is None:
            if self.error_state.error_type != ErrorType.LOST:
                self.error_state.error_type = ErrorType.LOST
                self.error_state.detected_at = current_time
                self.error_state.recovery_attempts = 0
            return ErrorType.LOST
        
        # Check for stuck
        if self._is_stuck(current_position, current_time):
            if self.error_state.error_type != ErrorType.STUCK:
                self.error_state.error_type = ErrorType.STUCK
                self.error_state.detected_at = current_time
                self.error_state.recovery_attempts = 0
            return ErrorType.STUCK
        
        # Check for timeout
        if operation_start_time:
            elapsed = current_time - operation_start_time
            if elapsed > self.timeout_threshold:
                if self.error_state.error_type != ErrorType.TIMEOUT:
                    self.error_state.error_type = ErrorType.TIMEOUT
                    self.error_state.detected_at = current_time
                    self.error_state.recovery_attempts = 0
                return ErrorType.TIMEOUT
        
        # Clear error if conditions are met
        if self.error_state.error_type:
            # Check if error is resolved
            if self._is_error_resolved(current_position, current_time):
                self.error_state.error_type = None
                self.error_state.recovery_attempts = 0
        
        return self.error_state.error_type
    
    def _is_stuck(self, current_position: Tuple[float, float], current_time: float) -> bool:
        """Check if robot is stuck"""
        if len(self.position_history) < 5:
            return False
        
        # Check movement over last stuck_time seconds
        cutoff_time = current_time - self.stuck_time
        recent_positions = [
            (x, y) for x, y, t in self.position_history
            if t >= cutoff_time
        ]
        
        if len(recent_positions) < 3:
            return False
        
        # Calculate total distance traveled
        total_distance = 0.0
        for i in range(1, len(recent_positions)):
            dx = recent_positions[i][0] - recent_positions[i-1][0]
            dy = recent_positions[i][1] - recent_positions[i-1][1]
            total_distance += math.sqrt(dx*dx + dy*dy)
        
        # Check if movement is below threshold
        return total_distance < self.stuck_threshold
    
    def _is_error_resolved(self, current_position: Tuple[float, float], current_time: float) -> bool:
        """Check if current error is resolved"""
        if not self.error_state.error_type:
            return True
        
        elapsed = current_time - self.error_state.detected_at
        
        if self.error_state.error_type == ErrorType.LOST:
            # Resolved if we have position again
            return current_position is not None
        
        elif self.error_state.error_type == ErrorType.STUCK:
            # Resolved if we've moved significantly
            if self.error_state.last_position:
                dx = current_position[0] - self.error_state.last_position[0]
                dy = current_position[1] - self.error_state.last_position[1]
                distance = math.sqrt(dx*dx + dy*dy)
                return distance > self.stuck_threshold * 2
        
        elif self.error_state.error_type == ErrorType.TIMEOUT:
            # Resolved if operation completes (checked externally)
            return False  # Timeout resolution handled by operation completion
        
        return False
    
    def get_recovery_command(self,
                            current_position: Optional[Tuple[float, float]],
                            current_theta: float) -> Tuple[int, int]:
        """
        Get recovery motor command based on error type
        
        Args:
            current_position: Current robot position
            current_theta: Current robot orientation
            
        Returns:
            (left_motor, right_motor) recovery command
        """
        if not self.error_state.error_type:
            return (0, 0)  # No error, no recovery needed
        
        recovery_func = self.recovery_behaviors.get(self.error_state.error_type)
        if recovery_func:
            self.error_state.recovery_attempts += 1
            return recovery_func(current_position, current_theta)
        
        return (0, 0)
    
    def _recover_from_stuck(self,
                           current_position: Optional[Tuple[float, float]],
                           current_theta: float) -> Tuple[int, int]:
        """Recovery behavior for stuck robot"""
        # Try backing up and turning
        attempts = self.error_state.recovery_attempts
        
        if attempts % 3 == 1:
            # Back up
            return (-30, -30)
        elif attempts % 3 == 2:
            # Turn left
            return (-50, 50)
        else:
            # Turn right
            return (50, -50)
    
    def _recover_from_lost(self,
                          current_position: Optional[Tuple[float, float]],
                          current_theta: float) -> Tuple[int, int]:
        """Recovery behavior for lost localization"""
        # Rotate in place to help camera find robot
        return (50, -50)  # Rotate left
    
    def _recover_from_blocked(self,
                             current_position: Optional[Tuple[float, float]],
                             current_theta: float) -> Tuple[int, int]:
        """Recovery behavior for blocked path"""
        # Back up and try different direction
        attempts = self.error_state.recovery_attempts
        
        if attempts % 2 == 1:
            return (-40, -40)  # Back up
        else:
            return (50, -50)  # Turn left
    
    def _recover_from_timeout(self,
                             current_position: Optional[Tuple[float, float]],
                             current_theta: float) -> Tuple[int, int]:
        """Recovery behavior for timeout"""
        # Stop and reassess
        return (0, 0)
    
    def _recover_from_sensor_failure(self,
                                    current_position: Optional[Tuple[float, float]],
                                    current_theta: float) -> Tuple[int, int]:
        """Recovery behavior for sensor failure"""
        # Stop for safety
        return (0, 0)
    
    def reset(self):
        """Reset error state"""
        self.error_state = ErrorState(
            stuck_threshold=self.stuck_threshold,
            stuck_time=self.stuck_time
        )
        self.position_history = []


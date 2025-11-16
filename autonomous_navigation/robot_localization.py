"""
Robot Localization Module

Tracks robot position in the arena using overhead camera and coordinate transformation.
Supports marker-based and tracking-based localization.
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from webcam_code.arena_transform import detect_red_corners


@dataclass
class RobotPose:
    """Represents robot position and orientation"""
    x: float  # X position in world coordinates (meters)
    y: float  # Y position in world coordinates (meters)
    theta: float  # Orientation in radians (0 = facing positive X)
    confidence: float  # Localization confidence (0-1)


class RobotLocalizer:
    """Localizes robot position in the arena"""
    
    def __init__(self, 
                 world_transform_matrix: Optional[np.ndarray] = None,
                 robot_marker_color: str = 'blue',
                 robot_size: float = 0.15):  # Robot size in meters
        """
        Initialize robot localizer
        
        Args:
            world_transform_matrix: 3x3 matrix for pixel to world coordinate transform
            robot_marker_color: Color of robot marker for detection
            robot_size: Size of robot in meters (for size-based filtering)
        """
        self.world_transform_matrix = world_transform_matrix
        self.robot_marker_color = robot_marker_color
        self.robot_size = robot_size
        
        # Position history for smoothing
        self.position_history: List[Tuple[float, float]] = []
        self.max_history = 10
        
        # Kalman filter parameters (simple moving average for now)
        self.alpha = 0.7  # Smoothing factor
        
        # Last known position
        self.last_pose: Optional[RobotPose] = None
        
    def detect_robot_marker(self, image: np.ndarray) -> Optional[Tuple[float, float]]:
        """
        Detect robot using colored marker
        
        Args:
            image: Input image (BGR format)
            
        Returns:
            (x, y) pixel coordinates of robot center, or None if not found
        """
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color range for robot marker
        if self.robot_marker_color == 'blue':
            lower = np.array([100, 50, 50])
            upper = np.array([130, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
        elif self.robot_marker_color == 'green':
            # Dark green range (for robot's green upper half)
            # More specific range for dark green
            lower = np.array([35, 40, 40])  # Lower saturation/value for dark green
            upper = np.array([85, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
        elif self.robot_marker_color == 'red':
            lower1 = np.array([0, 50, 50])
            upper1 = np.array([10, 255, 255])
            lower2 = np.array([170, 50, 50])
            upper2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower1, upper1)
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            # Default to blue
            lower = np.array([100, 50, 50])
            upper = np.array([130, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                      cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest contour (most likely robot)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filter by size (robot should be roughly robot_size in pixels)
        # This is approximate - adjust based on your setup
        min_area = 50
        max_area = 5000
        
        if min_area <= area <= max_area:
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                return (cx, cy)
        
        return None
    
    def estimate_orientation(self, image: np.ndarray, 
                           robot_center: Tuple[float, float]) -> float:
        """
        Estimate robot orientation using marker or shape
        
        Args:
            image: Input image
            robot_center: Robot center position in pixels
            
        Returns:
            Orientation angle in radians
        """
        # Simple approach: detect a front marker or use shape
        # For now, return 0 (facing positive X)
        # Can be enhanced with directional marker detection
        
        cx, cy = robot_center
        
        # Try to detect a front marker (different color or shape)
        # This is a placeholder - implement based on your robot's markers
        return 0.0
    
    def pixel_to_world(self, pixel_pos: Tuple[float, float]) -> Tuple[float, float]:
        """
        Convert pixel coordinates to world coordinates
        
        Args:
            pixel_pos: (x, y) in pixel coordinates
            
        Returns:
            (x, y) in world coordinates (meters)
        """
        if self.world_transform_matrix is None:
            # No transform available, return pixel coordinates
            return pixel_pos
        
        px, py = pixel_pos
        point = np.array([px, py, 1.0])
        world_point = self.world_transform_matrix @ point
        
        return (float(world_point[0]), float(world_point[1]))
    
    def localize(self, image: np.ndarray) -> Optional[RobotPose]:
        """
        Localize robot in the arena
        
        Args:
            image: Input image from overhead camera
            
        Returns:
            RobotPose object, or None if robot not found
        """
        # Detect robot marker
        robot_pixel = self.detect_robot_marker(image)
        
        if robot_pixel is None:
            # Try to use last known position with lower confidence
            if self.last_pose is not None:
                pose = RobotPose(
                    x=self.last_pose.x,
                    y=self.last_pose.y,
                    theta=self.last_pose.theta,
                    confidence=0.3  # Low confidence
                )
                return pose
            return None
        
        # Convert to world coordinates
        world_pos = self.pixel_to_world(robot_pixel)
        
        # Estimate orientation
        theta = self.estimate_orientation(image, robot_pixel)
        
        # Smooth position using history
        if self.last_pose is not None:
            # Simple exponential smoothing
            smoothed_x = self.alpha * world_pos[0] + (1 - self.alpha) * self.last_pose.x
            smoothed_y = self.alpha * world_pos[1] + (1 - self.alpha) * self.last_pose.y
            world_pos = (smoothed_x, smoothed_y)
        
        # Update history
        self.position_history.append(world_pos)
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
        
        # Calculate confidence based on detection quality
        confidence = 0.9  # High confidence if detected
        
        pose = RobotPose(
            x=world_pos[0],
            y=world_pos[1],
            theta=theta,
            confidence=confidence
        )
        
        self.last_pose = pose
        return pose
    
    def set_world_transform(self, transform_matrix: np.ndarray):
        """
        Set the world coordinate transformation matrix
        
        Args:
            transform_matrix: 3x3 transformation matrix
        """
        self.world_transform_matrix = transform_matrix
    
    def get_smoothed_position(self) -> Optional[Tuple[float, float]]:
        """
        Get smoothed position from history
        
        Returns:
            (x, y) world coordinates, or None if no history
        """
        if not self.position_history:
            return None
        
        # Simple average
        avg_x = sum(p[0] for p in self.position_history) / len(self.position_history)
        avg_y = sum(p[1] for p in self.position_history) / len(self.position_history)
        
        return (avg_x, avg_y)
    
    def visualize_pose(self, image: np.ndarray, pose: RobotPose) -> np.ndarray:
        """
        Draw robot pose on image
        
        Args:
            image: Input image
            pose: Robot pose
            
        Returns:
            Image with robot pose visualized
        """
        vis_image = image.copy()
        
        # Convert world coordinates back to pixel (if transform available)
        if self.world_transform_matrix is not None:
            # Inverse transform (approximate)
            inv_matrix = np.linalg.pinv(self.world_transform_matrix)
            world_point = np.array([pose.x, pose.y, 1.0])
            pixel_point = inv_matrix @ world_point
            px, py = int(pixel_point[0]), int(pixel_point[1])
        else:
            # Use last detected pixel position
            if self.last_pose:
                px, py = 0, 0  # Placeholder
            else:
                return vis_image
        
        # Draw robot position
        cv2.circle(vis_image, (px, py), 10, (255, 0, 0), -1)
        
        # Draw orientation arrow
        arrow_length = 20
        end_x = int(px + arrow_length * np.cos(pose.theta))
        end_y = int(py + arrow_length * np.sin(pose.theta))
        cv2.arrowedLine(vis_image, (px, py), (end_x, end_y), (255, 0, 0), 2)
        
        # Draw label
        label = f"Robot: ({pose.x:.2f}, {pose.y:.2f})"
        cv2.putText(vis_image, label, (px + 15, py - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return vis_image


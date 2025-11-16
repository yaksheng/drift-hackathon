"""
Obstacle Detection Module

Detects obstacles from overhead camera feed using color-based detection.
Only detects obstacles that are actually visible in the camera feed.
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from path_planner import Obstacle


@dataclass
class DetectedObstacle:
    """Represents a detected obstacle from camera"""
    center: Tuple[float, float]  # (x, y) in pixel coordinates
    world_pos: Tuple[float, float]  # (x, y) in world coordinates (meters)
    radius: float  # Radius in meters
    area: float  # Area in pixels
    color: str  # Detected color
    confidence: float  # Detection confidence (0-1)


class ObstacleDetector:
    """Detects obstacles from overhead camera using color-based detection"""
    
    def __init__(self,
                 obstacle_colors: Optional[List[str]] = None,
                 min_area: int = 500,
                 max_area: int = 50000):
        """
        Initialize obstacle detector
        
        Args:
            obstacle_colors: List of colors to detect as obstacles (default: all common colors)
            min_area: Minimum contour area to consider as obstacle
            max_area: Maximum contour area to consider as obstacle
        """
        self.min_area = min_area
        self.max_area = max_area
        
        # Default obstacle colors (common obstacle colors)
        if obstacle_colors is None:
            self.obstacle_colors = ['red', 'blue', 'green', 'yellow', 'orange']
        else:
            self.obstacle_colors = obstacle_colors
        
        # HSV color ranges
        self.color_ranges = {
            'blue': ((100, 50, 50), (130, 255, 255)),
            'green': ((40, 50, 50), (80, 255, 255)),
            'yellow': ((20, 50, 50), (30, 255, 255)),
            'red': ((0, 50, 50), (10, 255, 255)),
            'red2': ((170, 50, 50), (180, 255, 255)),
            'orange': ((10, 50, 50), (20, 255, 255)),
        }
    
    def detect_obstacles(self, 
                        image: np.ndarray,
                        transform_matrix: Optional[np.ndarray] = None) -> List[DetectedObstacle]:
        """
        Detect obstacles in overhead camera image
        
        Args:
            image: Input image (BGR format) from overhead camera
            transform_matrix: Optional transformation matrix for world coordinates
            
        Returns:
            List of detected obstacles
        """
        detected_obstacles = []
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detect obstacles for each color
        for color in self.obstacle_colors:
            if color not in self.color_ranges:
                continue
            
            # Get color range
            lower, upper = self.color_ranges[color]
            lower = np.array(lower)
            upper = np.array(upper)
            
            # Create mask
            mask = cv2.inRange(hsv, lower, upper)
            
            # Handle red (wraps around in HSV)
            if color == 'red':
                lower2, upper2 = self.color_ranges.get('red2', (lower, upper))
                mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
                mask = cv2.bitwise_or(mask, mask2)
            
            # Morphological operations to clean up
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                         cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by area
                if area < self.min_area or area > self.max_area:
                    continue
                
                # Get center and radius
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (float(x), float(y))
                
                # Convert to world coordinates if transform available
                world_pos = center
                if transform_matrix is not None:
                    world_pos = self._pixel_to_world(center, transform_matrix)
                
                # Estimate radius in world coordinates (meters)
                # Approximate: assume arena is ~2.5m x 4.0m in world coordinates
                # and image is typically 640x480 or similar
                h, w = image.shape[:2]
                pixel_radius = radius
                # Rough conversion: assume image width represents ~2.5m
                world_radius = (pixel_radius / w) * 2.5
                # Clamp to reasonable range
                world_radius = max(0.05, min(0.5, world_radius))
                
                # Calculate confidence based on area and circularity
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                else:
                    circularity = 0.0
                
                confidence = min(1.0, (area / self.max_area) * 0.5 + circularity * 0.5)
                
                obstacle = DetectedObstacle(
                    center=center,
                    world_pos=world_pos,
                    radius=world_radius,
                    area=area,
                    color=color,
                    confidence=confidence
                )
                detected_obstacles.append(obstacle)
        
        return detected_obstacles
    
    def _pixel_to_world(self, pixel_pos: Tuple[float, float], 
                       transform_matrix: np.ndarray) -> Tuple[float, float]:
        """Convert pixel coordinates to world coordinates"""
        px, py = pixel_pos
        point = np.array([px, py, 1.0])
        world_point = transform_matrix @ point
        return (float(world_point[0]), float(world_point[1]))
    
    def obstacles_to_path_planner(self, 
                                 detected_obstacles: List[DetectedObstacle]) -> List[Obstacle]:
        """
        Convert detected obstacles to PathPlanner Obstacle objects
        
        Args:
            detected_obstacles: List of detected obstacles from camera
            
        Returns:
            List of Obstacle objects for path planner
        """
        obstacles = []
        for det_obs in detected_obstacles:
            obstacle = Obstacle(
                x=det_obs.world_pos[0],
                y=det_obs.world_pos[1],
                radius=det_obs.radius,
                confidence=det_obs.confidence,
                color=det_obs.color
            )
            obstacles.append(obstacle)
        return obstacles


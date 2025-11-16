"""
Line Detection Module

Detects colored lines in the arena, specifically for finding the goal line.
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class DetectedLine:
    """Represents a detected line"""
    start: Tuple[float, float]  # (x, y) start point in pixel coordinates
    end: Tuple[float, float]  # (x, y) end point in pixel coordinates
    center: Tuple[float, float]  # (x, y) center point
    world_start: Tuple[float, float]  # Start in world coordinates
    world_end: Tuple[float, float]  # End in world coordinates
    world_center: Tuple[float, float]  # Center in world coordinates
    length: float  # Length in pixels
    color: str  # Detected color
    confidence: float  # Detection confidence (0-1)


class LineDetector:
    """Detects lines using color and Hough line detection"""
    
    def __init__(self, 
                 line_color: str = 'blue',
                 min_line_length: int = 50,
                 max_line_gap: int = 10):
        """
        Initialize line detector
        
        Args:
            line_color: Color of lines to detect
            min_line_length: Minimum line length in pixels
            max_line_gap: Maximum gap between line segments
        """
        self.line_color = line_color
        self.min_line_length = min_line_length
        self.max_line_gap = max_line_gap
        
        # HSV color ranges
        self.color_ranges = {
            'blue': ((100, 50, 50), (130, 255, 255)),
            'green': ((40, 50, 50), (80, 255, 255)),
            'red': ((0, 50, 50), (10, 255, 255)),
            'red2': ((170, 50, 50), (180, 255, 255)),
        }
    
    def detect_lines(self, image: np.ndarray, 
                    transform_matrix: Optional[np.ndarray] = None) -> List[DetectedLine]:
        """
        Detect lines of specified color in image
        
        Args:
            image: Input image (BGR format)
            transform_matrix: Optional transformation matrix for world coordinates
            
        Returns:
            List of detected lines
        """
        if self.line_color not in self.color_ranges:
            return []
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Get color range
        lower, upper = self.color_ranges[self.line_color]
        lower = np.array(lower)
        upper = np.array(upper)
        
        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Handle red (wraps around in HSV)
        if self.line_color == 'red':
            lower2, upper2 = self.color_ranges.get('red2', (lower, upper))
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask, mask2)
        
        # Morphological operations to clean up
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Use HoughLinesP to detect line segments
        lines = cv2.HoughLinesP(
            mask,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        detected_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Calculate center
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # Calculate length
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                
                # Convert to world coordinates if transform available
                world_start = (x1, y1)
                world_end = (x2, y2)
                world_center = (center_x, center_y)
                
                if transform_matrix is not None:
                    world_start = self._pixel_to_world((x1, y1), transform_matrix)
                    world_end = self._pixel_to_world((x2, y2), transform_matrix)
                    world_center = self._pixel_to_world((center_x, center_y), transform_matrix)
                
                detected_line = DetectedLine(
                    start=(x1, y1),
                    end=(x2, y2),
                    center=(center_x, center_y),
                    world_start=world_start,
                    world_end=world_end,
                    world_center=world_center,
                    length=length,
                    color=self.line_color,
                    confidence=min(1.0, length / 200.0)  # Confidence based on length
                )
                detected_lines.append(detected_line)
        
        return detected_lines
    
    def find_top_line(self, lines: List[DetectedLine]) -> Optional[DetectedLine]:
        """
        Find the line closest to the top of the image (highest Y coordinate)
        
        Args:
            lines: List of detected lines
            
        Returns:
            Line closest to top, or None
        """
        if not lines:
            return None
        
        # Find line with highest Y coordinate (closest to top in image coordinates)
        # In image coordinates, Y=0 is at top
        top_line = min(lines, key=lambda l: l.center[1])
        
        return top_line
    
    def find_middle_line(self, lines: List[DetectedLine]) -> Optional[DetectedLine]:
        """
        Find the line closest to the middle horizontally
        
        Args:
            lines: List of detected lines
            
        Returns:
            Line closest to horizontal center, or None
        """
        if not lines:
            return None
        
        # Find line closest to horizontal center
        # Assuming image width, we'll use the line's center X
        # For now, just return the longest line or first line
        # This can be improved with actual image dimensions
        if len(lines) == 1:
            return lines[0]
        
        # Return line closest to center (we'll need image width for this)
        # For now, return the longest line as it's likely the main goal line
        middle_line = max(lines, key=lambda l: l.length)
        
        return middle_line
    
    def _pixel_to_world(self, pixel_pos: Tuple[float, float], 
                       transform_matrix: np.ndarray) -> Tuple[float, float]:
        """Convert pixel coordinates to world coordinates"""
        px, py = pixel_pos
        point = np.array([px, py, 1.0])
        world_point = transform_matrix @ point
        return (float(world_point[0]), float(world_point[1]))


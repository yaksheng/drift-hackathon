"""
Target Detection Module

Detects and tracks targets/objects in the arena using computer vision.
Supports color-based detection, shape detection, and multiple target tracking.
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass


@dataclass
class Target:
    """Represents a detected target"""
    center: Tuple[float, float]  # (x, y) in pixel coordinates
    world_pos: Tuple[float, float]  # (x, y) in world coordinates (meters)
    area: float  # Area in pixels
    color: str  # Detected color
    confidence: float  # Detection confidence (0-1)
    id: Optional[int] = None  # Tracking ID


class TargetDetector:
    """Detects targets using color and shape analysis"""
    
    def __init__(self, 
                 target_colors: Optional[Dict[str, Tuple]] = None,
                 min_area: int = 100,
                 max_area: int = 50000):
        """
        Initialize target detector
        
        Args:
            target_colors: Dictionary of color names to HSV ranges
                          Format: {'color_name': ((lower_hsv), (upper_hsv))}
            min_area: Minimum contour area to consider as target
            max_area: Maximum contour area to consider as target
        """
        self.min_area = min_area
        self.max_area = max_area
        
        # Default color ranges (HSV)
        if target_colors is None:
            self.target_colors = {
                'blue': ((100, 50, 50), (130, 255, 255)),
                'green': ((40, 50, 50), (80, 255, 255)),
                'yellow': ((20, 50, 50), (30, 255, 255)),
                'red': ((0, 50, 50), (10, 255, 255)),  # Lower red
                'red2': ((170, 50, 50), (180, 255, 255)),  # Upper red
                'orange': ((10, 50, 50), (20, 255, 255)),
            }
        else:
            self.target_colors = target_colors
        
        # Tracking for multiple targets
        self.next_id = 0
        self.tracked_targets: Dict[int, Target] = {}
        self.max_tracking_distance = 50  # pixels
        
    def detect_color_targets(self, image: np.ndarray, 
                           color: str = 'blue') -> List[Target]:
        """
        Detect targets of a specific color
        
        Args:
            image: Input image (BGR format)
            color: Color name to detect
            
        Returns:
            List of detected Target objects
        """
        if color not in self.target_colors:
            return []
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Get color range
        lower, upper = self.target_colors[color]
        lower = np.array(lower)
        upper = np.array(upper)
        
        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Handle red (wraps around in HSV)
        if color == 'red':
            lower2, upper2 = self.target_colors.get('red2', (lower, upper))
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask, mask2)
        
        # Morphological operations to clean up
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                      cv2.CHAIN_APPROX_SIMPLE)
        
        targets = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if self.min_area <= area <= self.max_area:
                # Calculate centroid
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                    
                    # Calculate confidence based on area and shape
                    # Approximate contour to check if it's roughly circular
                    peri = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                    
                    # Circularity check
                    circularity = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
                    confidence = min(1.0, circularity * (area / self.max_area))
                    
                    target = Target(
                        center=(cx, cy),
                        world_pos=(0, 0),  # Will be set by coordinate transform
                        area=area,
                        color=color,
                        confidence=confidence
                    )
                    targets.append(target)
        
        return targets
    
    def detect_all_targets(self, image: np.ndarray) -> List[Target]:
        """
        Detect targets of all configured colors
        
        Args:
            image: Input image (BGR format)
            
        Returns:
            List of all detected Target objects
        """
        all_targets = []
        
        for color in self.target_colors.keys():
            if color == 'red2':  # Skip, handled with 'red'
                continue
            targets = self.detect_color_targets(image, color)
            all_targets.extend(targets)
        
        return all_targets
    
    def detect_shapes(self, image: np.ndarray, 
                     shape_type: str = 'circle') -> List[Target]:
        """
        Detect targets by shape (circle, square, triangle)
        
        Args:
            image: Input image (BGR format)
            shape_type: Type of shape to detect
            
        Returns:
            List of detected Target objects
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply threshold
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, 
                                      cv2.CHAIN_APPROX_SIMPLE)
        
        targets = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if self.min_area <= area <= self.max_area:
                # Approximate contour
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                
                # Check shape
                is_target = False
                if shape_type == 'circle':
                    circularity = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
                    is_target = circularity > 0.7
                elif shape_type == 'square':
                    is_target = len(approx) == 4
                elif shape_type == 'triangle':
                    is_target = len(approx) == 3
                
                if is_target:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = M["m10"] / M["m00"]
                        cy = M["m01"] / M["m00"]
                        
                        target = Target(
                            center=(cx, cy),
                            world_pos=(0, 0),
                            area=area,
                            color='unknown',
                            confidence=0.8
                        )
                        targets.append(target)
        
        return targets
    
    def track_targets(self, new_targets: List[Target]) -> List[Target]:
        """
        Track targets across frames using simple distance-based matching
        
        Args:
            new_targets: List of newly detected targets
            
        Returns:
            List of targets with tracking IDs assigned
        """
        tracked = []
        used_ids = set()
        
        # Match new targets to existing tracked targets
        for new_target in new_targets:
            best_match_id = None
            best_distance = float('inf')
            
            for target_id, old_target in self.tracked_targets.items():
                if target_id in used_ids:
                    continue
                
                # Calculate distance
                dx = new_target.center[0] - old_target.center[0]
                dy = new_target.center[1] - old_target.center[1]
                distance = np.sqrt(dx*dx + dy*dy)
                
                if distance < self.max_tracking_distance and distance < best_distance:
                    best_distance = distance
                    best_match_id = target_id
            
            # Assign ID
            if best_match_id is not None:
                new_target.id = best_match_id
                used_ids.add(best_match_id)
            else:
                new_target.id = self.next_id
                self.next_id += 1
            
            tracked.append(new_target)
            self.tracked_targets[new_target.id] = new_target
        
        # Remove old targets that weren't matched
        self.tracked_targets = {tid: t for tid, t in self.tracked_targets.items() 
                              if tid in used_ids}
        
        return tracked
    
    def convert_to_world_coords(self, targets: List[Target], 
                               transform_matrix: np.ndarray) -> List[Target]:
        """
        Convert pixel coordinates to world coordinates
        
        Args:
            targets: List of targets with pixel coordinates
            transform_matrix: 3x3 transformation matrix from pixel to world
            
        Returns:
            List of targets with world coordinates set
        """
        for target in targets:
            # Convert pixel to world coordinates
            px, py = target.center
            point = np.array([px, py, 1.0])
            world_point = transform_matrix @ point
            
            target.world_pos = (float(world_point[0]), float(world_point[1]))
        
        return targets
    
    def visualize_targets(self, image: np.ndarray, 
                         targets: List[Target]) -> np.ndarray:
        """
        Draw detected targets on image for visualization
        
        Args:
            image: Input image
            targets: List of targets to visualize
            
        Returns:
            Image with targets drawn
        """
        vis_image = image.copy()
        
        for target in targets:
            cx, cy = target.center
            
            # Draw circle
            radius = int(np.sqrt(target.area / np.pi))
            cv2.circle(vis_image, (int(cx), int(cy)), radius, (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(vis_image, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            
            # Draw label
            label = f"{target.color} {target.id if target.id else '?'}"
            if target.confidence > 0:
                label += f" ({target.confidence:.2f})"
            cv2.putText(vis_image, label, (int(cx) + 10, int(cy) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        return vis_image


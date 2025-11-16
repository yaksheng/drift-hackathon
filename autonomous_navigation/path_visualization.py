"""
Path Visualization Module

Overlays robot path and navigation information on camera feed images.
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))


class PathVisualizer:
    """Visualizes path and navigation info on camera images"""
    
    def __init__(self, world_transform: Optional[np.ndarray] = None):
        """
        Initialize path visualizer
        
        Args:
            world_transform: Transformation matrix from world to pixel coordinates
        """
        self.world_transform = world_transform
        self.path_history: List[Tuple[float, float]] = []  # Robot trajectory
        self.planned_path: List[Tuple[float, float]] = []  # Planned waypoints
    
    def world_to_pixel(self, world_pos: Tuple[float, float]) -> Optional[Tuple[int, int]]:
        """
        Convert world coordinates to pixel coordinates
        
        Args:
            world_pos: (x, y) in world coordinates
            
        Returns:
            (x, y) in pixel coordinates, or None if transform not available
        """
        if self.world_transform is None:
            return None
        
        # Use inverse transform
        try:
            inv_matrix = np.linalg.pinv(self.world_transform)
            world_point = np.array([world_pos[0], world_pos[1], 1.0])
            pixel_point = inv_matrix @ world_point
            return (int(pixel_point[0]), int(pixel_point[1]))
        except:
            return None
    
    def overlay_path(self, image: np.ndarray,
                    robot_pos: Optional[Tuple[float, float]] = None,
                    goal_pos: Optional[Tuple[float, float]] = None,
                    waypoints: Optional[List[Tuple[float, float]]] = None) -> np.ndarray:
        """
        Overlay path and navigation info on image
        
        Args:
            image: Input camera image
            robot_pos: Current robot position (world coordinates)
            goal_pos: Goal position (world coordinates)
            waypoints: List of waypoint positions (world coordinates)
            
        Returns:
            Image with path overlaid
        """
        vis_image = image.copy()
        
        # Draw path history (robot trajectory)
        if len(self.path_history) > 1:
            pixel_path = []
            for world_pos in self.path_history:
                pixel_pos = self.world_to_pixel(world_pos)
                if pixel_pos:
                    pixel_path.append(pixel_pos)
            
            if len(pixel_path) > 1:
                # Draw path as green line
                pts = np.array(pixel_path, np.int32)
                cv2.polylines(vis_image, [pts], False, (0, 255, 0), 2)
                
                # Draw path points
                for pt in pixel_path[-20:]:  # Last 20 points
                    cv2.circle(vis_image, pt, 2, (0, 255, 0), -1)
        
        # Draw planned waypoints
        if waypoints:
            for i, waypoint in enumerate(waypoints):
                pixel_pos = self.world_to_pixel(waypoint)
                if pixel_pos:
                    # Draw waypoint
                    cv2.circle(vis_image, pixel_pos, 8, (0, 165, 255), -1)  # Orange
                    cv2.circle(vis_image, pixel_pos, 8, (0, 0, 0), 2)  # Black border
                    # Label
                    cv2.putText(vis_image, f"W{i+1}", 
                               (pixel_pos[0] + 10, pixel_pos[1] - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Draw line from robot to next waypoint
        if robot_pos and waypoints and len(waypoints) > 0:
            robot_px = self.world_to_pixel(robot_pos)
            waypoint_px = self.world_to_pixel(waypoints[0])
            if robot_px and waypoint_px:
                cv2.line(vis_image, robot_px, waypoint_px, (0, 0, 255), 2, cv2.LINE_AA)
        
        # Draw robot position
        if robot_pos:
            robot_px = self.world_to_pixel(robot_pos)
            if robot_px:
                cv2.circle(vis_image, robot_px, 10, (255, 0, 0), -1)  # Blue
                cv2.circle(vis_image, robot_px, 10, (0, 0, 0), 2)  # Black border
                cv2.putText(vis_image, "Robot", 
                           (robot_px[0] + 15, robot_px[1] - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Draw goal position
        if goal_pos:
            goal_px = self.world_to_pixel(goal_pos)
            if goal_px:
                cv2.circle(vis_image, goal_px, 12, (0, 255, 255), -1)  # Yellow
                cv2.circle(vis_image, goal_px, 12, (0, 0, 0), 2)  # Black border
                cv2.putText(vis_image, "GOAL", 
                           (goal_px[0] + 15, goal_px[1] - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return vis_image
    
    def add_path_point(self, world_pos: Tuple[float, float]):
        """Add a point to path history"""
        self.path_history.append(world_pos)
        # Keep last 100 points to avoid memory issues
        if len(self.path_history) > 100:
            self.path_history.pop(0)
    
    def set_planned_path(self, waypoints: List[Tuple[float, float]]):
        """Set planned waypoints"""
        self.planned_path = waypoints
    
    def clear_path(self):
        """Clear path history"""
        self.path_history = []
        self.planned_path = []
    
    def set_world_transform(self, transform_matrix: np.ndarray):
        """Set world coordinate transformation matrix"""
        self.world_transform = transform_matrix


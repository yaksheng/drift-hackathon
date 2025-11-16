"""
Hybrid Vision System

Integrates overhead camera (global view) with onboard camera (local view)
for improved navigation and obstacle detection.
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass
from target_detection import TargetDetector, Target
from robot_localization import RobotLocalizer, RobotPose


@dataclass
class VisionFusion:
    """Result of fusing overhead and onboard vision"""
    robot_pose: Optional[RobotPose] = None
    targets: List[Target] = None
    obstacles: List[Tuple[float, float, float]] = None  # (x, y, radius)
    confidence: float = 0.0  # Overall confidence in fusion result
    overhead_available: bool = False
    onboard_available: bool = False


class HybridVisionSystem:
    """
    Hybrid vision system combining overhead and onboard cameras
    
    Strategy:
    - Overhead camera: Global view, target detection, robot localization
    - Onboard camera: Local obstacle detection, target verification, fine navigation
    """
    
    def __init__(self,
                 target_detector: TargetDetector,
                 robot_localizer: RobotLocalizer,
                 overhead_weight: float = 0.7,  # Weight for overhead camera
                 onboard_weight: float = 0.3):   # Weight for onboard camera
        """
        Initialize hybrid vision system
        
        Args:
            target_detector: Target detection module
            robot_localizer: Robot localization module
            overhead_weight: Weight for overhead camera data (0-1)
            onboard_weight: Weight for onboard camera data (0-1)
        """
        self.target_detector = target_detector
        self.robot_localizer = robot_localizer
        self.overhead_weight = overhead_weight
        self.onboard_weight = onboard_weight
        
        # Onboard camera processing
        self.onboard_target_detector = TargetDetector()  # Separate detector for onboard
        self.onboard_obstacle_detector = None  # Will be initialized when needed
        
    def process_overhead_frame(self, frame: np.ndarray) -> VisionFusion:
        """
        Process overhead camera frame
        
        Args:
            frame: Overhead camera frame
            
        Returns:
            VisionFusion with overhead camera data
        """
        fusion = VisionFusion()
        fusion.overhead_available = True
        
        # Detect targets
        targets = self.target_detector.detect(frame)
        fusion.targets = targets
        
        # Localize robot
        pose = self.robot_localizer.localize(frame)
        fusion.robot_pose = pose
        
        # Calculate confidence based on detections
        if pose and pose.confidence > 0.5:
            fusion.confidence = 0.8
        elif targets:
            fusion.confidence = 0.6
        else:
            fusion.confidence = 0.3
        
        return fusion
    
    def process_onboard_frame(self, frame: np.ndarray) -> VisionFusion:
        """
        Process onboard camera frame for local navigation
        
        Args:
            frame: Onboard camera frame (robot's perspective)
            
        Returns:
            VisionFusion with onboard camera data
        """
        fusion = VisionFusion()
        fusion.onboard_available = True
        
        # Detect targets in local view (for verification)
        targets = self.onboard_target_detector.detect(frame)
        fusion.targets = targets
        
        # Detect obstacles in local view
        obstacles = self.detect_local_obstacles(frame)
        fusion.obstacles = obstacles
        
        # Calculate confidence
        if targets or obstacles:
            fusion.confidence = 0.6
        else:
            fusion.confidence = 0.4
        
        return fusion
    
    def fuse_vision(self,
                   overhead_fusion: Optional[VisionFusion],
                   onboard_fusion: Optional[VisionFusion]) -> VisionFusion:
        """
        Fuse overhead and onboard vision data
        
        Args:
            overhead_fusion: Results from overhead camera
            onboard_fusion: Results from onboard camera
            
        Returns:
            Fused vision data
        """
        fused = VisionFusion()
        
        # Determine availability
        fused.overhead_available = overhead_fusion is not None and overhead_fusion.overhead_available
        fused.onboard_available = onboard_fusion is not None and onboard_fusion.onboard_available
        
        # Fuse robot pose (prefer overhead, use onboard if overhead unavailable)
        if overhead_fusion and overhead_fusion.robot_pose:
            fused.robot_pose = overhead_fusion.robot_pose
            fused.confidence += self.overhead_weight * overhead_fusion.confidence
        elif onboard_fusion:
            # Onboard can't localize globally, but can provide relative info
            fused.confidence += self.onboard_weight * onboard_fusion.confidence
        
        # Fuse targets (combine both sources, prioritize overhead)
        fused.targets = []
        if overhead_fusion and overhead_fusion.targets:
            fused.targets.extend(overhead_fusion.targets)
        if onboard_fusion and onboard_fusion.targets:
            # Add onboard targets that aren't duplicates
            for onboard_target in onboard_fusion.targets:
                is_duplicate = False
                for overhead_target in (overhead_fusion.targets if overhead_fusion else []):
                    # Check if targets are close (same target seen from different views)
                    dist = np.sqrt(
                        (onboard_target.world_pos[0] - overhead_target.world_pos[0])**2 +
                        (onboard_target.world_pos[1] - overhead_target.world_pos[1])**2
                    )
                    if dist < 0.3:  # Within 30cm, consider same target
                        is_duplicate = True
                        break
                if not is_duplicate:
                    fused.targets.append(onboard_target)
        
        # Fuse obstacles (onboard provides local obstacles)
        fused.obstacles = []
        if onboard_fusion and onboard_fusion.obstacles:
            fused.obstacles.extend(onboard_fusion.obstacles)
        
        # Normalize confidence
        fused.confidence = min(1.0, fused.confidence)
        
        return fused
    
    def detect_local_obstacles(self, frame: np.ndarray) -> List[Tuple[float, float, float]]:
        """
        Detect obstacles in onboard camera frame
        
        Uses edge detection and depth estimation to find obstacles
        
        Args:
            frame: Onboard camera frame
            
        Returns:
            List of (x, y, radius) obstacles in local coordinates
        """
        obstacles = []
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by size (likely obstacles)
        h, w = frame.shape[:2]
        min_area = (w * h) * 0.01  # At least 1% of frame
        max_area = (w * h) * 0.3   # At most 30% of frame
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if min_area < area < max_area:
                # Get bounding circle
                (x, y), radius = cv2.minEnclosingCircle(contour)
                
                # Convert to local coordinates (approximate)
                # Assume camera is at front of robot, pointing forward
                # This is a simplified model - in reality would need camera calibration
                local_x = 0.0  # Obstacle is in front
                local_y = (y - h/2) / h * 0.5  # Convert pixel offset to meters (rough estimate)
                local_radius = radius / w * 0.2  # Convert pixel radius to meters
                
                obstacles.append((local_x, local_y, local_radius))
        
        return obstacles
    
    def verify_target_with_onboard(self,
                                  target: Target,
                                  onboard_frame: np.ndarray) -> bool:
        """
        Verify target detected by overhead camera using onboard camera
        
        Args:
            target: Target detected by overhead camera
            onboard_frame: Current onboard camera frame
            
        Returns:
            True if target is verified in onboard view
        """
        # Detect targets in onboard view
        onboard_targets = self.onboard_target_detector.detect(onboard_frame)
        
        # Check if any onboard target matches the overhead target
        for onboard_target in onboard_targets:
            if onboard_target.color == target.color:
                # Target color matches - likely the same target
                return True
        
        return False


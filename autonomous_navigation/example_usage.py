"""
Example Usage of Autonomous Navigation System

This script demonstrates how to use the autonomous navigation modules.
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from autonomous_navigation.main import AutonomousNavigation


async def example_basic_navigation():
    """Basic example of autonomous navigation"""
    
    # Create navigation system
    nav = AutonomousNavigation(
        robot_ip="192.168.1.216",  # Change to your robot IP
        camera_url="http://192.168.1.109:5000/",  # Change to your camera URL
        target_color='blue',  # Color of targets to detect
        robot_marker_color='green',  # Color of robot marker
        use_overhead_camera=True
    )
    
    # Initialize
    if not await nav.initialize():
        print("Failed to initialize!")
        return
    
    try:
        # Run navigation
        if nav.use_overhead_camera and nav.camera_url:
            await nav.run_with_overhead_camera()
        else:
            await nav.run_without_overhead_camera()
    finally:
        await nav.shutdown()


async def example_manual_control():
    """Example of using modules manually"""
    from autonomous_navigation.target_detection import TargetDetector
    from autonomous_navigation.robot_localization import RobotLocalizer
    from autonomous_navigation.path_planner import PathPlanner
    from autonomous_navigation.navigation_controller import NavigationController
    
    import cv2
    import numpy as np
    
    # Initialize modules
    target_detector = TargetDetector()
    robot_localizer = RobotLocalizer()
    path_planner = PathPlanner()
    
    # Load an image (example)
    # image = cv2.imread('arena_image.jpg')
    
    # Detect targets
    # targets = target_detector.detect_all_targets(image)
    
    # Localize robot
    # pose = robot_localizer.localize(image)
    
    # Plan path
    # path_planner.plan_straight_path((0, 0), (2, 2))
    
    print("Manual control example (placeholder)")


if __name__ == "__main__":
    print("=" * 60)
    print("Autonomous Navigation - Example Usage")
    print("=" * 60)
    print("\nChoose an example:")
    print("1. Basic autonomous navigation")
    print("2. Manual module usage")
    
    choice = input("\nEnter choice (1 or 2): ").strip()
    
    if choice == "1":
        asyncio.run(example_basic_navigation())
    elif choice == "2":
        asyncio.run(example_manual_control())
    else:
        print("Invalid choice!")


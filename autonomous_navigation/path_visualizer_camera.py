#!/usr/bin/env python3
"""
Path Visualizer for Camera Feed

Captures current image from camera feed, detects lines and robot,
and draws the best path to the middle of the top three lines.
"""

import cv2
import numpy as np
import requests
import sys
import os
from typing import List, Tuple, Optional

# Add parent directories to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from line_detection import LineDetector, DetectedLine
from robot_localization import RobotLocalizer, RobotPose
from path_planner import PathPlanner, Waypoint, Obstacle
from obstacle_detection import ObstacleDetector


def capture_camera_frame(camera_url: str) -> Optional[np.ndarray]:
    """
    Capture a single frame from the camera stream
    
    Args:
        camera_url: URL of camera stream
        
    Returns:
        Frame as numpy array, or None if failed
    """
    try:
        print(f"📹 Capturing frame from {camera_url}...")
        response = requests.get(camera_url, stream=True, timeout=5)
        
        if response.status_code != 200:
            print(f"❌ Camera returned status {response.status_code}")
            return None
        
        bytes_data = bytes()
        for chunk in response.iter_content(chunk_size=1024):
            bytes_data += chunk
            a = bytes_data.find(b'\xff\xd8')
            b = bytes_data.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes_data[a:b+2]
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    print(f"✅ Captured frame: {frame.shape}")
                    return frame
        
        print("❌ Could not decode frame from camera")
        return None
    except Exception as e:
        print(f"❌ Error capturing frame: {e}")
        return None


def find_top_three_lines(lines: List[DetectedLine]) -> List[DetectedLine]:
    """
    Find the top three lines (highest Y coordinates in image)
    
    Args:
        lines: List of detected lines
        
    Returns:
        Top three lines sorted by Y coordinate (top to bottom)
    """
    if len(lines) < 3:
        return sorted(lines, key=lambda l: l.center[1])[:len(lines)]
    
    # Sort by Y coordinate (lower Y = higher in image)
    sorted_lines = sorted(lines, key=lambda l: l.center[1])
    return sorted_lines[:3]


def find_middle_line_of_top_three(lines: List[DetectedLine]) -> Optional[DetectedLine]:
    """
    Find the middle line of the top three lines
    
    Args:
        lines: Top three lines (should be exactly 3)
        
    Returns:
        The middle line (second line when sorted by Y)
    """
    if len(lines) < 3:
        return None
    
    # Lines are already sorted by Y (top to bottom)
    # Middle line is the second one (index 1)
    return lines[1]


def draw_path_on_image(image: np.ndarray,
                      robot_pos: Tuple[float, float],
                      goal_pos: Tuple[float, float],
                      waypoints: List[Waypoint],
                      obstacles: List[Tuple[float, float, float]],
                      lines: List[DetectedLine],
                      top_three_lines: List[DetectedLine],
                      middle_line: Optional[DetectedLine]) -> np.ndarray:
    """
    Draw path and annotations on the camera image
    
    Args:
        image: Camera frame
        robot_pos: Robot position (pixel coordinates)
        goal_pos: Goal position (pixel coordinates)
        waypoints: List of waypoints
        obstacles: List of obstacles (x, y, radius) in pixel coordinates
        lines: All detected lines
        top_three_lines: Top three lines
        middle_line: The middle line of top three
        
    Returns:
        Annotated image
    """
    img = image.copy()
    h, w = img.shape[:2]
    
    # Draw all detected lines (thin, light blue)
    for line in lines:
        cv2.line(img, 
                (int(line.start[0]), int(line.start[1])),
                (int(line.end[0]), int(line.end[1])),
                (200, 200, 100), 1)  # Light blue-gray
    
    # Highlight top three lines (thicker, blue)
    for i, line in enumerate(top_three_lines):
        color = (255, 0, 0) if line == middle_line else (200, 0, 0)  # Bright blue for middle, darker for others
        thickness = 3 if line == middle_line else 2
        cv2.line(img,
                (int(line.start[0]), int(line.start[1])),
                (int(line.end[0]), int(line.end[1])),
                color, thickness)
    
    # Draw obstacles (colored circles)
    for obs_x, obs_y, obs_radius in obstacles:
        cv2.circle(img, (int(obs_x), int(obs_y)), int(obs_radius),
                  (0, 0, 255), 2)  # Red outline
    
    # Draw waypoints (green circles)
    for waypoint in waypoints:
        # Convert world to pixel if needed (assuming waypoint is in world coords)
        # For now, assume waypoint coordinates match pixel coordinates
        wp_x, wp_y = int(waypoint.x), int(waypoint.y)
        cv2.circle(img, (wp_x, wp_y), 5, (0, 255, 0), -1)  # Green filled circle
    
    # Draw path from robot to goal through waypoints (green line)
    if waypoints:
        points = [robot_pos]
        for wp in waypoints:
            points.append((int(wp.x), int(wp.y)))
        points.append(goal_pos)
        
        for i in range(len(points) - 1):
            cv2.line(img, 
                    (int(points[i][0]), int(points[i][1])),
                    (int(points[i+1][0]), int(points[i+1][1])),
                    (0, 255, 0), 2)  # Green path
    
    # Draw robot position (green circle with arrow)
    robot_x, robot_y = int(robot_pos[0]), int(robot_pos[1])
    cv2.circle(img, (robot_x, robot_y), 10, (0, 255, 0), -1)  # Green robot
    cv2.circle(img, (robot_x, robot_y), 10, (0, 200, 0), 2)  # Darker green outline
    
    # Draw goal position (yellow circle)
    goal_x, goal_y = int(goal_pos[0]), int(goal_pos[1])
    cv2.circle(img, (goal_x, goal_y), 8, (0, 255, 255), -1)  # Yellow goal
    cv2.circle(img, (goal_x, goal_y), 8, (0, 200, 200), 2)  # Darker yellow outline
    
    # Add text labels
    cv2.putText(img, "ROBOT", (robot_x - 20, robot_y - 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    cv2.putText(img, "GOAL", (goal_x - 15, goal_y - 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    
    if middle_line:
        mid_x = int(middle_line.center[0])
        mid_y = int(middle_line.center[1])
        cv2.putText(img, "MIDDLE LINE", (mid_x - 40, mid_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    
    return img


def main():
    """Main function"""
    camera_url = "http://192.168.0.21:8000/"
    
    print("=" * 60)
    print("Path Visualizer - Camera Feed")
    print("=" * 60)
    print(f"Camera: {camera_url}")
    print()
    
    # Capture frame from camera
    frame = capture_camera_frame(camera_url)
    if frame is None:
        print("❌ Failed to capture frame")
        return
    
    # Initialize detectors
    line_detector = LineDetector(line_color='blue')
    robot_localizer = RobotLocalizer(robot_marker_color='green')
    obstacle_detector = ObstacleDetector()
    path_planner = PathPlanner()
    
    # Detect lines
    print("\n🔍 Detecting lines...")
    lines = line_detector.detect_lines(frame, None)
    print(f"   Found {len(lines)} lines")
    
    # Find top three lines
    top_three = find_top_three_lines(lines)
    print(f"   Top three lines: {len(top_three)}")
    
    if len(top_three) < 3:
        print("⚠️  Need at least 3 lines to find middle line")
        print("   Displaying image with detected lines...")
        # Draw all lines
        for line in lines:
            cv2.line(frame,
                    (int(line.start[0]), int(line.start[1])),
                    (int(line.end[0]), int(line.end[1])),
                    (255, 0, 0), 2)
        cv2.imshow('Camera Feed with Lines', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return
    
    # Find middle line
    middle_line = find_middle_line_of_top_three(top_three)
    if middle_line:
        print(f"   Middle line center: ({middle_line.center[0]:.1f}, {middle_line.center[1]:.1f})")
    
    # Detect robot
    print("\n🤖 Detecting robot...")
    pose = robot_localizer.localize(frame)
    if pose:
        robot_pos = (pose.x, pose.y)  # Pixel coordinates
        print(f"   Robot position: ({robot_pos[0]:.1f}, {robot_pos[1]:.1f})")
    else:
        print("   ⚠️  Robot not detected, using center of image")
        h, w = frame.shape[:2]
        robot_pos = (w // 2, h // 2)
    
    # Detect obstacles
    print("\n🚧 Detecting obstacles...")
    detected_obstacles = obstacle_detector.detect_obstacles(frame, None)
    print(f"   Found {len(detected_obstacles)} obstacles")
    
    # Convert obstacles to pixel coordinates for drawing
    # For visualization, we work entirely in pixel space
    obstacle_list = []
    for obs in detected_obstacles:
        # Use pixel coordinates directly, scale radius for visibility
        pixel_radius = max(10, int(obs.radius * 2))  # Scale for visibility
        obstacle_list.append((obs.center[0], obs.center[1], pixel_radius))
        
        # Add to path planner using pixel coordinates
        # Convert pixel radius to a reasonable scale (assume image is ~640px wide = ~2.5m)
        world_radius = (pixel_radius / 640.0) * 2.5
        path_obs = Obstacle(
            x=obs.center[0],  # Use pixel coordinates as "world" for path planning
            y=obs.center[1],
            radius=world_radius,
            confidence=obs.confidence,
            color=obs.color
        )
        path_planner.add_obstacle(path_obs)
    
    # Plan path to middle line
    if middle_line and pose:
        print("\n🗺️  Planning path...")
        goal_pos = (middle_line.center[0], middle_line.center[1])
        
        # Plan path from robot to goal (all in pixel coordinates)
        waypoints = path_planner.plan_obstacle_avoidance_path(robot_pos, goal_pos)
        
        print(f"   Goal: ({goal_pos[0]:.1f}, {goal_pos[1]:.1f})")
        print(f"   Waypoints: {len(waypoints)}")
        
        # Draw path on image
        annotated_frame = draw_path_on_image(
            frame,
            robot_pos,
            goal_pos,
            waypoints,
            obstacle_list,
            lines,
            top_three,
            middle_line
        )
        
        # Save image
        output_file = "path_visualization.jpg"
        output_path = os.path.join(os.path.dirname(__file__), output_file)
        cv2.imwrite(output_path, annotated_frame)
        print(f"\n💾 Saved visualization to: {output_path}")
        print(f"   Full path: {os.path.abspath(output_path)}")
        
        # Optionally display (only if DISPLAY is set, for headless systems)
        try:
            if 'DISPLAY' in os.environ or sys.platform == 'darwin':
                cv2.imshow('Path Visualization - Camera Feed', annotated_frame)
                print("\n📺 Displaying visualization window...")
                print("   Press any key to close (or wait 5 seconds)")
                cv2.waitKey(5000)  # Wait 5 seconds or until key press
                cv2.destroyAllWindows()
        except:
            print("   (Skipping display - headless mode)")
    else:
        print("⚠️  Cannot plan path: missing robot position or middle line")
        # Still save the frame with detected lines
        output_file = "camera_frame_with_lines.jpg"
        output_path = os.path.join(os.path.dirname(__file__), output_file)
        cv2.imwrite(output_path, frame)
        print(f"   Saved frame to: {output_path}")


if __name__ == "__main__":
    main()


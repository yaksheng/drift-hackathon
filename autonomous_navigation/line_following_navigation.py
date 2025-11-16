"""
Line Following Navigation System

Uses overhead camera to detect lines on the floor and guides the robot
to follow those lines using path planning.
"""

import cv2
import numpy as np
import asyncio
import argparse
import sys
import os
from typing import Optional, Tuple, List
import math

# Add parent directories to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from robot_code.python_client.galaxyrvr import GalaxyRVR
from robot_code.python_client.galaxyrvr_camera import CameraStream
from webcam_code.webcam_client import receive_stream

from line_detection import LineDetector, DetectedLine
from robot_localization import RobotLocalizer, RobotPose
from navigation_controller import NavigationController, NavigationState, ControlCommand
from path_planner import PathPlanner, Waypoint, Obstacle
from dead_reckoning import DeadReckoning
from obstacle_detection import ObstacleDetector


class LineFollowingNavigation:
    """Line following navigation using overhead camera"""
    
    def __init__(self,
                 robot_ip: str = "192.168.0.113",
                 camera_url: Optional[str] = None,
                 line_color: str = 'blue',
                 robot_marker_color: str = 'green'):
        """
        Initialize line following navigation
        
        Args:
            robot_ip: IP address of robot
            camera_url: URL of overhead camera stream
            line_color: Color of lines to follow
            robot_marker_color: Color of robot marker
        """
        self.robot_ip = robot_ip
        self.camera_url = camera_url
        self.line_color = line_color
        self.robot_marker_color = robot_marker_color
        
        # Initialize robot
        self.robot: Optional[GalaxyRVR] = None
        self.robot_camera: Optional[CameraStream] = None
        
        # Initialize modules
        self.line_detector = LineDetector(line_color=line_color)
        self.robot_localizer = RobotLocalizer(robot_marker_color=robot_marker_color)
        self.obstacle_detector = ObstacleDetector()  # Detect obstacles from camera
        self.path_planner = PathPlanner()
        self.navigation_controller: Optional[NavigationController] = None
        self.dead_reckoning = DeadReckoning()
        
        # State
        self.running = False
        self.current_pose: Optional[RobotPose] = None
        self.world_transform: Optional[np.ndarray] = None
        self.overhead_frame: Optional[np.ndarray] = None
        self.detected_lines: List[DetectedLine] = []
        self.path_waypoints: List[Waypoint] = []
        self.current_path_index = 0
        
        # Position and path tracking
        self.path_history: List[Tuple[float, float]] = []
        self.position_history: List[Tuple[float, float, float]] = []  # (x, y, theta)
        
        # Obstacle tracking (from camera only)
        self.detected_obstacles = []
        
        # Line following parameters
        self.line_follow_distance = 0.15  # Distance to maintain from line (meters)
        self.max_lookahead = 0.5  # Maximum lookahead distance along line (meters)
        
    async def initialize(self):
        """Initialize robot and camera connections"""
        print("=" * 60)
        print("Initializing Line Following Navigation System")
        print("=" * 60)
        
        # Connect to robot
        print(f"\nConnecting to robot at {self.robot_ip}...")
        self.robot = GalaxyRVR(self.robot_ip)
        if not await self.robot.connect():
            print("❌ Failed to connect to robot!")
            return False
        print("✅ Robot connected")
        
        # Initialize navigation controller
        self.navigation_controller = NavigationController(self.robot, max_speed=60, min_speed=30)
        
        # Start robot camera (for onboard vision if needed)
        print("\nStarting robot camera...")
        self.robot_camera = CameraStream(self.robot_ip, display=False)
        self.robot_camera.start()
        print("✅ Robot camera started")
        
        # Load world transform if available
        transform_file = 'auto_transform_matrix.npy'
        if os.path.exists(transform_file):
            print(f"\nLoading world transform from {transform_file}...")
            self.world_transform = np.load(transform_file)
            self.robot_localizer.set_world_transform(self.world_transform)
            print("✅ World transform loaded")
        else:
            print(f"\n⚠️  World transform file not found: {transform_file}")
            print("   Using pixel coordinates (no world transform)")
        
        print("\n" + "=" * 60)
        print("Initialization Complete!")
        print("=" * 60)
        return True
    
    async def shutdown(self):
        """Shutdown and cleanup"""
        print("\nShutting down...")
        
        if self.navigation_controller:
            self.navigation_controller.reset()
        
        if self.robot:
            self.robot.stop()
            await self.robot.send()
            await self.robot.disconnect()
        
        if self.robot_camera:
            self.robot_camera.stop()
        
        print("✅ Shutdown complete")
    
    def update_overhead_frame(self, frame: np.ndarray):
        """Update overhead camera frame"""
        self.overhead_frame = frame
    
    def detect_lines(self, frame: np.ndarray) -> List[DetectedLine]:
        """Detect lines in overhead camera frame"""
        lines = self.line_detector.detect_lines(frame, self.world_transform)
        self.detected_lines = lines
        return lines
    
    def find_path_along_line(self, 
                            robot_pos: Tuple[float, float],
                            robot_theta: float,
                            line: DetectedLine) -> List[Waypoint]:
        """
        Create waypoints along a line for the robot to follow
        
        Args:
            robot_pos: Current robot position (x, y)
            robot_theta: Current robot orientation
            line: Detected line to follow
            
        Returns:
            List of waypoints along the line
        """
        waypoints = []
        
        # Get line endpoints in world coordinates
        line_start = line.world_start
        line_end = line.world_end
        
        # Calculate line direction vector
        dx = line_end[0] - line_start[0]
        dy = line_end[1] - line_start[1]
        line_length = math.sqrt(dx*dx + dy*dy)
        
        if line_length < 0.1:  # Line too short
            return waypoints
        
        # Normalize direction
        dir_x = dx / line_length
        dir_y = dy / line_length
        
        # Find closest point on line to robot
        # Project robot position onto line
        robot_to_start_x = robot_pos[0] - line_start[0]
        robot_to_start_y = robot_pos[1] - line_start[1]
        
        # Dot product to find projection
        projection = robot_to_start_x * dir_x + robot_to_start_y * dir_y
        
        # Clamp to line segment
        projection = max(0, min(line_length, projection))
        
        # Calculate closest point on line
        closest_x = line_start[0] + projection * dir_x
        closest_y = line_start[1] + projection * dir_y
        
        # Create waypoints ahead along the line
        lookahead_distance = min(self.max_lookahead, line_length - projection)
        num_waypoints = max(3, int(lookahead_distance / 0.2))  # Waypoint every 20cm
        
        for i in range(1, num_waypoints + 1):
            distance_along_line = projection + (i * lookahead_distance / num_waypoints)
            
            if distance_along_line > line_length:
                break
            
            # Calculate waypoint position
            waypoint_x = line_start[0] + distance_along_line * dir_x
            waypoint_y = line_start[1] + distance_along_line * dir_y
            
            # Offset perpendicular to line to maintain following distance
            # Perpendicular vector (rotate 90 degrees)
            perp_x = -dir_y
            perp_y = dir_x
            
            # Offset position
            waypoint_x += perp_x * self.line_follow_distance
            waypoint_y += perp_y * self.line_follow_distance
            
            waypoint = Waypoint(x=waypoint_x, y=waypoint_y, reached=False)
            waypoints.append(waypoint)
        
        return waypoints
    
    def select_best_line(self, 
                        robot_pos: Tuple[float, float],
                        robot_theta: float) -> Optional[DetectedLine]:
        """
        Select the best line to follow based on robot position and orientation
        
        Args:
            robot_pos: Current robot position
            robot_theta: Current robot orientation
            
        Returns:
            Best line to follow, or None
        """
        if not self.detected_lines:
            return None
        
        # Score each line based on:
        # 1. Distance from robot
        # 2. Alignment with robot's forward direction
        # 3. Length (longer lines are better)
        
        best_line = None
        best_score = -float('inf')
        
        for line in self.detected_lines:
            # Distance from robot to line center
            dx = line.world_center[0] - robot_pos[0]
            dy = line.world_center[1] - robot_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Line direction
            line_dx = line.world_end[0] - line.world_start[0]
            line_dy = line.world_end[1] - line.world_start[1]
            line_length = math.sqrt(line_dx*line_dx + line_dy*line_dy)
            
            if line_length < 0.1:
                continue
            
            # Normalize line direction
            line_dir_x = line_dx / line_length
            line_dir_y = line_dy / line_length
            
            # Robot forward direction
            robot_dir_x = math.cos(robot_theta)
            robot_dir_y = math.sin(robot_theta)
            
            # Alignment score (dot product)
            alignment = robot_dir_x * line_dir_x + robot_dir_y * line_dir_y
            
            # Score: prefer closer lines, aligned with robot direction, and longer lines
            distance_score = 1.0 / (1.0 + distance)  # Closer is better
            alignment_score = (alignment + 1.0) / 2.0  # Normalize to [0, 1]
            length_score = min(1.0, line_length / 2.0)  # Prefer longer lines
            
            score = (distance_score * 0.3 + 
                    alignment_score * 0.4 + 
                    length_score * 0.3)
            
            if score > best_score:
                best_score = score
                best_line = line
        
        return best_line
    
    async def navigation_loop(self):
        """Main line following navigation loop"""
        print("\n" + "=" * 60)
        print("Starting Line Following Navigation Loop")
        print("=" * 60)
        print("\nControls:")
        print("  - Press 'q' to quit")
        print("  - Robot will follow detected lines")
        print()
        
        self.running = True
        last_update = asyncio.get_event_loop().time()
        
        try:
            while self.running:
                current_time = asyncio.get_event_loop().time()
                dt = current_time - last_update
                last_update = current_time
                
                # Process overhead camera frame
                if self.overhead_frame is not None:
                    # Localize robot from real camera feed
                    pose = self.robot_localizer.localize(self.overhead_frame)
                    if pose:
                        # Track position history
                        self.position_history.append((pose.x, pose.y, pose.theta))
                        self.path_history.append((pose.x, pose.y))
                        
                        if self.dead_reckoning.odometry is None:
                            self.dead_reckoning.initialize(
                                (pose.x, pose.y),
                                pose.theta
                            )
                        else:
                            self.dead_reckoning.update_from_camera(
                                (pose.x, pose.y),
                                pose.theta
                            )
                        self.current_pose = pose
                    else:
                        # Use dead reckoning if camera localization fails
                        dr_pose = self.dead_reckoning.get_estimated_pose()
                        if dr_pose:
                            self.current_pose = RobotPose(
                                x=dr_pose[0],
                                y=dr_pose[1],
                                theta=dr_pose[2],
                                confidence=0.5
                            )
                            # Track estimated position
                            self.path_history.append((dr_pose[0], dr_pose[1]))
                    
                    # Detect lines in real camera feed
                    lines = self.detect_lines(self.overhead_frame)
                    
                    # Detect obstacles from camera feed (real obstacles only, no simulated obstacles)
                    detected_obs = self.obstacle_detector.detect_obstacles(
                        self.overhead_frame, 
                        self.world_transform
                    )
                    self.detected_obstacles = detected_obs
                    
                    # Update path planner with detected obstacles
                    # Clear old obstacles and add new ones from camera
                    self.path_planner.obstacles = []
                    for det_obs in detected_obs:
                        obstacle = Obstacle(
                            x=det_obs.world_pos[0],
                            y=det_obs.world_pos[1],
                            radius=det_obs.radius,
                            confidence=det_obs.confidence,
                            color=det_obs.color
                        )
                        self.path_planner.add_obstacle(obstacle)
                    
                    # Select best line to follow
                    if self.current_pose and lines:
                        best_line = self.select_best_line(
                            (self.current_pose.x, self.current_pose.y),
                            self.current_pose.theta
                        )
                        
                        if best_line:
                            # Generate waypoints along the line
                            waypoints = self.find_path_along_line(
                                (self.current_pose.x, self.current_pose.y),
                                self.current_pose.theta,
                                best_line
                            )
                            
                            if waypoints:
                                # Update path planner
                                self.path_planner.current_path = waypoints
                                self.path_planner.current_path_index = 0
                                
                                # Get next waypoint
                                next_waypoint = waypoints[0]
                                
                                # Set waypoint in navigation controller
                                self.navigation_controller.set_waypoint(
                                    (next_waypoint.x, next_waypoint.y)
                                )
                                
                                # Update navigation controller
                                if self.current_pose:
                                    command = self.navigation_controller.update(
                                        (self.current_pose.x, self.current_pose.y),
                                        self.current_pose.theta,
                                        dt
                                    )
                                    
                                    # Update dead reckoning
                                    self.dead_reckoning.update_from_motors(
                                        command.left_motor,
                                        command.right_motor
                                    )
                                    
                                    # Apply command to robot
                                    self.robot.set_motors(command.left_motor, command.right_motor)
                                    self.robot.set_servo(command.servo_angle)
                                    # Force send
                                    self.robot._last_left_motor = None
                                    self.robot._last_right_motor = None
                                    await self.robot.send()
                
                # Print status with position tracking
                if self.current_pose:
                    state = self.navigation_controller.get_state() if self.navigation_controller else "UNKNOWN"
                    num_lines = len(self.detected_lines)
                    num_obstacles = len(self.detected_obstacles)
                    path_length = len(self.path_history)
                    print(f"\rState: {state.value:12s} | "
                          f"Pos: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}) | "
                          f"θ: {math.degrees(self.current_pose.theta):.1f}° | "
                          f"Lines: {num_lines} | "
                          f"Obstacles (camera): {num_obstacles} | "
                          f"Waypoints: {len(self.path_waypoints)} | "
                          f"Path points: {path_length}", end='', flush=True)
                
                await asyncio.sleep(0.1)  # 10 Hz update rate
        
        except KeyboardInterrupt:
            print("\n\n⚠️  Interrupted by user")
        except Exception as e:
            print(f"\n\n❌ Error in navigation loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
    
    async def run_with_overhead_camera(self):
        """Run navigation with overhead camera stream"""
        if not self.camera_url:
            print("❌ Camera URL not provided!")
            return
        
        print(f"\nConnecting to overhead camera: {self.camera_url}")
        
        # Path tracking
        self.path_history = []
        self.position_history = []
        
        # Start camera stream in background
        async def camera_loop():
            try:
                import requests
                print(f"📡 Connecting to camera stream...")
                stream = requests.get(self.camera_url, stream=True, timeout=10)
                
                if stream.status_code != 200:
                    print(f"❌ Camera stream error: {stream.status_code}")
                    return
                
                print("✅ Camera connected! Receiving frames...")
                bytes_data = bytes()
                frame_count = 0
                
                for chunk in stream.iter_content(chunk_size=1024):
                    if not self.running:
                        break
                    
                    bytes_data += chunk
                    
                    # Find JPEG boundaries
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        
                        # Decode frame
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), 
                                           cv2.IMREAD_COLOR)
                        if frame is not None:
                            frame_count += 1
                            self.update_overhead_frame(frame)
                            if frame_count % 30 == 0:
                                print(f"📹 Received {frame_count} frames from camera")
            
            except Exception as e:
                print(f"❌ Camera stream error: {e}")
                import traceback
                traceback.print_exc()
        
        # Run camera and navigation in parallel
        await asyncio.gather(
            camera_loop(),
            self.navigation_loop()
        )


async def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Line Following Navigation System')
    parser.add_argument('--robot-ip', type=str, default='192.168.0.113',
                       help='Robot IP address')
    parser.add_argument('--camera-url', type=str, default='http://192.168.0.21:8000/',
                       help='Overhead camera stream URL (default: http://192.168.0.21:8000/)')
    parser.add_argument('--line-color', type=str, default='blue',
                       choices=['blue', 'green', 'red'],
                       help='Color of lines to follow')
    parser.add_argument('--robot-marker-color', type=str, default='green',
                       choices=['blue', 'green', 'red'],
                       help='Color of robot marker')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("Line Following Navigation System")
    print("=" * 60)
    print(f"Robot IP: {args.robot_ip}")
    print(f"Camera URL: {args.camera_url}")
    print(f"Line Color: {args.line_color}")
    print(f"Robot Marker: {args.robot_marker_color}")
    print()
    
    # Create navigation system
    nav = LineFollowingNavigation(
        robot_ip=args.robot_ip,
        camera_url=args.camera_url,
        line_color=args.line_color,
        robot_marker_color=args.robot_marker_color
    )
    
    # Initialize
    if not await nav.initialize():
        print("❌ Initialization failed!")
        return
    
    try:
        # Run navigation with camera
        await nav.run_with_overhead_camera()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    finally:
        # Print path summary
        if nav.path_history:
            print(f"\n📊 Path Summary:")
            print(f"   Total path points: {len(nav.path_history)}")
            if nav.position_history:
                print(f"   Total position updates: {len(nav.position_history)}")
                print(f"   Final position: ({nav.position_history[-1][0]:.2f}, {nav.position_history[-1][1]:.2f})")
        await nav.shutdown()


if __name__ == "__main__":
    asyncio.run(main())


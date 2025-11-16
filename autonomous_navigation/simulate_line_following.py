#!/usr/bin/env python3
"""
Simulated Line Following Navigation

Tests line following navigation in a simulated environment.
"""

import asyncio
import argparse
import numpy as np
import matplotlib.pyplot as plt
import cv2
from typing import List, Tuple, Optional
import sys
import os
import math

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from simulator import SimulatedRobot
from simulator_visualization import ArenaVisualizer
from mock_robot import MockGalaxyRVR
from line_detection import LineDetector, DetectedLine
from robot_localization import RobotLocalizer
from path_planner import PathPlanner, Waypoint, Obstacle
from navigation_controller import NavigationController, NavigationState
from dead_reckoning import DeadReckoning
from obstacle_detection import ObstacleDetector


class SimulatedLineFollowing:
    """Line following navigation in simulated environment"""
    
    def __init__(self,
                 initial_pos: Tuple[float, float] = (0.5, 0.5),
                 initial_theta: float = 0.0,
                 lines: Optional[List[Tuple[float, float, float, float]]] = None):
        """
        Initialize simulated line following
        
        Args:
            initial_pos: Starting robot position
            initial_theta: Starting robot orientation
            lines: List of lines (x1, y1, x2, y2) to follow
        """
        # Arena bounds
        arena_bounds = ((0, 0), (2.5, 4.0))
        
        # Create simulated robot
        self.robot = SimulatedRobot(
            initial_pos=initial_pos,
            initial_theta=initial_theta,
            arena_bounds=arena_bounds
        )
        
        # Initialize path planner first (needed for obstacles)
        self.path_planner = PathPlanner(arena_bounds=arena_bounds)
        
        # Initialize obstacle detector (will detect from camera feed)
        self.obstacle_detector = ObstacleDetector()
        
        # No simulated obstacles - only use obstacles detected from camera
        self.detected_obstacles = []
        
        # Add lines to robot
        if lines:
            for x1, y1, x2, y2 in lines:
                self.robot.add_line(x1, y1, x2, y2)
        else:
            # Default lines (horizontal lines across arena)
            self.robot.add_line(0.0, 1.0, 2.5, 1.0)  # Line 1
            self.robot.add_line(0.0, 2.0, 2.5, 2.0)  # Line 2
            self.robot.add_line(0.0, 3.0, 2.5, 3.0)  # Line 3
            # Main path line (vertical)
            self.robot.add_line(1.25, 0.0, 1.25, 4.0)  # Center vertical line
        
        # Initialize modules
        self.mock_robot = MockGalaxyRVR(self.robot)
        self.line_detector = LineDetector(line_color='blue')
        self.robot_localizer = RobotLocalizer(robot_marker_color='green')
        self.navigation_controller = NavigationController(self.mock_robot, max_speed=60, min_speed=30)
        self.dead_reckoning = DeadReckoning()
        
        # Visualization
        self.visualizer = ArenaVisualizer(arena_bounds)
        self.visualizer.draw_arena()
        # Obstacles will be drawn from camera detection, not pre-mapped
        plt.ion()  # Interactive mode
        
        # State
        self.current_pose = None
        self.detected_lines = []
        self.path_waypoints = []
        self.path_history = []
        
        # Line following parameters
        self.line_follow_distance = 0.15
        self.max_lookahead = 0.5
        
    async def initialize(self):
        """Initialize simulation"""
        print("=" * 60)
        print("Simulated Line Following Navigation")
        print("=" * 60)
        print(f"📍 Initial position: ({self.robot.get_position()[0]:.2f}, {self.robot.get_position()[1]:.2f})")
        print(f"🚧 Obstacles: Will be detected from camera feed (no pre-mapped obstacles)")
        print(f"📏 Lines: {len(self.robot.lines)}")
        print("✓ Connected to simulated robot")
        return True
    
    async def shutdown(self):
        """Shutdown simulation"""
        self.running = False
        if self.navigation_controller:
            self.navigation_controller.reset()
        print("✓ Disconnected from simulated robot")
    
    def create_synthetic_overhead_image(self) -> np.ndarray:
        """
        Create synthetic overhead camera image matching real camera feed
        This is only used as fallback when real camera is not available
        """
        # Create image matching real camera size (640x480)
        img = np.ones((480, 640, 3), dtype=np.uint8) * 255  # White background like real feed
        
        # Get robot state
        state = self.robot.get_state()
        robot_x, robot_y = state.x, state.y
        
        # Convert world to pixel coordinates (simple scaling)
        # Arena: 2.5m x 4.0m -> 640x480 pixels
        def world_to_pixel(wx, wy):
            px = int((wx / 2.5) * 640)
            py = int((wy / 4.0) * 480)
            return px, py
        
        # Draw obstacles detected from camera (if any were previously detected)
        # These represent real obstacles from camera feed
        for det_obs in self.detected_obstacles:
            obs_px, obs_py = world_to_pixel(det_obs.world_pos[0], det_obs.world_pos[1])
            obs_radius_px = int((det_obs.radius / 2.5) * 640)
            
            # Color mapping (BGR format for OpenCV)
            color_map = {
                'red': (0, 0, 255),
                'blue': (255, 0, 0),
                'green': (0, 255, 0),
                'yellow': (0, 255, 255),
                'orange': (0, 165, 255)
            }
            color = color_map.get(det_obs.color, (128, 128, 128))
            cv2.circle(img, (obs_px, obs_py), obs_radius_px, color, -1)
        
        # Draw lines (blue) - matching real camera feed
        for line in self.robot.lines:
            x1, y1, x2, y2 = line
            px1, py1 = world_to_pixel(x1, y1)
            px2, py2 = world_to_pixel(x2, y2)
            cv2.line(img, (px1, py1), (px2, py2), (255, 0, 0), 3)  # Blue line
        
        # Draw robot (green upper, light blue lower) - matching real robot appearance
        robot_px, robot_py = world_to_pixel(robot_x, robot_y)
        # Draw lower half (light blue)
        cv2.circle(img, (robot_px, robot_py), 20, (255, 200, 100), -1)  # Light blue
        # Draw upper half (green)
        cv2.ellipse(img, (robot_px, robot_py - 5), (20, 15), 0, 0, 180, (0, 255, 0), -1)  # Green upper half
        
        return img
    
    def find_path_along_line(self, 
                            robot_pos: Tuple[float, float],
                            robot_theta: float,
                            line: DetectedLine) -> List[Waypoint]:
        """Create waypoints along a line for the robot to follow"""
        waypoints = []
        
        line_start = line.world_start
        line_end = line.world_end
        
        dx = line_end[0] - line_start[0]
        dy = line_end[1] - line_start[1]
        line_length = math.sqrt(dx*dx + dy*dy)
        
        if line_length < 0.1:
            return waypoints
        
        dir_x = dx / line_length
        dir_y = dy / line_length
        
        # Find closest point on line to robot
        robot_to_start_x = robot_pos[0] - line_start[0]
        robot_to_start_y = robot_pos[1] - line_start[1]
        projection = robot_to_start_x * dir_x + robot_to_start_y * dir_y
        projection = max(0, min(line_length, projection))
        
        closest_x = line_start[0] + projection * dir_x
        closest_y = line_start[1] + projection * dir_y
        
        # Create waypoints ahead
        lookahead_distance = min(self.max_lookahead, line_length - projection)
        num_waypoints = max(3, int(lookahead_distance / 0.2))
        
        for i in range(1, num_waypoints + 1):
            distance_along_line = projection + (i * lookahead_distance / num_waypoints)
            
            if distance_along_line > line_length:
                break
            
            waypoint_x = line_start[0] + distance_along_line * dir_x
            waypoint_y = line_start[1] + distance_along_line * dir_y
            
            # Offset perpendicular
            perp_x = -dir_y
            perp_y = dir_x
            waypoint_x += perp_x * self.line_follow_distance
            waypoint_y += perp_y * self.line_follow_distance
            
            waypoint = Waypoint(x=waypoint_x, y=waypoint_y, reached=False)
            waypoints.append(waypoint)
        
        return waypoints
    
    def select_best_line(self, 
                        robot_pos: Tuple[float, float],
                        robot_theta: float) -> Optional[DetectedLine]:
        """Select the best line to follow"""
        if not self.detected_lines:
            return None
        
        best_line = None
        best_score = -float('inf')
        
        for line in self.detected_lines:
            dx = line.world_center[0] - robot_pos[0]
            dy = line.world_center[1] - robot_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            line_dx = line.world_end[0] - line.world_start[0]
            line_dy = line.world_end[1] - line.world_start[1]
            line_length = math.sqrt(line_dx*line_dx + line_dy*line_dy)
            
            if line_length < 0.1:
                continue
            
            line_dir_x = line_dx / line_length
            line_dir_y = line_dy / line_length
            
            robot_dir_x = math.cos(robot_theta)
            robot_dir_y = math.sin(robot_theta)
            
            alignment = robot_dir_x * line_dir_x + robot_dir_y * line_dir_y
            
            distance_score = 1.0 / (1.0 + distance)
            alignment_score = (alignment + 1.0) / 2.0
            length_score = min(1.0, line_length / 2.0)
            
            score = (distance_score * 0.3 + alignment_score * 0.4 + length_score * 0.3)
            
            if score > best_score:
                best_score = score
                best_line = line
        
        return best_line
    
    async def navigation_loop(self, max_iterations: int = 1000):
        """Main navigation loop"""
        print("\n" + "=" * 60)
        print("Starting Line Following Navigation")
        print("=" * 60)
        
        iteration = 0
        
        # Start real camera stream if URL provided
        real_camera_frame = None
        if self.camera_url:
            # Fetch initial frame synchronously
            try:
                import requests
                print(f"📹 Connecting to real camera: {self.camera_url}")
                response = requests.get(self.camera_url, stream=True, timeout=5)
                if response.status_code == 200:
                    bytes_data = bytes()
                    for chunk in response.iter_content(chunk_size=1024):
                        bytes_data += chunk
                        a = bytes_data.find(b'\xff\xd8')
                        b = bytes_data.find(b'\xff\xd9')
                        if a != -1 and b != -1:
                            jpg = bytes_data[a:b+2]
                            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if frame is not None:
                                real_camera_frame = frame
                                print(f"✅ Successfully connected to camera! Frame size: {frame.shape}")
                                break
                else:
                    print(f"⚠️  Camera returned status {response.status_code}, using synthetic images")
            except Exception as e:
                print(f"⚠️  Could not connect to camera: {e}")
                print("   Using synthetic images for testing")
        
        # Start continuous camera stream if URL provided
        camera_stream_task = None
        if self.camera_url:
            async def camera_stream_loop():
                nonlocal real_camera_frame
                try:
                    import requests
                    stream = requests.get(self.camera_url, stream=True, timeout=10)
                    if stream.status_code == 200:
                        bytes_data = bytes()
                        for chunk in stream.iter_content(chunk_size=1024):
                            if not self.running:
                                break
                            bytes_data += chunk
                            a = bytes_data.find(b'\xff\xd8')
                            b = bytes_data.find(b'\xff\xd9')
                            if a != -1 and b != -1:
                                jpg = bytes_data[a:b+2]
                                bytes_data = bytes_data[b+2:]
                                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                                if frame is not None:
                                    real_camera_frame = frame
                except Exception as e:
                    print(f"⚠️  Camera stream error: {e}")
            
            # Start camera stream in background
            import asyncio
            camera_stream_task = asyncio.create_task(camera_stream_loop())
            self.running = True
        
        while iteration < max_iterations:
            # Get overhead image - ALWAYS use real camera if available
            if self.camera_url and real_camera_frame is not None:
                overhead_frame = real_camera_frame.copy()
            else:
                # Fallback to synthetic only if camera not available
                overhead_frame = self.create_synthetic_overhead_image()
            
            # Localize robot
            pose = self.robot_localizer.localize(overhead_frame)
            if pose:
                if self.dead_reckoning.odometry is None:
                    self.dead_reckoning.initialize((pose.x, pose.y), pose.theta)
                else:
                    self.dead_reckoning.update_from_camera((pose.x, pose.y), pose.theta)
                self.current_pose = pose
            else:
                dr_pose = self.dead_reckoning.get_estimated_pose()
                if dr_pose:
                    from robot_localization import RobotPose
                    self.current_pose = RobotPose(
                        x=dr_pose[0], y=dr_pose[1], theta=dr_pose[2], confidence=0.5
                    )
            
            # Detect lines
            lines = self.line_detector.detect_lines(overhead_frame, None)
            self.detected_lines = lines
            
            # Detect obstacles from camera feed (real obstacles only, no simulated obstacles)
            detected_obs = self.obstacle_detector.detect_obstacles(overhead_frame, None)
            self.detected_obstacles = detected_obs
            
            # Log detected obstacles
            if detected_obs and iteration % 50 == 0:
                print(f"\n🚧 Detected {len(detected_obs)} obstacles from camera:")
                for i, obs in enumerate(detected_obs):
                    print(f"   Obstacle {i+1}: color={obs.color}, pos=({obs.world_pos[0]:.2f}, {obs.world_pos[1]:.2f}), radius={obs.radius:.2f}m, confidence={obs.confidence:.2f}")
            
            # Update path planner with detected obstacles
            # Clear old obstacles and add new ones from camera
            self.path_planner.obstacles = []
            # Clear robot obstacles and re-add from camera
            self.robot.obstacles = []
            for det_obs in detected_obs:
                obstacle = Obstacle(
                    x=det_obs.world_pos[0],
                    y=det_obs.world_pos[1],
                    radius=det_obs.radius,
                    confidence=det_obs.confidence,
                    color=det_obs.color
                )
                self.path_planner.add_obstacle(obstacle)
                # Also add to robot for collision detection
                self.robot.add_obstacle(det_obs.world_pos[0], det_obs.world_pos[1], det_obs.radius)
            
            # Select best line and create path
            if self.current_pose and lines:
                best_line = self.select_best_line(
                    (self.current_pose.x, self.current_pose.y),
                    self.current_pose.theta
                )
                
                if best_line:
                    waypoints = self.find_path_along_line(
                        (self.current_pose.x, self.current_pose.y),
                        self.current_pose.theta,
                        best_line
                    )
                    
                    if waypoints:
                        self.path_waypoints = waypoints
                        next_waypoint = waypoints[0]
                        self.navigation_controller.set_waypoint(
                            (next_waypoint.x, next_waypoint.y)
                        )
            
            # Update navigation controller
            if self.current_pose:
                command = self.navigation_controller.update(
                    (self.current_pose.x, self.current_pose.y),
                    self.current_pose.theta,
                    0.1
                )
                
                self.dead_reckoning.update_from_motors(
                    command.left_motor,
                    command.right_motor
                )
                
                self.mock_robot.set_motors(command.left_motor, command.right_motor)
                await self.mock_robot.send()
            
            # Update robot physics
            self.robot.update(0.1)
            
            # Add to path history
            if self.current_pose:
                self.path_history.append((self.current_pose.x, self.current_pose.y))
            
            # Update visualization
            if iteration % 10 == 0:  # Update every 10 iterations
                state = self.robot.get_state()
                # Clear and redraw
                self.visualizer.ax.clear()
                self.visualizer.draw_arena()
                
                # Draw obstacles detected from camera (real obstacles only)
                # These are the actual obstacles visible in the camera feed
                if self.detected_obstacles:
                    obstacle_list = [(obs.world_pos[0], obs.world_pos[1], obs.radius) 
                                    for obs in self.detected_obstacles]
                    obstacle_colors = [obs.color if obs.color else 'red' 
                                      for obs in self.detected_obstacles]
                    self.visualizer.draw_obstacles(obstacle_list, obstacle_colors=obstacle_colors)
                    
                    # Also print obstacle info for debugging
                    if iteration % 100 == 0:
                        print(f"\n📊 Visualization: Showing {len(self.detected_obstacles)} obstacles from camera feed")
                
                # Draw robot
                self.visualizer.draw_robot(state)
                
                # Draw detected lines
                for line in self.detected_lines:
                    self.visualizer.ax.plot(
                        [line.world_start[0], line.world_end[0]],
                        [line.world_start[1], line.world_end[1]],
                        'b-', linewidth=2, alpha=0.7, label='Detected Lines'
                    )
                
                # Draw waypoints
                if self.path_waypoints:
                    waypoint_x = [wp.x for wp in self.path_waypoints]
                    waypoint_y = [wp.y for wp in self.path_waypoints]
                    self.visualizer.ax.plot(waypoint_x, waypoint_y, 'go-', 
                                          markersize=8, linewidth=1, alpha=0.6, label='Waypoints')
                
                # Draw path history
                if hasattr(self, 'path_history'):
                    if self.path_history:
                        path_x = [p[0] for p in self.path_history]
                        path_y = [p[1] for p in self.path_history]
                        self.visualizer.ax.plot(path_x, path_y, 'g-', 
                                              linewidth=1, alpha=0.4, label='Robot Path')
                
                # Update status with obstacle info (from camera)
                num_obstacles = len(self.detected_obstacles)
                status = f"Iteration: {iteration} | Lines: {len(self.detected_lines)} | Waypoints: {len(self.path_waypoints)} | Obstacles (camera): {num_obstacles}"
                if self.current_pose:
                    status += f" | Pos: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})"
                self.visualizer.update_status(status)
                
                plt.pause(0.01)
            
            iteration += 1
            
            if iteration % 50 == 0:
                if self.current_pose:
                    print(f"\rIteration {iteration}: pos=({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), "
                          f"lines={len(self.detected_lines)}, waypoints={len(self.path_waypoints)}", end='', flush=True)
        
        # Stop camera stream
        if camera_stream_task:
            camera_stream_task.cancel()
            try:
                await camera_stream_task
            except asyncio.CancelledError:
                pass
        
        print("\n✅ Navigation complete!")


async def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Simulated Line Following Navigation')
    parser.add_argument('--initial-x', type=float, default=0.5,
                       help='Initial X position (meters)')
    parser.add_argument('--initial-y', type=float, default=0.5,
                       help='Initial Y position (meters)')
    parser.add_argument('--initial-theta', type=float, default=0.0,
                       help='Initial orientation (radians)')
    parser.add_argument('--max-iterations', type=int, default=500,
                       help='Maximum navigation iterations')
    parser.add_argument('--no-wait', action='store_true',
                       help='Do not wait for user input at end')
    parser.add_argument('--camera-url', type=str, default=None,
                       help='Real camera URL to use for obstacle detection (e.g., http://192.168.0.21:8000/)')
    
    args = parser.parse_args()
    
    # Create navigation system
    nav = SimulatedLineFollowing(
        initial_pos=(args.initial_x, args.initial_y),
        initial_theta=args.initial_theta
    )
    
    # If camera URL provided, use real camera for obstacle detection
    if args.camera_url:
        print(f"\n📹 Using real camera for obstacle detection: {args.camera_url}")
        nav.camera_url = args.camera_url
    
    # Initialize
    if not await nav.initialize():
        print("❌ Initialization failed!")
        return
    
    try:
        await nav.navigation_loop(max_iterations=args.max_iterations)
    finally:
        await nav.shutdown()
        if not args.no_wait:
            print("\nPress Enter to close...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass


if __name__ == "__main__":
    asyncio.run(main())


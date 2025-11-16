"""
Main Navigation Loop

Integrates all modules for autonomous navigation:
- Target detection
- Robot localization
- Path planning
- Navigation control
"""

import asyncio
import cv2
import numpy as np
import argparse
import sys
import os
import time
from typing import Optional, Tuple

# Add parent directories to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from robot_code.python_client.galaxyrvr import GalaxyRVR
from robot_code.python_client.galaxyrvr_camera import CameraStream
from webcam_code.webcam_client import receive_stream

from target_detection import TargetDetector, Target
from robot_localization import RobotLocalizer, RobotPose
from path_planner import PathPlanner, Waypoint
from navigation_controller import NavigationController, NavigationState
from line_detection import LineDetector, DetectedLine
from dead_reckoning import DeadReckoning
from path_visualization import PathVisualizer
from hybrid_vision import HybridVisionSystem, VisionFusion
from error_recovery import ErrorRecovery, ErrorType
from performance_optimizer import PerformanceOptimizer


class AutonomousNavigation:
    """Main autonomous navigation system"""
    
    def __init__(self,
                 robot_ip: str = "192.168.1.216",
                 camera_url: Optional[str] = None,
                 target_color: str = 'blue',
                 robot_marker_color: str = 'green',
                 use_overhead_camera: bool = True):
        """
        Initialize autonomous navigation system
        
        Args:
            robot_ip: IP address of robot
            camera_url: URL of overhead camera stream (if using)
            target_color: Color of targets to detect
            robot_marker_color: Color of robot marker
            use_overhead_camera: Whether to use overhead camera for localization
        """
        self.robot_ip = robot_ip
        self.camera_url = camera_url
        self.target_color = target_color
        self.robot_marker_color = robot_marker_color
        self.use_overhead_camera = use_overhead_camera
        
        # Initialize robot
        self.robot: Optional[GalaxyRVR] = None
        self.robot_camera: Optional[CameraStream] = None
        
        # Initialize modules
        self.target_detector = TargetDetector()
        self.line_detector = LineDetector(line_color='blue')  # Detect blue lines
        self.robot_localizer = RobotLocalizer(robot_marker_color=robot_marker_color)
        self.path_planner = PathPlanner()
        self.navigation_controller: Optional[NavigationController] = None
        self.dead_reckoning = DeadReckoning()  # For handling camera delay
        self.path_visualizer: Optional[PathVisualizer] = None  # For path overlay
        
        # Phase 3: Advanced features
        self.hybrid_vision: Optional[HybridVisionSystem] = None
        self.error_recovery = ErrorRecovery()
        self.performance_optimizer = PerformanceOptimizer()
        self.operation_start_time: Optional[float] = None
        
        # State
        self.running = False
        self.current_targets: list[Target] = []
        self.current_lines: list[DetectedLine] = []
        self.goal_line: Optional[DetectedLine] = None  # The blue line goal at top
        self.current_pose: Optional[RobotPose] = None
        self.world_transform: Optional[np.ndarray] = None
        self.last_camera_update: float = 0  # Track camera update time
        
        # Overhead camera frame
        self.overhead_frame: Optional[np.ndarray] = None
        
        # Path visualization settings
        self.save_path_images: bool = False  # Set to True to save images
        self.display_path_images: bool = False  # Set to True to display images
        
    async def initialize(self):
        """Initialize robot and camera connections"""
        print("=" * 60)
        print("Initializing Autonomous Navigation System")
        print("=" * 60)
        
        # Connect to robot
        print(f"\nConnecting to robot at {self.robot_ip}...")
        self.robot = GalaxyRVR(self.robot_ip)
        if not await self.robot.connect():
            print("❌ Failed to connect to robot!")
            return False
        print("✅ Robot connected")
        
        # Initialize navigation controller
        self.navigation_controller = NavigationController(self.robot)
        
        # Start robot camera (for onboard vision)
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
            # Initialize path visualizer with transform
            self.path_visualizer = PathVisualizer(world_transform=self.world_transform)
            print("✅ World transform loaded")
        else:
            print(f"\n⚠️  World transform file not found: {transform_file}")
            print("   Using pixel coordinates (no world transform)")
            self.path_visualizer = PathVisualizer(world_transform=None)
        
        # Initialize hybrid vision system
        self.hybrid_vision = HybridVisionSystem(
            self.target_detector,
            self.robot_localizer
        )
        
        print("\n" + "=" * 60)
        print("Initialization Complete!")
        print("=" * 60)
        print("📡 Dead reckoning enabled to handle camera delay")
        print("📊 Path visualization enabled")
        print("🔀 Hybrid vision system enabled (overhead + onboard)")
        print("🛡️  Error recovery system enabled")
        print("⚡ Performance optimization enabled")
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
    
    def detect_targets(self, frame: np.ndarray) -> list[Target]:
        """Detect targets in frame"""
        targets = self.target_detector.detect_color_targets(frame, self.target_color)
        
        # Convert to world coordinates if transform available
        if self.world_transform is not None:
            targets = self.target_detector.convert_to_world_coords(targets, self.world_transform)
        
        # Track targets
        targets = self.target_detector.track_targets(targets)
        
        return targets
    
    def detect_goal_line(self, frame: np.ndarray) -> Optional[DetectedLine]:
        """
        Detect the blue goal line at the top of the arena
        
        Args:
            frame: Input image from overhead camera
            
        Returns:
            DetectedLine for the goal line, or None if not found
        """
        # Detect all blue lines
        lines = self.line_detector.detect_lines(frame, self.world_transform)
        self.current_lines = lines
        
        if not lines:
            return None
        
        # Find the line at the top (highest Y in world coordinates, which is top of arena)
        # In world coordinates, higher Y = closer to top
        top_line = max(lines, key=lambda l: l.world_center[1])
        
        # Also check if it's near the middle horizontally
        # For now, use the topmost line as goal
        self.goal_line = top_line
        
        return top_line
    
    def localize_robot(self, frame: np.ndarray) -> Optional[RobotPose]:
        """Localize robot position"""
        pose = self.robot_localizer.localize(frame)
        return pose
    
    async def navigation_loop(self):
        """Main navigation control loop"""
        print("\n" + "=" * 60)
        print("Starting Navigation Loop")
        print("=" * 60)
        print("\nControls:")
        print("  - Press 'q' to quit")
        print("  - Press 's' to search for targets")
        print("  - Press 'r' to reset")
        print()
        
        self.running = True
        last_update = time.time()
        self.operation_start_time = time.time()
        
        try:
            while self.running:
                current_time = time.time()
                dt = current_time - last_update
                last_update = current_time
                
                # Phase 3: Performance Optimization - Check if we should process this frame
                if not self.performance_optimizer.should_process_frame(current_time):
                    await asyncio.sleep(0.01)  # Small sleep to prevent busy waiting
                    continue
                
                # Start processing timer
                processing_start = self.performance_optimizer.start_processing()
                self.performance_optimizer.record_frame(current_time)
                
                # Check if camera update is delayed
                time_since_camera = current_time - self.last_camera_update
                camera_delayed = time_since_camera > 2.0  # More than 2 seconds
                
                goal_line = None
                targets = []
                overhead_fusion: Optional[VisionFusion] = None
                onboard_fusion: Optional[VisionFusion] = None
                
                # Phase 3: Hybrid Vision - Process overhead frame
                if self.use_overhead_camera and self.overhead_frame is not None:
                    frame = self.overhead_frame.copy()
                    
                    # Use hybrid vision system
                    overhead_fusion = self.hybrid_vision.process_overhead_frame(frame)
                    
                    # Extract results
                    goal_line = self.detect_goal_line(frame)
                    self.goal_line = goal_line
                    targets = overhead_fusion.targets if overhead_fusion.targets else []
                    self.current_targets = targets
                    pose = overhead_fusion.robot_pose
                    
                    # Update dead reckoning with camera measurement (ground truth)
                    if pose:
                        # Initialize dead reckoning if not already initialized
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
                        self.last_camera_update = current_time
                        self.current_pose = pose
                    elif camera_delayed:
                        # Use dead reckoning estimate if camera is delayed
                        dr_pose = self.dead_reckoning.get_estimated_pose()
                        if dr_pose:
                            self.current_pose = RobotPose(
                                x=dr_pose[0],
                                y=dr_pose[1],
                                theta=dr_pose[2]
                            )
                            print(f"\r⚠️  Using dead reckoning (camera delay: {time_since_camera:.1f}s)", end='', flush=True)
                elif camera_delayed:
                    # No camera frame available, use dead reckoning
                    dr_pose = self.dead_reckoning.get_estimated_pose()
                    if dr_pose:
                        self.current_pose = RobotPose(
                            x=dr_pose[0],
                            y=dr_pose[1],
                            theta=dr_pose[2]
                        )
                        print(f"\r⚠️  Using dead reckoning (no camera: {time_since_camera:.1f}s)", end='', flush=True)
                
                # Phase 3: Hybrid Vision - Process onboard frame if available
                if self.robot_camera and hasattr(self.robot_camera, 'get_frame'):
                    try:
                        onboard_frame = self.robot_camera.get_frame()
                        if onboard_frame is not None:
                            onboard_fusion = self.hybrid_vision.process_onboard_frame(onboard_frame)
                    except:
                        pass  # Onboard camera not available
                
                # Phase 3: Fuse vision data
                if self.hybrid_vision:
                    fused_vision = self.hybrid_vision.fuse_vision(overhead_fusion, onboard_fusion)
                    # Use fused targets if available
                    if fused_vision.targets:
                        targets = fused_vision.targets
                        self.current_targets = targets
                    # Use fused obstacles
                    if fused_vision.obstacles:
                        for obs_x, obs_y, obs_radius in fused_vision.obstacles:
                            from path_planner import Obstacle
                            obstacle = Obstacle(x=obs_x, y=obs_y, radius=obs_radius, confidence=0.6)
                            self.path_planner.add_obstacle(obstacle)
                
                # Phase 3: Error Recovery - Detect errors
                error_type = None
                if self.navigation_controller:
                    error_type = self.error_recovery.detect_errors(
                        (self.current_pose.x, self.current_pose.y) if self.current_pose else None,
                        self.navigation_controller.state,
                        self.operation_start_time
                    )
                    
                    # Update error recovery with current position
                    if self.current_pose:
                        self.error_recovery.update_position((self.current_pose.x, self.current_pose.y))
                    
                    # Handle critical errors immediately
                    if error_type in [ErrorType.SENSOR_FAILURE]:
                        print(f"\n⚠️  Critical error detected: {error_type.value}")
                        recovery_cmd = self.error_recovery.get_recovery_command(
                            (self.current_pose.x, self.current_pose.y) if self.current_pose else None,
                            self.current_pose.theta if self.current_pose else 0.0
                        )
                        if recovery_cmd != (0, 0):
                            self.robot.set_motors(recovery_cmd[0], recovery_cmd[1])
                            await self.robot.send()
                            await asyncio.sleep(0.1)  # Execute recovery command
                            continue  # Skip normal navigation this cycle
                
                # Determine goal position (works with both camera and dead reckoning)
                goal_pos = None
                
                if goal_line and self.current_pose:
                    # Use the center of the blue line as goal
                    goal_pos = goal_line.world_center
                    print(f"🎯 Goal line detected at: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})")
                    
                    # Check if reached goal line
                    distance_to_line = self.distance((self.current_pose.x, self.current_pose.y), goal_pos)
                    if distance_to_line < 0.15:  # Within 15cm
                        self.navigation_controller.state = NavigationState.ARRIVED
                        self.robot.stop()
                        await self.robot.send()
                        print(f"\n🎯 SUCCESS: Reached blue line at top! (distance: {distance_to_line:.2f}m)")
                        print("✅ Challenge complete: Navigated to middle blue line while avoiding obstacles!")
                        break
                
                # If no goal line detected, use closest target as fallback
                if goal_pos is None and targets and self.current_pose:
                    closest_target = min(targets, 
                                        key=lambda t: self.distance(
                                            (self.current_pose.x, self.current_pose.y), 
                                            t.world_pos
                                        ))
                    goal_pos = closest_target.world_pos
                    print(f"⚠️  Goal line not detected, using target: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})")
                
                # Update obstacles dynamically from sensor data
                if self.current_pose and self.robot:
                    self.path_planner.update_obstacle_from_sensors(
                        (self.current_pose.x, self.current_pose.y),
                        self.current_pose.theta,
                        self.robot.ultrasonic_distance,
                        self.robot.ir_left,
                        self.robot.ir_right
                    )
                
                # Set goal and plan path
                if goal_pos and self.current_pose:
                    # Set target in navigation controller
                    self.navigation_controller.set_target(goal_pos)
                    
                    # Plan path (obstacles are detected dynamically from sensors)
                    self.path_planner.replan_path(
                        (self.current_pose.x, self.current_pose.y),
                        goal_pos
                    )
                    
                    # Get next waypoint
                    waypoint = self.path_planner.get_next_waypoint()
                    if waypoint:
                        self.navigation_controller.set_waypoint(
                            (waypoint.x, waypoint.y)
                        )
                
                # Update navigation controller
                if self.navigation_controller and self.current_pose:
                    command = self.navigation_controller.update(
                        (self.current_pose.x, self.current_pose.y),
                        self.current_pose.theta,
                        dt
                    )
                    
                    # Update dead reckoning with motor commands
                    self.dead_reckoning.update_from_motors(
                        command.left_motor,
                        command.right_motor
                    )
                    
                    # Phase 3: Error Recovery - Check if we should override command
                    if error_type and error_type != ErrorType.TIMEOUT:
                        # Use recovery command instead
                        recovery_cmd = self.error_recovery.get_recovery_command(
                            (self.current_pose.x, self.current_pose.y),
                            self.current_pose.theta
                        )
                        if recovery_cmd != (0, 0):
                            command.left_motor = recovery_cmd[0]
                            command.right_motor = recovery_cmd[1]
                    
                    # Apply command to robot
                    # Note: If in obstacle avoidance mode, motor commands are ignored
                    # and Arduino handles control automatically
                    if not self.robot._obstacle_avoidance_enabled and not self.robot._obstacle_following_enabled:
                        self.robot.set_motors(command.left_motor, command.right_motor)
                    self.robot.set_servo(command.servo_angle)
                    # Force send by resetting tracking variables to ensure command is sent
                    # This ensures mode changes are sent even if motor values haven't changed
                    self.robot._last_left_motor = None
                    self.robot._last_right_motor = None
                    self.robot._last_obstacle_avoidance_enabled = not self.robot._obstacle_avoidance_enabled
                    self.robot._last_obstacle_following_enabled = not self.robot._obstacle_following_enabled
                    await self.robot.send()
                
                # Update path visualization
                if self.path_visualizer and self.current_pose:
                    # Add current position to path history
                    self.path_visualizer.add_path_point((self.current_pose.x, self.current_pose.y))
                    
                    # Get planned waypoints
                    waypoints = []
                    if self.path_planner.current_path:
                        waypoints = [(wp.x, wp.y) for wp in self.path_planner.current_path if not wp.reached]
                    
                    # Get goal position
                    goal_pos = None
                    if self.goal_line:
                        goal_pos = self.goal_line.world_center
                    elif self.current_targets:
                        closest = min(self.current_targets,
                                    key=lambda t: self.distance((self.current_pose.x, self.current_pose.y),
                                                               t.world_pos))
                        goal_pos = closest.world_pos
                    
                    # Overlay path on frame if available
                    if self.use_overhead_camera and self.overhead_frame is not None:
                        vis_frame = self.path_visualizer.overlay_path(
                            self.overhead_frame.copy(),
                            robot_pos=(self.current_pose.x, self.current_pose.y),
                            goal_pos=goal_pos,
                            waypoints=waypoints
                        )
                        
                        # Phase 3: Add performance metrics overlay
                        metrics = self.performance_optimizer.get_metrics()
                        metrics_text = f"FPS: {metrics.frame_rate:.1f} | "
                        metrics_text += f"Proc: {metrics.processing_time*1000:.1f}ms | "
                        metrics_text += f"Cache: {metrics.cache_hit_rate*100:.0f}%"
                        cv2.putText(vis_frame, metrics_text, (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        # Phase 3: Add error status overlay
                        if error_type:
                            error_text = f"ERROR: {error_type.value.upper()}"
                            cv2.putText(vis_frame, error_text, (10, 60),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        
                        # Phase 3: Add hybrid vision status
                        if self.hybrid_vision:
                            vision_text = "Vision: "
                            if overhead_fusion and overhead_fusion.overhead_available:
                                vision_text += "Overhead✓ "
                            if onboard_fusion and onboard_fusion.onboard_available:
                                vision_text += "Onboard✓"
                            if vision_text != "Vision: ":
                                cv2.putText(vis_frame, vision_text, (10, 90),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                        
                        # Save or display image if enabled
                        if self.save_path_images:
                            os.makedirs('path_images', exist_ok=True)
                            filename = f"path_images/path_{int(current_time * 1000)}.jpg"
                            cv2.imwrite(filename, vis_frame)
                        
                        if self.display_path_images:
                            cv2.imshow('Navigation Path', vis_frame)
                            cv2.waitKey(1)
                
                # Phase 3: End processing timer
                self.performance_optimizer.end_processing(processing_start)
                
                # Print status
                if self.current_pose:
                    state = self.navigation_controller.get_state() if self.navigation_controller else "UNKNOWN"
                    dr_confidence = self.dead_reckoning.get_confidence()
                    metrics = self.performance_optimizer.get_metrics()
                    print(f"\rState: {state.value:12s} | "
                          f"Pos: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}) | "
                          f"Targets: {len(self.current_targets)} | "
                          f"DR: {dr_confidence:.2f} | "
                          f"FPS: {metrics.frame_rate:.1f}", end='', flush=True)
                
                # Phase 3: Adaptive sleep based on performance
                sleep_time = max(0.01, self.performance_optimizer.frame_interval - 
                               self.performance_optimizer.metrics.processing_time)
                await asyncio.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n\n⚠️  Interrupted by user")
        except Exception as e:
            print(f"\n\n❌ Error in navigation loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
    
    def distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate distance between two points"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return np.sqrt(dx*dx + dy*dy)
    
    async def run_with_overhead_camera(self):
        """Run navigation with overhead camera stream"""
        if not self.camera_url:
            print("❌ Camera URL not provided!")
            return
        
        print(f"\nConnecting to overhead camera: {self.camera_url}")
        
        # Start camera stream in background
        async def camera_loop():
            try:
                # Use requests to get frames (simplified)
                import requests
                stream = requests.get(self.camera_url, stream=True, timeout=5)
                
                if stream.status_code != 200:
                    print(f"❌ Camera stream error: {stream.status_code}")
                    return
                
                bytes_data = bytes()
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
                            self.update_overhead_frame(frame)
            
            except Exception as e:
                print(f"❌ Camera stream error: {e}")
        
        # Run camera and navigation in parallel
        await asyncio.gather(
            camera_loop(),
            self.navigation_loop()
        )
    
    async def run_without_overhead_camera(self):
        """Run navigation using only robot's onboard camera"""
        print("\n⚠️  Running without overhead camera (onboard vision only)")
        print("   This mode is limited - overhead camera recommended")
        
        await self.navigation_loop()


async def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Autonomous Navigation System')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.216',
                       help='Robot IP address')
    parser.add_argument('--camera-url', type=str, default=None,
                       help='Overhead camera stream URL (e.g., http://192.168.1.109:5000/)')
    parser.add_argument('--target-color', type=str, default='blue',
                       choices=['blue', 'green', 'yellow', 'red', 'orange'],
                       help='Color of targets to detect')
    parser.add_argument('--robot-marker-color', type=str, default='green',
                       choices=['blue', 'green', 'red'],
                       help='Color of robot marker')
    parser.add_argument('--no-overhead', action='store_true',
                       help='Disable overhead camera (use onboard only)')
    parser.add_argument('--save-path-images', action='store_true',
                       help='Save path visualization images to path_images/ directory')
    parser.add_argument('--display-path-images', action='store_true',
                       help='Display path visualization in real-time window')
    
    args = parser.parse_args()
    
    # Create navigation system
    nav = AutonomousNavigation(
        robot_ip=args.robot_ip,
        camera_url=args.camera_url,
        target_color=args.target_color,
        robot_marker_color=args.robot_marker_color,
        use_overhead_camera=not args.no_overhead
    )
    
    # Enable path visualization if requested
    nav.save_path_images = args.save_path_images
    nav.display_path_images = args.display_path_images
    
    # Initialize
    if not await nav.initialize():
        print("❌ Initialization failed!")
        return
    
    try:
        # Run navigation
        if nav.use_overhead_camera and nav.camera_url:
            await nav.run_with_overhead_camera()
        else:
            await nav.run_without_overhead_camera()
    finally:
        await nav.shutdown()


if __name__ == "__main__":
    asyncio.run(main())


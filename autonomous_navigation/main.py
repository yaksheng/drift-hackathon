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
        self.robot_localizer = RobotLocalizer(robot_marker_color=robot_marker_color)
        self.path_planner = PathPlanner()
        self.navigation_controller: Optional[NavigationController] = None
        
        # State
        self.running = False
        self.current_targets: list[Target] = []
        self.current_pose: Optional[RobotPose] = None
        self.world_transform: Optional[np.ndarray] = None
        
        # Overhead camera frame
        self.overhead_frame: Optional[np.ndarray] = None
        
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
    
    def detect_targets(self, frame: np.ndarray) -> list[Target]:
        """Detect targets in frame"""
        targets = self.target_detector.detect_color_targets(frame, self.target_color)
        
        # Convert to world coordinates if transform available
        if self.world_transform is not None:
            targets = self.target_detector.convert_to_world_coords(targets, self.world_transform)
        
        # Track targets
        targets = self.target_detector.track_targets(targets)
        
        return targets
    
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
        
        try:
            while self.running:
                current_time = time.time()
                dt = current_time - last_update
                last_update = current_time
                
                # Get overhead frame if available
                if self.use_overhead_camera and self.overhead_frame is not None:
                    frame = self.overhead_frame.copy()
                    
                    # Detect targets
                    targets = self.detect_targets(frame)
                    self.current_targets = targets
                    
                    # Localize robot
                    pose = self.localize_robot(frame)
                    self.current_pose = pose
                    
                    # Select target (closest one)
                    if targets and pose:
                        # Find closest target
                        closest_target = min(targets, 
                                            key=lambda t: self.distance(
                                                (pose.x, pose.y), 
                                                t.world_pos
                                            ))
                        
                        # Set target in navigation controller
                        self.navigation_controller.set_target(closest_target.world_pos)
                        
                        # Plan path
                        if pose:
                            self.path_planner.replan_path(
                                (pose.x, pose.y),
                                closest_target.world_pos
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
                    
                    # Apply command to robot
                    self.robot.set_motors(command.left_motor, command.right_motor)
                    self.robot.set_servo(command.servo_angle)
                    await self.robot.send()
                
                # Print status
                if self.current_pose:
                    state = self.navigation_controller.get_state() if self.navigation_controller else "UNKNOWN"
                    print(f"\rState: {state.value:12s} | "
                          f"Pos: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}) | "
                          f"Targets: {len(self.current_targets)}", end='', flush=True)
                
                # Small delay
                await asyncio.sleep(0.1)
        
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
    
    args = parser.parse_args()
    
    # Create navigation system
    nav = AutonomousNavigation(
        robot_ip=args.robot_ip,
        camera_url=args.camera_url,
        target_color=args.target_color,
        robot_marker_color=args.robot_marker_color,
        use_overhead_camera=not args.no_overhead
    )
    
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


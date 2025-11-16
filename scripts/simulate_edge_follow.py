"""Simulation for edge following and strip selection routine."""
from __future__ import annotations

import argparse
import asyncio
import time
import sys
import os
from typing import List, Optional

import cv2
import numpy as np

# Add robot_code to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'robot_code', 'python_client'))

# Add scripts directory to path for edge_follow_to_strip
sys.path.insert(0, os.path.dirname(__file__))

# Patch the imports before importing edge_follow_to_strip
# This allows us to use mock classes instead of real GalaxyRVR and CameraStream
import edge_follow_to_strip

# Import the original edge following controller
# We'll patch the imports to use our mock classes

# We need to monkey-patch the imports in edge_follow_to_strip module
# to use our mock classes instead of the real ones


class SimulatedCameraStream:
    """Simulated camera stream that generates synthetic onboard camera views."""
    
    def __init__(self, robot, environment, display=False):
        """
        Initialize simulated camera stream
        
        Args:
            robot: SimulatedRobot instance
            environment: SimulatedEnvironment instance
            display: If True, display camera feed
        """
        self.robot = robot
        self.environment = environment
        self.display = display
        self.latest_frame = None
        self.running = True
        
    def start(self):
        """Start camera stream (no-op for simulation)"""
        pass
        
    def stop(self):
        """Stop camera stream"""
        self.running = False
        if self.display:
            cv2.destroyAllWindows()
    
    def get_frame(self):
        """Get current camera frame based on robot position"""
        if not self.running:
            return None
        
        # Generate camera view based on robot position and orientation
        self.latest_frame = self.environment.get_camera_view(
            self.robot.state.x,
            self.robot.state.y,
            self.robot.state.theta,
            self.robot.state.servo_angle
        )
        
        if self.display and self.latest_frame is not None:
            cv2.imshow('Simulated Camera Feed', self.latest_frame)
            cv2.waitKey(1)
        
        return self.latest_frame


class SimulatedRobot:
    """Simulated robot with physics."""
    
    def __init__(self, initial_pos=(0.0, 0.0), initial_theta=0.0):
        """Initialize simulated robot"""
        self.state = type('State', (), {
            'x': initial_pos[0],
            'y': initial_pos[1],
            'theta': initial_theta,
            'left_motor': 0,
            'right_motor': 0,
            'servo_angle': 90
        })()
        
        # Robot physical parameters
        self.wheel_base = 0.12  # meters
        self.max_speed = 0.5  # m/s
        self.last_update_time = time.time()
        
    def set_motors(self, left: int, right: int):
        """Set motor speeds (-100 to 100)"""
        self.state.left_motor = max(-100, min(100, int(left)))
        self.state.right_motor = max(-100, min(100, int(right)))
    
    def set_servo(self, angle: int):
        """Set servo angle (0-180)"""
        self.state.servo_angle = max(0, min(180, int(angle)))
    
    def forward(self, speed: int):
        """Move forward at given speed"""
        self.set_motors(speed, speed)
    
    def backward(self, speed: int):
        """Move backward at given speed"""
        self.set_motors(-speed, -speed)
    
    def stop(self):
        """Stop robot"""
        self.set_motors(0, 0)
    
    def update(self, dt: float = 0.05):
        """Update robot physics"""
        # Convert motor speeds (-100 to 100) to wheel velocities
        left_vel = (self.state.left_motor / 100.0) * self.max_speed
        right_vel = (self.state.right_motor / 100.0) * self.max_speed
        
        # Differential drive kinematics
        linear_vel = (left_vel + right_vel) / 2.0
        angular_vel = (right_vel - left_vel) / self.wheel_base
        
        # Update position
        self.state.theta += angular_vel * dt
        self.state.x += linear_vel * np.cos(self.state.theta) * dt
        self.state.y += linear_vel * np.sin(self.state.theta) * dt
        
        # Keep theta in [-pi, pi]
        while self.state.theta > np.pi:
            self.state.theta -= 2 * np.pi
        while self.state.theta < -np.pi:
            self.state.theta += 2 * np.pi


class SimulatedEnvironment:
    """Simulated environment with white mat, edges, and black strips."""
    
    def __init__(self, width=3.0, height=4.0):
        """
        Initialize environment
        
        Args:
            width: Environment width in meters
            height: Environment height in meters
        """
        self.width = width
        self.height = height
        
        # White mat area (starts from origin, extends right and up)
        self.mat_start_x = 0.0
        self.mat_start_y = 0.0
        self.mat_width = 2.0  # meters
        self.mat_height = 3.5  # meters
        
        # Black strips (horizontal lines on the mat)
        # Strips are at increasing Y positions
        self.strip_positions = [
            1.5,  # Strip 1 (nearest)
            2.0,  # Strip 2 (middle)
            2.5,  # Strip 3 (furthest)
        ]
        self.strip_width = 0.15  # meters (15cm wide)
        
        # Camera parameters
        self.camera_resolution = (320, 240)  # width, height
        self.camera_fov = 60.0  # degrees
        self.camera_height = 0.15  # meters above ground
        self.pixels_per_meter = 100  # For rendering
        
    def get_camera_view(self, robot_x: float, robot_y: float, 
                       robot_theta: float, servo_angle: int) -> np.ndarray:
        """
        Generate camera view from robot's perspective
        
        Args:
            robot_x: Robot X position in meters
            robot_y: Robot Y position in meters
            robot_theta: Robot orientation in radians
            servo_angle: Servo angle (0-180, 90 = straight down)
        
        Returns:
            Camera frame as numpy array (BGR)
        """
        width, height = self.camera_resolution
        
        # Create blank frame (dark background - represents floor/table)
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame.fill(30)  # Dark gray background
        
        # Calculate camera look direction based on servo angle
        # Servo angle: 0 = forward, 90 = down, 180 = backward
        # For edge following, we use ~135 (45 degrees down) or 170 (nearly straight down)
        # Convert servo angle to look-down angle: 90 = straight down, 135 = 45 degrees down
        look_down_angle = np.radians(servo_angle - 90)  # 0 = forward, positive = down
        
        # Calculate view area on ground
        # Camera FOV in radians
        fov_rad = np.radians(self.camera_fov)
        
        # Distance to ground plane (projection distance)
        if abs(look_down_angle) < 0.01:
            ground_distance = 0.5  # Default forward distance if looking forward
        else:
            ground_distance = self.camera_height / np.sin(abs(look_down_angle))
        
        # View area dimensions at ground
        view_width = 2 * ground_distance * np.tan(fov_rad / 2)
        view_height = view_width * (height / width)
        
        # Calculate view center in world coordinates
        # Camera looks in robot's forward direction, projected down
        forward_x = np.cos(robot_theta)
        forward_y = np.sin(robot_theta)
        
        # View center is ahead of robot (projected onto ground)
        view_center_x = robot_x + forward_x * ground_distance * np.cos(look_down_angle)
        view_center_y = robot_y + forward_y * ground_distance * np.cos(look_down_angle)
        
        # Draw white mat area and strips
        # Convert world coordinates to pixel coordinates
        for y_pixel in range(height):
            for x_pixel in range(width):
                # Convert pixel to world coordinates
                # Camera view: x increases left to right, y increases top to bottom
                # But in world, we need to account for robot orientation
                rel_x = (x_pixel / width - 0.5) * view_width
                rel_y = ((height - y_pixel) / height - 0.5) * view_height
                
                # Rotate relative coordinates by robot orientation
                world_x = view_center_x + rel_x * np.cos(robot_theta) - rel_y * np.sin(robot_theta)
                world_y = view_center_y + rel_x * np.sin(robot_theta) + rel_y * np.cos(robot_theta)
                
                # Check if point is on white mat
                on_mat = (self.mat_start_x <= world_x <= self.mat_start_x + self.mat_width and
                         self.mat_start_y <= world_y <= self.mat_start_y + self.mat_height)
                
                if on_mat:
                    # Check if point is on a black strip
                    on_strip = False
                    for strip_y in self.strip_positions:
                        if abs(world_y - strip_y) < self.strip_width / 2:
                            on_strip = True
                            break
                    
                    if on_strip:
                        # Black strip
                        frame[y_pixel, x_pixel] = [20, 20, 20]  # Very dark (almost black)
                    else:
                        # White mat
                        frame[y_pixel, x_pixel] = [240, 240, 240]  # Light gray/white
                else:
                    # Outside mat - dark background (this creates the edge)
                    frame[y_pixel, x_pixel] = [30, 30, 30]  # Dark gray/black background
        
        # Add some noise for realism
        noise = np.random.randint(-10, 10, frame.shape, dtype=np.int16)
        frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        # Add slight blur to simulate camera
        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        
        return frame


class MockGalaxyRVR:
    """Mock robot interface compatible with GalaxyRVR from edge_follow_to_strip.py."""
    
    def __init__(self, robot_ip="192.168.1.216", port=8765):
        """
        Initialize mock robot
        
        Args:
            robot_ip: IP address (ignored in simulation)
            port: Port (ignored in simulation)
        """
        self.robot_ip = robot_ip
        self.port = port
        self.connected = False
        self.running = True  # For compatibility with EdgeFollowNavigator
        self.simulated_robot = None  # Will be set by simulation
        self.environment = None  # Will be set by simulation
        
        # Sensor data (for obstacle avoidance)
        self.ultrasonic_distance = None  # cm
        self.ir_left = None  # 0 or 1
        self.ir_right = None  # 0 or 1
        
        # Motor state (for compatibility with GalaxyRVR interface)
        self.left_motor = 0
        self.right_motor = 0
        
    def set_simulated_robot(self, simulated_robot: SimulatedRobot):
        """Set the simulated robot instance"""
        self.simulated_robot = simulated_robot
        
    def set_environment(self, environment: SimulatedEnvironment):
        """Set the environment for sensor readings"""
        self.environment = environment
        
    def _update_sensors(self):
        """Update sensor readings based on robot position and environment"""
        if not self.simulated_robot or not self.environment:
            return
        
        # Get robot state
        state = self.simulated_robot.state
        
        # Simulate ultrasonic sensor (forward distance)
        # Check for obstacles in front of robot
        forward_x = state.x + 0.2 * np.cos(state.theta)  # 20cm ahead
        forward_y = state.y + 0.2 * np.sin(state.theta)
        
        # Check if robot is near mat edge (simulate obstacle)
        # For now, set to None (no obstacles) - can be enhanced later
        self.ultrasonic_distance = None  # No obstacles in simulation
        
        # Simulate IR sensors (left and right)
        # Check if robot is near mat boundaries
        left_x = state.x - 0.1 * np.sin(state.theta)
        left_y = state.y + 0.1 * np.cos(state.theta)
        right_x = state.x + 0.1 * np.sin(state.theta)
        right_y = state.y - 0.1 * np.cos(state.theta)
        
        # Check if outside mat boundaries (obstacle detected)
        mat = self.environment
        left_outside = not (mat.mat_start_x <= left_x <= mat.mat_start_x + mat.mat_width and
                           mat.mat_start_y <= left_y <= mat.mat_start_y + mat.mat_height)
        right_outside = not (mat.mat_start_x <= right_x <= mat.mat_start_x + mat.mat_width and
                            mat.mat_start_y <= right_y <= mat.mat_start_y + mat.mat_height)
        
        self.ir_left = 1 if left_outside else 0
        self.ir_right = 1 if right_outside else 0
        
    async def connect(self):
        """Mock connection"""
        self.connected = True
        return True
    
    async def disconnect(self):
        """Mock disconnection"""
        self.connected = False
    
    def set_motors(self, left: int, right: int):
        """Set motor speeds"""
        self.left_motor = left
        self.right_motor = right
        if self.simulated_robot:
            self.simulated_robot.set_motors(left, right)
            self._update_sensors()
    
    def set_servo(self, angle: int):
        """Set servo angle"""
        if self.simulated_robot:
            self.simulated_robot.set_servo(angle)
    
    def forward(self, speed: int):
        """Move forward"""
        self.left_motor = speed
        self.right_motor = speed
        if self.simulated_robot:
            self.simulated_robot.forward(speed)
            self._update_sensors()
    
    def backward(self, speed: int):
        """Move backward"""
        self.left_motor = -speed
        self.right_motor = -speed
        if self.simulated_robot:
            self.simulated_robot.backward(speed)
            self._update_sensors()
    
    def stop(self):
        """Stop robot"""
        self.left_motor = 0
        self.right_motor = 0
        if self.simulated_robot:
            self.simulated_robot.stop()
            self._update_sensors()
    
    async def send(self):
        """Mock send (updates sensors)"""
        self._update_sensors()
        pass


async def simulate_edge_following(strip: int, display: bool = True, max_iterations: int = 2000):
    """
    Simulate edge following mission using the original edge_follow_to_strip.py
    
    Args:
        strip: Target strip number (1, 2, or 3)
        display: If True, display visualization
        max_iterations: Maximum iterations before timeout
    """
    print("=" * 60)
    print("Edge Following Simulation")
    print("=" * 60)
    print(f"Target: Strip #{strip}")
    print("Using original edge_follow_to_strip.py controller")
    print()
    
    # Create simulated components
    environment = SimulatedEnvironment()
    robot = SimulatedRobot(initial_pos=(0.2, 0.2), initial_theta=0.0)
    camera = SimulatedCameraStream(robot, environment, display=display)
    
    # Create mock robot (compatible with GalaxyRVR interface)
    mock_robot = MockGalaxyRVR()
    mock_robot.set_simulated_robot(robot)
    mock_robot.set_environment(environment)
    
    # Connect mock robot
    await mock_robot.connect()
    
    # Monkey-patch the imports in edge_follow_to_strip to use our mock classes
    # This allows the original EdgeFollowNavigator to work with our mocks
    edge_follow_to_strip.GalaxyRVR = MockGalaxyRVR
    edge_follow_to_strip.CameraStream = SimulatedCameraStream
    
    # Now import EdgeFollowNavigator (it will use our patched classes)
    from edge_follow_to_strip import EdgeFollowNavigator
    
    # Create controller using original EdgeFollowNavigator
    controller = EdgeFollowNavigator(
        robot=mock_robot,
        camera=camera,
        strip_target=strip,
        hug_side="right",
        base_speed=70,
        edge_servo_angle=135,
        down_servo_angle=170,
    )
    
    # Visualization
    if display:
        viz_window = "Edge Following Simulation"
        cv2.namedWindow(viz_window, cv2.WINDOW_NORMAL)
    
    try:
        # Run simulation
        iteration = 0
        start_time = time.time()
        
        # Run the controller
        controller_task = asyncio.create_task(controller.run())
        
        # Simulation loop
        while iteration < max_iterations:
            iteration += 1
            
            # Update robot physics
            robot.update(dt=0.05)
            
            # Update visualization
            if display:
                viz_frame = environment.create_top_view(robot.state.x, robot.state.y, robot.state.theta)
                cv2.imshow(viz_window, viz_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            # Check if controller finished
            if controller_task.done():
                break
            
            await asyncio.sleep(0.05)
        
        # Wait for controller to finish
        try:
            await asyncio.wait_for(controller_task, timeout=60.0)  # Increased timeout to 60 seconds
        except asyncio.TimeoutError:
            pass
        
        elapsed = time.time() - start_time
        print(f"\nSimulation completed in {elapsed:.2f} seconds ({iteration} iterations)")
        print(f"Final position: ({robot.state.x:.2f}, {robot.state.y:.2f})")
        print(f"Final orientation: {np.degrees(robot.state.theta):.1f}°")
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted")
    finally:
        robot.stop()
        camera.stop()
        if display:
            cv2.destroyAllWindows()


# Add top view visualization method to environment
def _create_top_view(self, robot_x: float, robot_y: float, robot_theta: float) -> np.ndarray:
    """Create top-down view of environment"""
    width_px = int(self.width * self.pixels_per_meter)
    height_px = int(self.height * self.pixels_per_meter)
    
    frame = np.zeros((height_px, width_px, 3), dtype=np.uint8)
    
    # Draw white mat
    mat_x1 = int(self.mat_start_x * self.pixels_per_meter)
    mat_y1 = int(self.mat_start_y * self.pixels_per_meter)
    mat_x2 = int((self.mat_start_x + self.mat_width) * self.pixels_per_meter)
    mat_y2 = int((self.mat_start_y + self.mat_height) * self.pixels_per_meter)
    cv2.rectangle(frame, (mat_x1, mat_y1), (mat_x2, mat_y2), (240, 240, 240), -1)
    
    # Draw black strips
    for strip_y in self.strip_positions:
        strip_y_px = int(strip_y * self.pixels_per_meter)
        strip_h = int(self.strip_width * self.pixels_per_meter)
        cv2.rectangle(frame, (mat_x1, strip_y_px - strip_h//2), 
                     (mat_x2, strip_y_px + strip_h//2), (20, 20, 20), -1)
    
    # Draw robot
    robot_x_px = int(robot_x * self.pixels_per_meter)
    robot_y_px = int(robot_y * self.pixels_per_meter)
    cv2.circle(frame, (robot_x_px, robot_y_px), 15, (0, 255, 0), -1)
    
    # Draw robot orientation
    arrow_len = 30
    arrow_x = int(robot_x_px + arrow_len * np.cos(robot_theta))
    arrow_y = int(robot_y_px + arrow_len * np.sin(robot_theta))
    cv2.arrowedLine(frame, (robot_x_px, robot_y_px), (arrow_x, arrow_y), (0, 255, 0), 2)
    
    # Flip Y axis for display (origin at top-left)
    frame = cv2.flip(frame, 0)
    
    return frame

# Add method to SimulatedEnvironment
SimulatedEnvironment.create_top_view = _create_top_view


def parse_args():
    parser = argparse.ArgumentParser(description="Simulate edge following routine.")
    parser.add_argument("strip", type=int, choices=(1, 2, 3), 
                       help="Target strip number (1=nearest, 3=furthest).")
    parser.add_argument("--display", action="store_true", default=True,
                       help="Display visualization windows.")
    parser.add_argument("--max-iterations", type=int, default=2000,
                       help="Maximum simulation iterations.")
    return parser.parse_args()


async def main():
    args = parse_args()
    await simulate_edge_following(
        strip=args.strip,
        display=args.display,
        max_iterations=args.max_iterations
    )


if __name__ == "__main__":
    asyncio.run(main())


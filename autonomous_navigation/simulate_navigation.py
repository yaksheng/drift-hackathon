"""
Simulation Main Script

Runs autonomous navigation in a simulated environment.
Supports line stopping challenge (stop at 1st, 2nd, or 3rd line).
"""

import asyncio
import argparse
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from simulator import SimulatedRobot
from simulator_visualization import ArenaVisualizer
from mock_robot import MockGalaxyRVR
from target_detection import TargetDetector
from robot_localization import RobotLocalizer
from path_planner import PathPlanner
from navigation_controller import NavigationController, NavigationState


class SimulatedNavigation:
    """Autonomous navigation in simulated environment"""
    
    def __init__(self,
                 stop_at_line: int = 1,  # 1, 2, or 3
                 initial_pos: Tuple[float, float] = (0.5, 0.5),
                 initial_theta: float = 0.0,
                 target_positions: Optional[List[Tuple[float, float]]] = None,
                 obstacles: Optional[List[Tuple[float, float, float]]] = None,
                 lines: Optional[List[Tuple[float, float, float, float]]] = None):
        """
        Initialize simulated navigation
        
        Args:
            stop_at_line: Which line to stop at (1, 2, or 3)
            initial_pos: Starting robot position
            initial_theta: Starting robot orientation
            target_positions: List of target positions (x, y)
            obstacles: List of obstacles (x, y, radius)
            lines: List of lines (x1, y1, x2, y2)
        """
        self.stop_at_line = stop_at_line
        
        # Arena bounds (based on typical arena size)
        arena_bounds = ((0, 0), (2.5, 4.0))
        
        # Create simulated robot
        self.robot = SimulatedRobot(
            initial_pos=initial_pos,
            initial_theta=initial_theta,
            arena_bounds=arena_bounds
        )
        
        # Add default obstacles if none provided
        if obstacles:
            for x, y, radius in obstacles:
                self.robot.add_obstacle(x, y, radius)
        else:
            # Default obstacles
            self.robot.add_obstacle(1.0, 2.0, 0.2)
            self.robot.add_obstacle(1.5, 1.5, 0.15)
        
        # Add default lines if none provided
        if lines:
            for x1, y1, x2, y2 in lines:
                self.robot.add_line(x1, y1, x2, y2)
        else:
            # Default lines (horizontal lines across arena)
            self.robot.add_line(0.0, 1.0, 2.5, 1.0)  # Line 1
            self.robot.add_line(0.0, 2.0, 2.5, 2.0)  # Line 2
            self.robot.add_line(0.0, 3.0, 2.5, 3.0)  # Line 3
        
        # Add default targets if none provided
        if target_positions is None:
            target_positions = [
                (2.0, 3.5, 'blue'),
                (1.5, 3.0, 'blue'),
                (0.8, 2.5, 'blue'),
            ]
        
        self.target_positions = target_positions
        
        # Create visualizer
        self.visualizer = ArenaVisualizer(arena_bounds)
        self.visualizer.draw_arena()
        self.visualizer.draw_obstacles(self.robot.obstacles)
        self.visualizer.draw_lines(self.robot.lines, stop_line_index=stop_at_line - 1)
        
        # Create mock robot interface
        self.mock_robot = MockGalaxyRVR(self.robot)
        
        # Initialize navigation modules
        self.target_detector = TargetDetector()
        self.robot_localizer = RobotLocalizer()
        self.path_planner = PathPlanner(arena_bounds=arena_bounds)
        self.navigation_controller = NavigationController(self.mock_robot)
        
        # State
        self.running = False
        self.current_targets = []
        self.current_pose = None
        self.line_crossed = [False, False, False]  # Track which lines crossed
        
        # For visualization
        self.path_history = []
        
    async def initialize(self):
        """Initialize navigation system"""
        print("=" * 60)
        print("Simulated Navigation System")
        print("=" * 60)
        print(f"\nStop at line: {self.stop_at_line}")
        print(f"Initial position: {self.robot.get_position()}")
        print(f"Targets: {len(self.target_positions)}")
        print(f"Obstacles: {len(self.robot.obstacles)}")
        print(f"Lines: {len(self.robot.lines)}")
        print()
        
        # Connect mock robot
        await self.mock_robot.connect()
        
        return True
    
    async def shutdown(self):
        """Shutdown and cleanup"""
        print("\nShutting down simulation...")
        await self.mock_robot.disconnect()
        print("✅ Shutdown complete")
    
    def detect_targets_simulated(self) -> List:
        """Detect targets in simulated environment"""
        # In simulation, we know target positions directly
        # Convert to Target objects
        targets = []
        for i, (x, y, color) in enumerate(self.target_positions):
            from target_detection import Target
            target = Target(
                center=(x * 100, y * 100),  # Convert to pixel-like coordinates
                world_pos=(x, y),
                area=1000,
                color=color,
                confidence=1.0,
                id=i
            )
            targets.append(target)
        
        return targets
    
    def localize_robot_simulated(self):
        """Localize robot in simulated environment"""
        from robot_localization import RobotPose
        
        pos = self.robot.get_position()
        theta = self.robot.get_orientation()
        
        pose = RobotPose(
            x=pos[0],
            y=pos[1],
            theta=theta,
            confidence=1.0
        )
        
        return pose
    
    def check_line_stopping(self) -> bool:
        """
        Check if robot should stop (crossed target line)
        
        Returns:
            True if should stop
        """
        # Check each line
        for i in range(len(self.robot.lines)):
            if not self.line_crossed[i]:
                if self.robot.check_line_crossing(i):
                    self.line_crossed[i] = True
                    print(f"\n✓ Crossed line {i+1}")
                    
                    # Check if this is the target line
                    if i == (self.stop_at_line - 1):
                        print(f"\n🎯 STOPPING at line {self.stop_at_line}!")
                        return True
        
        return False
    
    async def navigation_loop(self, max_iterations: int = 1000):
        """Main navigation control loop"""
        print("\n" + "=" * 60)
        print("Starting Navigation Loop")
        print("=" * 60)
        print("\nControls:")
        print("  - Close window to stop")
        print()
        
        self.running = True
        iteration = 0
        
        # Animation setup
        plt.ion()  # Interactive mode
        
        try:
            while self.running and iteration < max_iterations:
                iteration += 1
                
                # Detect targets
                targets = self.detect_targets_simulated()
                self.current_targets = targets
                
                # Localize robot
                pose = self.localize_robot_simulated()
                self.current_pose = pose
                
                # Check line stopping
                if self.check_line_stopping():
                    self.navigation_controller.state = NavigationState.ARRIVED
                    self.robot.set_motors(0, 0)
                    print("\n✅ Challenge complete: Stopped at target line!")
                    break
                
                # Select closest target
                if targets and pose:
                    closest_target = min(targets,
                                        key=lambda t: np.sqrt(
                                            (pose.x - t.world_pos[0])**2 +
                                            (pose.y - t.world_pos[1])**2
                                        ))
                    
                    # Set target
                    self.navigation_controller.set_target(closest_target.world_pos)
                    
                    # Plan path
                    self.path_planner.replan_path(
                        (pose.x, pose.y),
                        closest_target.world_pos
                    )
                    
                    # Get next waypoint
                    waypoint = self.path_planner.get_next_waypoint()
                    if waypoint:
                        self.navigation_controller.set_waypoint((waypoint.x, waypoint.y))
                
                # Update navigation controller
                if self.navigation_controller and self.current_pose:
                    command = self.navigation_controller.update(
                        (self.current_pose.x, self.current_pose.y),
                        self.current_pose.theta,
                        dt=0.1
                    )
                    
                    # Apply command
                    self.mock_robot.set_motors(command.left_motor, command.right_motor)
                    self.mock_robot.set_servo(command.servo_angle)
                    await self.mock_robot.send()
                
                # Update visualization
                self._update_visualization()
                
                # Small delay
                await asyncio.sleep(0.1)
                
                # Check if arrived
                if self.navigation_controller.is_arrived():
                    print("\n✅ Arrived at target!")
                    await asyncio.sleep(1)
                    break
        
        except KeyboardInterrupt:
            print("\n\n⚠️  Interrupted by user")
        except Exception as e:
            print(f"\n\n❌ Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            plt.ioff()
    
    def _update_visualization(self):
        """Update visualization"""
        # Clear and redraw
        self.visualizer.clear()
        
        # Draw robot
        state = self.robot.get_state()
        self.visualizer.draw_robot(state)
        
        # Draw targets
        target_list = [(t.world_pos[0], t.world_pos[1], t.color) 
                      for t in self.current_targets]
        self.visualizer.draw_targets(target_list)
        
        # Draw path
        if self.path_planner.current_path:
            path = [(wp.x, wp.y) for wp in self.path_planner.current_path]
            self.path_history.append(self.robot.get_position())
            if len(self.path_history) > 1:
                self.visualizer.draw_path(self.path_history)
        
        # Update status
        state_name = self.navigation_controller.get_state().value if self.navigation_controller else "UNKNOWN"
        status = f"State: {state_name}\n"
        if self.current_pose:
            status += f"Pos: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})\n"
            status += f"Theta: {self.current_pose.theta:.2f} rad\n"
        status += f"Lines crossed: {sum(self.line_crossed)}/{len(self.robot.lines)}\n"
        status += f"Stop at line: {self.stop_at_line}"
        
        self.visualizer.update_status(status)
        
        # Refresh plot
        plt.draw()
        plt.pause(0.01)


async def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Simulated Navigation System')
    parser.add_argument('--stop-at-line', type=int, default=1, choices=[1, 2, 3],
                       help='Which line to stop at (1, 2, or 3)')
    parser.add_argument('--initial-x', type=float, default=0.5,
                       help='Initial X position (meters)')
    parser.add_argument('--initial-y', type=float, default=0.5,
                       help='Initial Y position (meters)')
    parser.add_argument('--initial-theta', type=float, default=0.0,
                       help='Initial orientation (radians)')
    parser.add_argument('--max-iterations', type=int, default=1000,
                       help='Maximum navigation iterations')
    
    args = parser.parse_args()
    
    # Create navigation system
    nav = SimulatedNavigation(
        stop_at_line=args.stop_at_line,
        initial_pos=(args.initial_x, args.initial_y),
        initial_theta=args.initial_theta
    )
    
    # Initialize
    if not await nav.initialize():
        print("❌ Initialization failed!")
        return
    
    try:
        # Run navigation
        await nav.navigation_loop(max_iterations=args.max_iterations)
    finally:
        await nav.shutdown()
        print("\nPress Enter to close...")
        input()


if __name__ == "__main__":
    asyncio.run(main())


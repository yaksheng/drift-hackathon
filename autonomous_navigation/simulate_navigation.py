#!/usr/bin/env python3
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
from path_planner import PathPlanner, Obstacle
from navigation_controller import NavigationController, NavigationState
from obstacle_map import ObstacleMap, get_default_obstacle_map


class SimulatedNavigation:
    """Autonomous navigation in simulated environment"""
    
    def __init__(self,
                 stop_at_line: int = 1,  # 1, 2, or 3 (milestone, but goal is to reach top)
                 initial_pos: Tuple[float, float] = (0.5, 0.5),
                 initial_theta: float = 0.0,
                 target_positions: Optional[List[Tuple[float, float]]] = None,
                 obstacles: Optional[List[Tuple[float, float, float]]] = None,
                 lines: Optional[List[Tuple[float, float, float, float]]] = None,
                 goal_y: float = 3.5):  # Goal: reach top of arena
        """
        Initialize simulated navigation
        
        Args:
            stop_at_line: Which line to track crossing (1, 2, or 3) - milestone only
            initial_pos: Starting robot position
            initial_theta: Starting robot orientation
            target_positions: List of target positions (x, y)
            obstacles: List of obstacles (x, y, radius)
            lines: List of lines (x1, y1, x2, y2)
            goal_y: Y coordinate to reach (top of arena)
        """
        self.stop_at_line = stop_at_line
        self.goal_y = goal_y  # Objective: reach this Y coordinate (top of frame)
        
        # Arena bounds (based on typical arena size)
        arena_bounds = ((0, 0), (2.5, 4.0))
        
        # Create simulated robot
        self.robot = SimulatedRobot(
            initial_pos=initial_pos,
            initial_theta=initial_theta,
            arena_bounds=arena_bounds
        )
        
        # Initialize path planner first
        self.path_planner = PathPlanner(arena_bounds=arena_bounds)
        
        # Load pre-mapped obstacles (known positions, variable properties)
        self.obstacle_map = get_default_obstacle_map()
        pre_mapped_obstacles = self.obstacle_map.get_obstacles()
        
        # Add pre-mapped obstacles to robot and path planner
        if obstacles:
            # Use provided obstacles if specified
            for x, y, radius in obstacles:
                self.robot.add_obstacle(x, y, radius)
                # Also add to path planner
                self.path_planner.add_obstacle(Obstacle(x=x, y=y, radius=radius, confidence=1.0))
        else:
            # Use pre-mapped obstacles (known positions)
            for obs in pre_mapped_obstacles:
                self.robot.add_obstacle(obs.x, obs.y, obs.radius)
                self.path_planner.add_obstacle(obs)
        
        # Add default lines if none provided
        if lines:
            for x1, y1, x2, y2 in lines:
                self.robot.add_line(x1, y1, x2, y2)
        else:
            # Default lines (horizontal lines across arena)
            self.robot.add_line(0.0, 1.0, 2.5, 1.0)  # Line 1
            self.robot.add_line(0.0, 2.0, 2.5, 2.0)  # Line 2
            self.robot.add_line(0.0, 3.0, 2.5, 3.0)  # Line 3
        
        # Main goal: Blue line at the top (middle of arena width)
        # This represents the goal line from the overhead camera
        self.goal_line = (1.25, goal_y)  # Center X, top Y - this is the blue line goal
        self.goal_line_y = goal_y
        
        # Add default targets if none provided
        # Main goal is the blue line at top - represented as a target for navigation
        if target_positions is None:
            target_positions = [
                (1.25, goal_y, 'blue'),  # Main goal: blue line at top center
                (2.0, 3.2, 'blue'),      # Intermediate waypoint
                (0.5, 2.8, 'blue'),     # Intermediate waypoint
            ]
        
        self.target_positions = target_positions
        self.main_goal = target_positions[0]  # Primary goal: blue line at top
        
        # Create visualizer
        self.visualizer = ArenaVisualizer(arena_bounds)
        self.visualizer.draw_arena()
        self.visualizer.draw_obstacles(self.robot.obstacles)
        self.visualizer.draw_lines(self.robot.lines, stop_line_index=stop_at_line - 1)
        
        # Draw goal blue line at top
        self.visualizer.draw_goal_line((0.0, goal_y), (2.5, goal_y))
        
        # Create mock robot interface
        self.mock_robot = MockGalaxyRVR(self.robot)
        
        # Initialize navigation modules (path_planner already initialized above)
        self.target_detector = TargetDetector()
        self.robot_localizer = RobotLocalizer()
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
        print(f"\n🎯 Objective: Reach middle blue line at top (y = {self.goal_y:.1f}m)")
        print(f"📍 Initial position: {self.robot.get_position()}")
        print(f"🎯 Main goal: Blue line at ({self.goal_line[0]:.2f}, {self.goal_line[1]:.2f})")
        print(f"📍 Targets: {len(self.target_positions)}")
        print(f"🚧 Obstacles: {len(self.robot.obstacles)} (pre-mapped, known positions)")
        print(f"📏 Lines: {len(self.robot.lines)} (milestone tracking)")
        print(f"   - Line {self.stop_at_line} crossing will be tracked")
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
        print("\n🎮 Controls:")
        print("  - Close window to stop")
        print("  - Watch the robot navigate to the top!")
        print("  - Blue line at top = Main goal (middle blue line)")
        print("  - Green line = Robot's path")
        print("  - Red circles = Obstacles to avoid (dynamic)")
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
                
                # Check if reached blue line at top (main objective)
                # Check if robot is at the goal line Y coordinate and near center X
                if pose:
                    distance_to_line_y = abs(pose.y - self.goal_line_y)
                    distance_to_center_x = abs(pose.x - self.goal_line[0])
                    
                    # Success if within 15cm of line Y and within 30cm of center X
                    if distance_to_line_y < 0.15 and distance_to_center_x < 0.30:
                        self.navigation_controller.state = NavigationState.ARRIVED
                        self.robot.set_motors(0, 0)
                        print(f"\n🎯 SUCCESS: Reached blue line at top! (y = {pose.y:.2f}, x = {pose.x:.2f})")
                        print("✅ Challenge complete: Navigated to middle blue line while avoiding obstacles!")
                        break
                
                # Check line crossing (milestone tracking)
                line_crossed = self.check_line_stopping()
                if line_crossed and self.stop_at_line and sum(self.line_crossed) >= self.stop_at_line:
                    print(f"\n✓ Milestone: Crossed line {self.stop_at_line}")
                    # Continue to top - don't stop here
                
                # Select target - prioritize main goal if close, otherwise closest
                if targets and pose:
                    # If close to top, go directly to main goal
                    if pose.y > self.goal_y - 0.5:
                        target_pos = self.main_goal[:2]  # (x, y) of main goal
                    else:
                        # Select closest target that's ahead (higher y)
                        valid_targets = [t for t in targets if t.world_pos[1] > pose.y]
                        if valid_targets:
                            closest_target = min(valid_targets,
                                                key=lambda t: np.sqrt(
                                                    (pose.x - t.world_pos[0])**2 +
                                                    (pose.y - t.world_pos[1])**2
                                                ))
                            target_pos = closest_target.world_pos
                        else:
                            # Fallback to main goal
                            target_pos = self.main_goal[:2]
                    
                    # Set target
                    self.navigation_controller.set_target(target_pos)
                    
                    # Plan path to target
                    self.path_planner.replan_path(
                        (pose.x, pose.y),
                        target_pos
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
                
                # Update obstacles dynamically in simulation (from sensor readings)
                if self.current_pose and self.robot:
                    state = self.robot.get_state()
                    # Update path planner with sensor obstacles
                    self.path_planner.update_obstacle_from_sensors(
                        (self.current_pose.x, self.current_pose.y),
                        self.current_pose.theta,
                        state.ultrasonic_distance,
                        state.ir_left,
                        state.ir_right
                    )
                    # Update visualizer with current obstacles
                    self.visualizer.draw_obstacles(self.robot.obstacles)
                    # Also draw path planner obstacles (sensor-detected)
                    if self.path_planner.obstacles:
                        sensor_obstacles = [(obs.x, obs.y, obs.radius) 
                                          for obs in self.path_planner.obstacles 
                                          if obs.confidence < 0.9]
                        if sensor_obstacles:
                            self.visualizer.draw_sensor_obstacles(sensor_obstacles)
                
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
        
        # Draw arena
        self.visualizer.draw_arena()
        
        # Draw obstacles with colors
        obstacle_colors = []
        if hasattr(self, 'obstacle_map'):
            pre_mapped = self.obstacle_map.get_obstacles()
            if len(pre_mapped) == len(self.robot.obstacles):
                obstacle_colors = [obs.color or 'red' for obs in pre_mapped]
            else:
                obstacle_colors = ['red'] * len(self.robot.obstacles)
        else:
            obstacle_colors = ['red'] * len(self.robot.obstacles)
        self.visualizer.draw_obstacles(self.robot.obstacles, obstacle_colors=obstacle_colors)
        
        # Draw robot
        state = self.robot.get_state()
        self.visualizer.draw_robot(state)
        
        # Draw targets
        target_list = [(t.world_pos[0], t.world_pos[1], t.color) 
                      for t in self.current_targets]
        self.visualizer.draw_targets(target_list)
        
        # Draw path (robot trajectory)
        self.path_history.append(self.robot.get_position())
        if len(self.path_history) > 1:
            self.visualizer.draw_path(self.path_history)
        
        # Draw planned waypoints
        if self.path_planner.current_path:
            waypoint_path = [(wp.x, wp.y) for wp in self.path_planner.current_path if not wp.reached]
            if waypoint_path:
                # Draw waypoints as small squares
                wp_x = [p[0] for p in waypoint_path]
                wp_y = [p[1] for p in waypoint_path]
                self.visualizer.ax.scatter(wp_x, wp_y, c='orange', s=100, marker='s', 
                              alpha=0.6, label='Waypoints', zorder=2)
                # Draw line to next waypoint
                if self.current_pose and waypoint_path:
                    self.visualizer.ax.plot([self.current_pose.x, waypoint_path[0][0]], 
                               [self.current_pose.y, waypoint_path[0][1]],
                               'r--', linewidth=2, alpha=0.5, label='Next waypoint')
        
        # Update status
        state_name = self.navigation_controller.get_state().value if self.navigation_controller else "UNKNOWN"
        status = f"🎯 Goal: Blue line at top (y={self.goal_y:.1f}m)\n"
        status += f"State: {state_name}\n"
        if self.current_pose:
            status += f"Position: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})m\n"
            dist_to_line = abs(self.current_pose.y - self.goal_line_y)
            dist_to_center = abs(self.current_pose.x - self.goal_line[0])
            status += f"Dist to line: {dist_to_line:.2f}m\n"
            status += f"Dist to center: {dist_to_center:.2f}m\n"
            status += f"Progress: {(self.current_pose.y / self.goal_y * 100):.1f}%\n"
        status += f"Lines crossed: {sum(self.line_crossed)}/{len(self.robot.lines)}\n"
        if self.path_planner.current_path:
            status += f"Waypoints: {len([w for w in self.path_planner.current_path if not w.reached])} remaining"
        
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
    parser.add_argument('--no-wait', action='store_true',
                       help='Do not wait for user input at end (for automated testing)')
    
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
        if not args.no_wait:
            print("\nPress Enter to close...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass  # Non-interactive mode or interrupted


if __name__ == "__main__":
    asyncio.run(main())


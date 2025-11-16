#!/usr/bin/env python3
"""
Visual Robot Tests

Runs visual tests showing the robot navigating in the simulated arena.
Shows robot movement, path planning, obstacle avoidance, and goal reaching.
"""

import asyncio
import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for headless testing
import matplotlib.pyplot as plt
plt.ion()  # Turn on interactive mode for real-time updates

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from simulator import SimulatedRobot
from simulator_visualization import ArenaVisualizer
from mock_robot import MockGalaxyRVR
from target_detection import TargetDetector
from robot_localization import RobotLocalizer
from path_planner import PathPlanner
from navigation_controller import NavigationController, NavigationState


class VisualRobotTester:
    """Visual tests for robot navigation"""
    
    def __init__(self):
        self.visualizer = None
        self.robot = None
        self.mock_robot = None
        self.planner = None
        self.controller = None
    
    def setup_test_environment(self, initial_pos=(0.5, 0.5), initial_theta=0.0):
        """Set up test environment"""
        # Create simulator
        self.robot = SimulatedRobot(
            initial_pos=initial_pos,
            initial_theta=initial_theta
        )
        
        # Create mock robot
        self.mock_robot = MockGalaxyRVR(self.robot)
        
        # Create navigation components
        self.planner = PathPlanner()
        self.controller = NavigationController(self.mock_robot)
        
        # Create visualizer
        self.visualizer = ArenaVisualizer(
            arena_bounds=((0, 0), (2.5, 4.0)),
            figsize=(14, 10)
        )
        
        # Draw arena
        self.visualizer.draw_arena()
    
    async def test_basic_movement(self):
        """Test 1: Basic robot movement visualization"""
        print("\n" + "="*60)
        print("VISUAL TEST 1: Basic Robot Movement")
        print("="*60)
        print("Watch the robot move forward and rotate...")
        
        self.setup_test_environment()
        
        # Draw initial position
        state = self.robot.get_state()
        self.visualizer.draw_robot(state)
        self.visualizer.update_status("Test 1: Basic Movement\nInitial Position")
        plt.draw()
        plt.pause(1.0)
        
        # Move forward
        print("Moving forward...")
        for i in range(10):
            self.robot.set_motors(50, 50)
            self.robot.update(0.1)
            state = self.robot.get_state()
            self.visualizer.draw_robot(state)
            self.visualizer.update_status(f"Test 1: Moving Forward\nPosition: ({state.x:.2f}, {state.y:.2f})")
            plt.draw()
            plt.pause(0.1)
        
        # Rotate
        print("Rotating...")
        for i in range(5):
            self.robot.set_motors(-50, 50)
            self.robot.update(0.1)
            state = self.robot.get_state()
            self.visualizer.draw_robot(state)
            self.visualizer.update_status(f"Test 1: Rotating\nAngle: {state.theta:.2f} rad")
            plt.draw()
            plt.pause(0.1)
        
        print("✅ Test 1 Complete!")
        plt.pause(2.0)
        plt.close()
    
    async def test_path_planning_visual(self):
        """Test 2: Path planning visualization"""
        print("\n" + "="*60)
        print("VISUAL TEST 2: Path Planning")
        print("="*60)
        print("Watch the robot plan and follow a path...")
        
        self.setup_test_environment()
        
        # Set goal
        start = (0.5, 0.5)
        goal = (2.0, 3.0)
        
        # Plan path
        self.planner.replan_path(start, goal)
        
        # Draw goal
        self.visualizer.draw_targets([(goal[0], goal[1], 'blue')])
        
        # Draw path
        if self.planner.current_path:
            path = [(wp.x, wp.y) for wp in self.planner.current_path]
            self.visualizer.draw_path(path)
        
        # Draw robot at start
        state = self.robot.get_state()
        self.visualizer.draw_robot(state)
        self.visualizer.update_status("Test 2: Path Planning\nPath planned from start to goal")
        plt.draw()
        plt.pause(2.0)
        
        print("✅ Test 2 Complete!")
        plt.pause(2.0)
        plt.close()
    
    async def test_obstacle_avoidance_visual(self):
        """Test 3: Obstacle avoidance visualization"""
        print("\n" + "="*60)
        print("VISUAL TEST 3: Obstacle Avoidance")
        print("="*60)
        print("Watch the robot avoid obstacles...")
        
        self.setup_test_environment()
        
        # Add obstacles
        obstacles = [
            (1.0, 1.0, 0.2),
            (1.5, 2.0, 0.15),
            (2.0, 1.5, 0.2)
        ]
        for obs in obstacles:
            self.robot.add_obstacle(obs[0], obs[1], obs[2])
            self.planner.obstacles.append(
                type('Obstacle', (), {'x': obs[0], 'y': obs[1], 'radius': obs[2], 'confidence': 1.0})()
            )
        
        # Draw obstacles
        self.visualizer.draw_obstacles(obstacles)
        
        # Set goal
        start = (0.5, 0.5)
        goal = (2.0, 3.0)
        
        # Plan path (should avoid obstacles)
        self.planner.replan_path(start, goal)
        
        # Draw goal
        self.visualizer.draw_targets([(goal[0], goal[1], 'blue')])
        
        # Draw path
        if self.planner.current_path:
            path = [(wp.x, wp.y) for wp in self.planner.current_path]
            self.visualizer.draw_path(path)
        
        # Draw robot
        state = self.robot.get_state()
        self.visualizer.draw_robot(state)
        self.visualizer.update_status("Test 3: Obstacle Avoidance\nPath planned avoiding obstacles")
        plt.draw()
        plt.pause(3.0)
        
        print("✅ Test 3 Complete!")
        plt.pause(2.0)
        plt.close()
    
    async def test_navigation_to_goal_visual(self):
        """Test 4: Full navigation to goal with visualization"""
        print("\n" + "="*60)
        print("VISUAL TEST 4: Navigation to Goal")
        print("="*60)
        print("Watch the robot navigate to the goal...")
        
        self.setup_test_environment()
        
        # Add some obstacles
        obstacles = [
            (1.5, 1.5, 0.2),
            (2.0, 2.5, 0.15)
        ]
        for obs in obstacles:
            self.robot.add_obstacle(obs[0], obs[1], obs[2])
            self.planner.obstacles.append(
                type('Obstacle', (), {'x': obs[0], 'y': obs[1], 'radius': obs[2], 'confidence': 1.0})()
            )
        
        # Draw obstacles
        self.visualizer.draw_obstacles(obstacles)
        
        # Set goal
        goal = (2.0, 3.5)  # Top of arena
        self.controller.set_target(goal)
        
        # Draw goal
        self.visualizer.draw_targets([(goal[0], goal[1], 'blue')])
        
        # Draw goal line
        self.visualizer.draw_goal_line((0.5, 3.5), (2.0, 3.5))
        
        # Navigation loop
        path_history = []
        max_iterations = 100
        
        print("Starting navigation...")
        for i in range(max_iterations):
            # Get current state
            state = self.robot.get_state()
            current_pos = (state.x, state.y)
            current_theta = state.theta
            
            # Add to path history
            path_history.append(current_pos)
            
            # Check if reached goal
            distance_to_goal = np.sqrt(
                (current_pos[0] - goal[0])**2 + 
                (current_pos[1] - goal[1])**2
            )
            
            if distance_to_goal < 0.2:
                print(f"✅ Goal reached at iteration {i}!")
                break
            
            # Plan path
            self.planner.replan_path(current_pos, goal)
            waypoint = self.planner.get_next_waypoint()
            
            if waypoint:
                self.controller.set_waypoint((waypoint.x, waypoint.y))
                self.controller.state = NavigationState.NAVIGATING
            
            # Update controller
            command = self.controller.update(current_pos, current_theta, 0.1)
            
            # Apply to robot
            self.robot.set_motors(command.left_motor, command.right_motor)
            self.robot.update(0.1)
            
            # Update visualization
            self.visualizer.clear()
            self.visualizer.draw_arena()
            self.visualizer.draw_obstacles(obstacles)
            self.visualizer.draw_targets([(goal[0], goal[1], 'blue')])
            self.visualizer.draw_goal_line((0.5, 3.5), (2.0, 3.5))
            self.visualizer.draw_path(path_history)
            
            # Draw waypoints
            if self.planner.current_path:
                waypoint_path = [(wp.x, wp.y) for wp in self.planner.current_path if not wp.reached]
                if waypoint_path:
                    wp_x = [p[0] for p in waypoint_path]
                    wp_y = [p[1] for p in waypoint_path]
                    self.visualizer.ax.scatter(wp_x, wp_y, c='orange', s=100, marker='s', 
                                  alpha=0.6, label='Waypoints', zorder=2)
            
            state = self.robot.get_state()
            self.visualizer.draw_robot(state)
            
            # Update status
            status = f"Test 4: Navigation to Goal\n"
            status += f"Iteration: {i}/{max_iterations}\n"
            status += f"Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})\n"
            status += f"Distance to goal: {distance_to_goal:.2f}m\n"
            status += f"State: {self.controller.state.value}"
            self.visualizer.update_status(status)
            
            plt.draw()
            plt.pause(0.05)  # Small delay for visualization
            
            if i % 10 == 0:
                print(f"  Iteration {i}: pos=({current_pos[0]:.2f}, {current_pos[1]:.2f}), "
                      f"dist={distance_to_goal:.2f}m")
        
        final_state = self.robot.get_state()
        final_pos = (final_state.x, final_state.y)
        final_distance = np.sqrt(
            (final_pos[0] - goal[0])**2 + 
            (final_pos[1] - goal[1])**2
        )
        
        print(f"\nFinal position: ({final_pos[0]:.2f}, {final_pos[1]:.2f})")
        print(f"Distance to goal: {final_distance:.2f}m")
        print("✅ Test 4 Complete!")
        
        # Keep window open
        self.visualizer.update_status(f"Test 4: Complete!\nFinal distance: {final_distance:.2f}m\nPress Enter to continue...")
        plt.draw()
        input("\nPress Enter to close visualization...")
        plt.close()
    
    async def test_sensor_obstacle_detection_visual(self):
        """Test 5: Sensor-based obstacle detection visualization"""
        print("\n" + "="*60)
        print("VISUAL TEST 5: Sensor Obstacle Detection")
        print("="*60)
        print("Watch the robot detect obstacles with sensors...")
        
        self.setup_test_environment()
        
        # Add obstacle in front
        obstacle = (1.0, 0.5, 0.2)
        self.robot.add_obstacle(obstacle[0], obstacle[1], obstacle[2])
        
        # Draw static obstacles
        self.visualizer.draw_obstacles([obstacle])
        
        # Move robot towards obstacle
        path_history = []
        for i in range(15):
            self.robot.set_motors(50, 50)
            self.robot.update(0.1)
            
            state = self.robot.get_state()
            current_pos = (state.x, state.y)
            path_history.append(current_pos)
            
            # Get sensor readings
            ultrasonic = state.ultrasonic_distance
            ir_left = state.ir_left
            ir_right = state.ir_right
            
            # Update sensor obstacles
            sensor_obstacles = []
            if ultrasonic is not None and ultrasonic < 50:
                dist_m = ultrasonic / 100.0
                obs_x = current_pos[0] + dist_m * np.cos(state.theta)
                obs_y = current_pos[1] + dist_m * np.sin(state.theta)
                sensor_obstacles.append((obs_x, obs_y, 0.15))
            
            # Update visualization
            self.visualizer.clear()
            self.visualizer.draw_arena()
            self.visualizer.draw_obstacles([obstacle])
            if sensor_obstacles:
                self.visualizer.draw_sensor_obstacles(sensor_obstacles)
            self.visualizer.draw_path(path_history)
            self.visualizer.draw_robot(state)
            
            status = f"Test 5: Sensor Obstacle Detection\n"
            status += f"Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})\n"
            status += f"Ultrasonic: {ultrasonic:.1f}cm\n"
            status += f"IR Left: {ir_left}, IR Right: {ir_right}"
            self.visualizer.update_status(status)
            
            plt.draw()
            plt.pause(0.1)
        
        print("✅ Test 5 Complete!")
        plt.pause(2.0)
        plt.close()
    
    async def run_all_visual_tests(self):
        """Run all visual tests"""
        print("\n" + "="*60)
        print("VISUAL ROBOT TESTS")
        print("="*60)
        print("Running visual tests to demonstrate robot capabilities...")
        print("Each test will show a matplotlib window.")
        print("="*60)
        
        try:
            await self.test_basic_movement()
            await self.test_path_planning_visual()
            await self.test_obstacle_avoidance_visual()
            await self.test_sensor_obstacle_detection_visual()
            await self.test_navigation_to_goal_visual()
            
            print("\n" + "="*60)
            print("✅ ALL VISUAL TESTS COMPLETE!")
            print("="*60)
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Tests interrupted by user")
        except Exception as e:
            print(f"\n\n❌ Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            plt.close('all')


async def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Visual Robot Tests')
    parser.add_argument('--test', type=int, choices=[1, 2, 3, 4, 5],
                       help='Run specific test (1-5)')
    parser.add_argument('--all', action='store_true',
                       help='Run all visual tests')
    
    args = parser.parse_args()
    
    tester = VisualRobotTester()
    
    if args.test:
        # Run specific test
        test_map = {
            1: tester.test_basic_movement,
            2: tester.test_path_planning_visual,
            3: tester.test_obstacle_avoidance_visual,
            4: tester.test_navigation_to_goal_visual,
            5: tester.test_sensor_obstacle_detection_visual
        }
        await test_map[args.test]()
    else:
        # Run all tests
        await tester.run_all_visual_tests()


if __name__ == "__main__":
    asyncio.run(main())


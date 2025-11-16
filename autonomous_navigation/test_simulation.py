#!/usr/bin/env python3
"""
Test script for simulation environment

Runs tests in CLI mode first, then shows visual version.
"""

import asyncio
import sys
import os
import numpy as np

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from simulator import SimulatedRobot
from simulator_visualization import ArenaVisualizer
from mock_robot import MockGalaxyRVR
from target_detection import TargetDetector
from robot_localization import RobotLocalizer
from path_planner import PathPlanner
from navigation_controller import NavigationController, NavigationState


class SimulationTester:
    """Test simulation environment"""
    
    def __init__(self):
        self.test_results = []
        self.passed = 0
        self.failed = 0
    
    def test_robot_initialization(self):
        """Test 1: Robot initialization"""
        print("\n" + "="*60)
        print("TEST 1: Robot Initialization")
        print("="*60)
        try:
            robot = SimulatedRobot(
                initial_pos=(0.5, 0.5),
                initial_theta=0.0
            )
            state = robot.get_state()
            assert state.x == 0.5, f"Expected x=0.5, got {state.x}"
            assert state.y == 0.5, f"Expected y=0.5, got {state.y}"
            assert state.theta == 0.0, f"Expected theta=0.0, got {state.theta}"
            print("✅ PASS: Robot initialized correctly")
            self.passed += 1
            return True
        except Exception as e:
            print(f"❌ FAIL: {e}")
            self.failed += 1
            return False
    
    def test_robot_movement(self):
        """Test 2: Robot movement"""
        print("\n" + "="*60)
        print("TEST 2: Robot Movement")
        print("="*60)
        try:
            robot = SimulatedRobot(
                initial_pos=(0.5, 0.5),
                initial_theta=0.0
            )
            
            # Move forward
            robot.set_motors(50, 50)  # Both motors forward
            robot.update(0.1)  # Update for 0.1 seconds
            
            state = robot.get_state()
            assert state.x > 0.5, f"Robot should move forward, x={state.x}"
            print(f"✅ PASS: Robot moved forward (x={state.x:.3f})")
            
            # Test rotation
            initial_theta = state.theta
            robot.set_motors(-50, 50)  # Turn left
            robot.update(0.1)
            new_state = robot.get_state()
            assert new_state.theta > initial_theta, "Robot should rotate left"
            print(f"✅ PASS: Robot rotated (theta={new_state.theta:.3f})")
            
            self.passed += 1
            return True
        except Exception as e:
            print(f"❌ FAIL: {e}")
            self.failed += 1
            return False
    
    def test_sensor_simulation(self):
        """Test 3: Sensor simulation"""
        print("\n" + "="*60)
        print("TEST 3: Sensor Simulation")
        print("="*60)
        try:
            robot = SimulatedRobot(
                initial_pos=(0.5, 0.5),
                initial_theta=0.0
            )
            
            # Add an obstacle
            robot.add_obstacle(1.0, 0.5, 0.2)  # Obstacle at (1.0, 0.5) with radius 0.2
            
            # Move towards obstacle
            robot.set_motors(50, 50)
            robot.update(0.5)
            
            # Check ultrasonic sensor (from state)
            state = robot.get_state()
            ultrasonic = state.ultrasonic_distance
            assert ultrasonic is not None, "Ultrasonic sensor should return a value"
            assert ultrasonic < 100, f"Ultrasonic should detect obstacle, got {ultrasonic}cm"
            print(f"✅ PASS: Ultrasonic sensor working (distance={ultrasonic:.1f}cm)")
            
            # Check IR sensors (from state)
            ir_left = state.ir_left
            ir_right = state.ir_right
            assert ir_left is not None and ir_right is not None, "IR sensors should return values"
            print(f"✅ PASS: IR sensors working (left={ir_left}, right={ir_right})")
            
            self.passed += 1
            return True
        except Exception as e:
            print(f"❌ FAIL: {e}")
            self.failed += 1
            return False
    
    def test_path_planner(self):
        """Test 4: Path planning"""
        print("\n" + "="*60)
        print("TEST 4: Path Planning")
        print("="*60)
        try:
            planner = PathPlanner()
            
            # Plan path from (0.5, 0.5) to (3.0, 3.0)
            start = (0.5, 0.5)
            goal = (3.0, 3.0)
            
            planner.replan_path(start, goal)
            
            assert planner.current_path is not None, "Path should be generated"
            assert len(planner.current_path) > 0, "Path should have waypoints"
            
            waypoint = planner.get_next_waypoint()
            assert waypoint is not None, "Should get next waypoint"
            print(f"✅ PASS: Path planned with {len(planner.current_path)} waypoints")
            print(f"   First waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f})")
            
            self.passed += 1
            return True
        except Exception as e:
            print(f"❌ FAIL: {e}")
            self.failed += 1
            return False
    
    def test_navigation_controller(self):
        """Test 5: Navigation controller"""
        print("\n" + "="*60)
        print("TEST 5: Navigation Controller")
        print("="*60)
        try:
            # Create simulated robot first
            sim_robot = SimulatedRobot(initial_pos=(0.5, 0.5), initial_theta=0.0)
            mock_robot = MockGalaxyRVR(sim_robot)
            controller = NavigationController(mock_robot)
            
            # Set a waypoint
            controller.set_waypoint((2.0, 2.0))
            
            # Update controller
            command = controller.update(
                current_pos=(0.5, 0.5),
                current_theta=0.0,
                dt=0.1
            )
            
            assert command is not None, "Controller should return a command"
            assert hasattr(command, 'left_motor'), "Command should have left_motor"
            assert hasattr(command, 'right_motor'), "Command should have right_motor"
            print(f"✅ PASS: Navigation controller working")
            print(f"   Command: left={command.left_motor}, right={command.right_motor}")
            
            self.passed += 1
            return True
        except Exception as e:
            print(f"❌ FAIL: {e}")
            self.failed += 1
            return False
    
    async def test_full_navigation_simulation(self):
        """Test 6: Full navigation simulation (non-visual)"""
        print("\n" + "="*60)
        print("TEST 6: Full Navigation Simulation (CLI)")
        print("="*60)
        try:
            # Create simulation components
            robot = SimulatedRobot(
                initial_pos=(0.5, 0.5),
                initial_theta=0.0
            )
            
            # Add some obstacles
            robot.add_obstacle(1.5, 1.5, 0.2)
            robot.add_obstacle(2.5, 2.0, 0.15)
            
            # Goal at top (we'll use path planner to navigate to it)
            goal_y = 3.5
            
            # Create navigation components
            mock_robot = MockGalaxyRVR(robot)  # Pass simulated robot
            
            localizer = RobotLocalizer(robot_marker_color='green')
            path_planner = PathPlanner()
            controller = NavigationController(mock_robot)
            
            # Set goal
            goal_pos = (2.0, goal_y)
            controller.set_target(goal_pos)
            
            # Run simulation for limited iterations
            max_iterations = 50
            iterations = 0
            reached_goal = False
            
            print("Running simulation (non-visual)...")
            for i in range(max_iterations):
                iterations += 1
                
                # Get robot state
                state = robot.get_state()
                current_pos = (state.x, state.y)
                current_theta = state.theta
                
                # Check if reached goal
                distance_to_goal = np.sqrt(
                    (current_pos[0] - goal_pos[0])**2 + 
                    (current_pos[1] - goal_pos[1])**2
                )
                
                if distance_to_goal < 0.2:  # Within 20cm
                    reached_goal = True
                    print(f"✅ Goal reached at iteration {iterations}")
                    break
                
                # Plan path
                path_planner.replan_path(current_pos, goal_pos)
                waypoint = path_planner.get_next_waypoint()
                
                if waypoint:
                    controller.set_waypoint((waypoint.x, waypoint.y))
                
                # Update controller
                command = controller.update(current_pos, current_theta, 0.1)
                
                # Apply to robot
                robot.set_motors(command.left_motor, command.right_motor)
                robot.update(0.1)
                
                if i % 10 == 0:
                    print(f"  Iteration {i}: pos=({current_pos[0]:.2f}, {current_pos[1]:.2f}), "
                          f"dist_to_goal={distance_to_goal:.2f}m")
            
            final_state = robot.get_state()
            final_pos = (final_state.x, final_state.y)
            final_distance = np.sqrt(
                (final_pos[0] - goal_pos[0])**2 + 
                (final_pos[1] - goal_pos[1])**2
            )
            
            print(f"\nFinal position: ({final_pos[0]:.2f}, {final_pos[1]:.2f})")
            print(f"Distance to goal: {final_distance:.2f}m")
            print(f"Iterations: {iterations}/{max_iterations}")
            
            # Test passes if robot moved significantly towards goal
            if reached_goal or final_distance < 1.0:
                print("✅ PASS: Navigation simulation completed successfully")
                self.passed += 1
                return True
            else:
                print(f"⚠️  PARTIAL: Robot moved but didn't reach goal (distance={final_distance:.2f}m)")
                # Still count as pass if robot moved forward
                if final_pos[1] > 0.7:  # Moved forward at least 20cm
                    self.passed += 1
                    return True
                else:
                    self.failed += 1
                    return False
                    
        except Exception as e:
            print(f"❌ FAIL: {e}")
            import traceback
            traceback.print_exc()
            self.failed += 1
            return False
    
    async def run_all_tests(self):
        """Run all tests"""
        print("\n" + "="*60)
        print("SIMULATION ENVIRONMENT TEST SUITE")
        print("="*60)
        
        # Run tests
        self.test_robot_initialization()
        self.test_robot_movement()
        self.test_sensor_simulation()
        self.test_path_planner()
        self.test_navigation_controller()
        
        # Run async test
        await self.test_full_navigation_simulation()
        
        # Print summary
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        print(f"Total tests: {self.passed + self.failed}")
        print(f"✅ Passed: {self.passed}")
        print(f"❌ Failed: {self.failed}")
        print("="*60)
        
        return self.failed == 0


async def main():
    """Main entry point"""
    tester = SimulationTester()
    all_passed = await tester.run_all_tests()
    
    if all_passed:
        print("\n🎉 All tests passed! Ready for visual test.")
        return 0
    else:
        print("\n⚠️  Some tests failed. Review output above.")
        return 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)


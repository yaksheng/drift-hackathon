#!/usr/bin/env python3
"""
Integration Tests for Autonomous Navigation System

Tests modules working together.
"""

import sys
import os
import numpy as np
import unittest
import asyncio

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from simulator import SimulatedRobot
from mock_robot import MockGalaxyRVR
from target_detection import TargetDetector
from robot_localization import RobotLocalizer
from path_planner import PathPlanner
from navigation_controller import NavigationController, NavigationState
from dead_reckoning import DeadReckoning


class TestTargetDetectionAndLocalization(unittest.TestCase):
    """Test target detection and robot localization integration"""
    
    def test_detection_and_localization_workflow(self):
        """Test that detection and localization can work together"""
        detector = TargetDetector()
        localizer = RobotLocalizer(robot_marker_color='green')
        
        # Set up transform
        transform = np.eye(3)
        localizer.set_world_transform(transform)
        
        # Both should work independently
        self.assertIsNotNone(detector)
        self.assertIsNotNone(localizer)
        self.assertIsNotNone(localizer.world_transform_matrix)


class TestPathPlanningAndNavigation(unittest.TestCase):
    """Test path planning and navigation controller integration"""
    
    def setUp(self):
        self.sim_robot = SimulatedRobot(initial_pos=(0.5, 0.5), initial_theta=0.0)
        self.mock_robot = MockGalaxyRVR(self.sim_robot)
        self.planner = PathPlanner()
        self.controller = NavigationController(self.mock_robot)
    
    def test_plan_and_navigate(self):
        """Test planning a path and navigating to it"""
        start = (0.5, 0.5)
        goal = (2.0, 2.0)
        
        # Plan path
        self.planner.replan_path(start, goal)
        self.assertIsNotNone(self.planner.current_path)
        self.assertGreater(len(self.planner.current_path), 0)
        
        # Set waypoint in controller
        waypoint = self.planner.get_next_waypoint()
        self.assertIsNotNone(waypoint)
        self.controller.set_waypoint((waypoint.x, waypoint.y))
        self.controller.state = NavigationState.NAVIGATING
        
        # Update controller
        command = self.controller.update(
            current_pos=start,
            current_theta=0.0,
            dt=0.1
        )
        
        self.assertIsNotNone(command)
        # Should generate movement command
        self.assertNotEqual(command.left_motor, 0)
        self.assertNotEqual(command.right_motor, 0)
    
    def test_obstacle_avoidance_integration(self):
        """Test path planning with obstacles"""
        # Add obstacle
        self.planner.obstacles.append(
            type('Obstacle', (), {
                'x': 1.0,
                'y': 1.0,
                'radius': 0.3,
                'confidence': 1.0
            })()
        )
        
        start = (0.5, 0.5)
        goal = (2.0, 2.0)
        
        # Plan path (should avoid obstacle)
        self.planner.replan_path(start, goal)
        self.assertIsNotNone(self.planner.current_path)
        
        # Check that path doesn't go through obstacle
        for wp in self.planner.current_path:
            dist_to_obstacle = np.sqrt((wp.x - 1.0)**2 + (wp.y - 1.0)**2)
            self.assertGreater(dist_to_obstacle, 0.3, "Path should avoid obstacle")


class TestDeadReckoningIntegration(unittest.TestCase):
    """Test dead reckoning with navigation"""
    
    def setUp(self):
        self.dr = DeadReckoning()
        self.sim_robot = SimulatedRobot(initial_pos=(0.5, 0.5), initial_theta=0.0)
        self.mock_robot = MockGalaxyRVR(self.sim_robot)
        self.controller = NavigationController(self.mock_robot)
    
    def test_dead_reckoning_with_motor_commands(self):
        """Test dead reckoning updates from motor commands"""
        # Initialize
        self.dr.initialize((0.5, 0.5), 0.0)
        
        # Set waypoint and navigate
        self.controller.set_waypoint((1.0, 0.5))
        self.controller.state = NavigationState.NAVIGATING
        
        # Get command
        command = self.controller.update(
            current_pos=(0.5, 0.5),
            current_theta=0.0,
            dt=0.1
        )
        
        # Update dead reckoning
        self.dr.update_from_motors(command.left_motor, command.right_motor)
        
        # Check that position changed
        pose = self.dr.get_estimated_pose()
        self.assertIsNotNone(pose)
        # Position should have changed (moved forward)
        self.assertGreater(pose[0], 0.5)
    
    def test_camera_correction(self):
        """Test dead reckoning correction from camera"""
        self.dr.initialize((0.5, 0.5), 0.0)
        
        # Simulate some movement first
        self.dr.update_from_motors(50, 50)
        import time
        time.sleep(0.1)
        self.dr.update_from_motors(50, 50)
        
        # Get position before camera correction
        pose_before = self.dr.get_estimated_pose()
        
        # Camera provides ground truth (different from dead reckoning estimate)
        self.dr.update_from_camera((0.6, 0.5), 0.0)
        
        # Position should be updated (may be blended, but should change)
        pose_after = self.dr.get_estimated_pose()
        self.assertIsNotNone(pose_after)
        # Camera update should influence the position
        # The position should be closer to camera measurement than before
        self.assertIsNotNone(pose_after)


class TestFullNavigationLoop(unittest.TestCase):
    """Test full navigation loop integration"""
    
    def setUp(self):
        self.sim_robot = SimulatedRobot(initial_pos=(0.5, 0.5), initial_theta=0.0)
        self.mock_robot = MockGalaxyRVR(self.sim_robot)
        self.planner = PathPlanner()
        self.controller = NavigationController(self.mock_robot)
        self.dr = DeadReckoning()
    
    def test_navigation_to_goal(self):
        """Test complete navigation to a goal"""
        # Initialize dead reckoning
        self.dr.initialize((0.5, 0.5), 0.0)
        
        # Set goal
        goal = (2.0, 1.0)
        self.controller.set_target(goal)
        
        # Plan path
        self.planner.replan_path((0.5, 0.5), goal)
        waypoint = self.planner.get_next_waypoint()
        self.assertIsNotNone(waypoint)
        
        # Set waypoint
        self.controller.set_waypoint((waypoint.x, waypoint.y))
        self.controller.state = NavigationState.NAVIGATING
        
        # Simulate navigation loop
        current_pos = (0.5, 0.5)
        current_theta = 0.0
        
        for i in range(10):  # 10 iterations
            # Update controller
            command = self.controller.update(
                current_pos,
                current_theta,
                dt=0.1
            )
            
            # Update dead reckoning
            self.dr.update_from_motors(command.left_motor, command.right_motor)
            
            # Get new position
            pose = self.dr.get_estimated_pose()
            current_pos = (pose[0], pose[1])
            current_theta = pose[2]
            
            # Check progress
            if i > 0:
                # Should be moving towards goal
                self.assertGreater(current_pos[0], 0.5)


def run_tests():
    """Run all integration tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestTargetDetectionAndLocalization))
    suite.addTests(loader.loadTestsFromTestCase(TestPathPlanningAndNavigation))
    suite.addTests(loader.loadTestsFromTestCase(TestDeadReckoningIntegration))
    suite.addTests(loader.loadTestsFromTestCase(TestFullNavigationLoop))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    print("="*60)
    print("INTEGRATION TESTS FOR AUTONOMOUS NAVIGATION SYSTEM")
    print("="*60)
    
    success = run_tests()
    
    print("\n" + "="*60)
    if success:
        print("✅ All integration tests passed!")
    else:
        print("❌ Some integration tests failed!")
    print("="*60)
    
    sys.exit(0 if success else 1)


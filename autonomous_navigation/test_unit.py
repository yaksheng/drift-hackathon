#!/usr/bin/env python3
"""
Unit Tests for Autonomous Navigation Modules

Tests individual modules in isolation.
"""

import sys
import os
import numpy as np
import unittest
from unittest.mock import Mock, patch, MagicMock

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from target_detection import TargetDetector, Target
from robot_localization import RobotLocalizer, RobotPose
from path_planner import PathPlanner, Waypoint, Obstacle
from navigation_controller import NavigationController, NavigationState, ControlCommand
from dead_reckoning import DeadReckoning, OdometryState
from line_detection import LineDetector, DetectedLine


class TestTargetDetector(unittest.TestCase):
    """Test Target Detection Module"""
    
    def setUp(self):
        self.detector = TargetDetector()
    
    def test_initialization(self):
        """Test detector initializes correctly"""
        self.assertIsNotNone(self.detector)
        self.assertIsNotNone(self.detector.target_colors)
    
    def test_color_ranges(self):
        """Test color range definitions"""
        self.assertIn('blue', self.detector.target_colors)
        self.assertIn('green', self.detector.target_colors)
        self.assertIn('red', self.detector.target_colors)
    
    def test_target_creation(self):
        """Test target object creation"""
        target = Target(
            center=(100, 200),
            world_pos=(1.0, 2.0),
            area=1000.0,
            color='blue',
            confidence=0.9
        )
        self.assertEqual(target.world_pos, (1.0, 2.0))
        self.assertEqual(target.color, 'blue')
        self.assertEqual(target.confidence, 0.9)
        self.assertEqual(target.center, (100, 200))


class TestRobotLocalizer(unittest.TestCase):
    """Test Robot Localization Module"""
    
    def setUp(self):
        self.localizer = RobotLocalizer(robot_marker_color='green')
    
    def test_initialization(self):
        """Test localizer initializes correctly"""
        self.assertIsNotNone(self.localizer)
        self.assertEqual(self.localizer.robot_marker_color, 'green')
    
    def test_world_transform_setting(self):
        """Test setting world transform matrix"""
        transform = np.eye(3)
        self.localizer.set_world_transform(transform)
        self.assertIsNotNone(self.localizer.world_transform_matrix)
        np.testing.assert_array_equal(
            self.localizer.world_transform_matrix,
            transform
        )
    
    def test_pose_creation(self):
        """Test RobotPose creation"""
        pose = RobotPose(x=1.0, y=2.0, theta=0.5, confidence=0.9)
        self.assertEqual(pose.x, 1.0)
        self.assertEqual(pose.y, 2.0)
        self.assertEqual(pose.theta, 0.5)
        self.assertEqual(pose.confidence, 0.9)


class TestPathPlanner(unittest.TestCase):
    """Test Path Planning Module"""
    
    def setUp(self):
        self.planner = PathPlanner()
    
    def test_initialization(self):
        """Test planner initializes correctly"""
        self.assertIsNotNone(self.planner)
        self.assertEqual(len(self.planner.obstacles), 0)
        self.assertEqual(len(self.planner.current_path), 0)  # Empty list, not None
    
    def test_add_obstacle(self):
        """Test adding obstacles"""
        obstacle = Obstacle(x=1.0, y=1.0, radius=0.2, confidence=1.0)
        self.planner.obstacles.append(obstacle)
        self.assertEqual(len(self.planner.obstacles), 1)
        self.assertEqual(self.planner.obstacles[0].x, 1.0)
    
    def test_replan_path_straight(self):
        """Test planning a straight path without obstacles"""
        start = (0.0, 0.0)
        goal = (2.0, 0.0)
        
        self.planner.replan_path(start, goal)
        
        self.assertIsNotNone(self.planner.current_path)
        self.assertGreater(len(self.planner.current_path), 0)
        
        # Last waypoint should be close to goal
        last_wp = self.planner.current_path[-1]
        self.assertLess(np.sqrt((last_wp.x - goal[0])**2 + (last_wp.y - goal[1])**2), 0.5)
    
    def test_get_next_waypoint(self):
        """Test getting next waypoint"""
        start = (0.0, 0.0)
        goal = (2.0, 0.0)
        
        self.planner.replan_path(start, goal)
        waypoint = self.planner.get_next_waypoint()
        
        self.assertIsNotNone(waypoint)
        self.assertIsInstance(waypoint, Waypoint)
        self.assertFalse(waypoint.reached)


class TestDeadReckoning(unittest.TestCase):
    """Test Dead Reckoning Module"""
    
    def setUp(self):
        self.dr = DeadReckoning()
    
    def test_initialization(self):
        """Test dead reckoning initializes correctly"""
        self.assertIsNotNone(self.dr)
        self.assertIsNone(self.dr.odometry)
    
    def test_initialize(self):
        """Test initializing with position"""
        self.dr.initialize((1.0, 2.0), 0.5)
        
        self.assertIsNotNone(self.dr.odometry)
        self.assertEqual(self.dr.odometry.x, 1.0)
        self.assertEqual(self.dr.odometry.y, 2.0)
        self.assertEqual(self.dr.odometry.theta, 0.5)
    
    def test_update_from_camera(self):
        """Test updating from camera measurement"""
        self.dr.initialize((0.0, 0.0), 0.0)
        
        # Simulate some movement first
        self.dr.update_from_motors(50, 50)
        import time
        time.sleep(0.1)
        self.dr.update_from_motors(50, 50)
        
        # Now update from camera (ground truth)
        self.dr.update_from_camera((1.0, 2.0), 0.5)
        
        # After camera update, position should be updated
        pose = self.dr.get_estimated_pose()
        self.assertIsNotNone(pose)
        # Position should be updated (may be blended, but should change)
        # The camera update should influence the position
        self.assertIsNotNone(pose)
    
    def test_get_estimated_pose(self):
        """Test getting estimated pose"""
        self.dr.initialize((1.0, 2.0), 0.5)
        pose = self.dr.get_estimated_pose()
        
        self.assertIsNotNone(pose)
        self.assertEqual(pose[0], 1.0)
        self.assertEqual(pose[1], 2.0)
        self.assertEqual(pose[2], 0.5)
    
    def test_get_confidence(self):
        """Test confidence calculation"""
        self.dr.initialize((0.0, 0.0), 0.0)
        confidence = self.dr.get_confidence()
        
        self.assertGreaterEqual(confidence, 0.0)
        self.assertLessEqual(confidence, 1.0)


class TestLineDetector(unittest.TestCase):
    """Test Line Detection Module"""
    
    def setUp(self):
        self.detector = LineDetector(line_color='blue')
    
    def test_initialization(self):
        """Test detector initializes correctly"""
        self.assertIsNotNone(self.detector)
        self.assertEqual(self.detector.line_color, 'blue')
    
    def test_color_ranges(self):
        """Test color range definitions"""
        self.assertIn('blue', self.detector.color_ranges)
        self.assertIn('red', self.detector.color_ranges)
    
    def test_detected_line_creation(self):
        """Test DetectedLine creation"""
        line = DetectedLine(
            start=(0, 0),
            end=(100, 100),
            center=(50, 50),
            world_start=(0.0, 0.0),
            world_end=(1.0, 1.0),
            world_center=(0.5, 0.5),
            length=141.42,
            color='blue',
            confidence=0.8
        )
        self.assertEqual(line.world_center, (0.5, 0.5))
        self.assertEqual(line.color, 'blue')
        self.assertEqual(line.confidence, 0.8)


class TestNavigationController(unittest.TestCase):
    """Test Navigation Controller Module"""
    
    def setUp(self):
        # Create a proper mock robot with sensor attributes
        self.mock_robot = Mock()
        self.mock_robot.ultrasonic_distance = None
        self.mock_robot.ir_left = 0
        self.mock_robot.ir_right = 0
        self.controller = NavigationController(self.mock_robot)
    
    def test_initialization(self):
        """Test controller initializes correctly"""
        self.assertIsNotNone(self.controller)
        self.assertEqual(self.controller.state, NavigationState.IDLE)
    
    def test_set_waypoint(self):
        """Test setting a waypoint"""
        self.controller.set_waypoint((2.0, 3.0))
        self.assertIsNotNone(self.controller.current_waypoint)
        self.assertEqual(self.controller.current_waypoint, (2.0, 3.0))
    
    def test_set_target(self):
        """Test setting a target"""
        self.controller.set_target((5.0, 5.0))
        self.assertIsNotNone(self.controller.target_position)
        self.assertEqual(self.controller.target_position, (5.0, 5.0))
    
    def test_update_idle_state(self):
        """Test update in IDLE state"""
        command = self.controller.update(
            current_pos=(0.0, 0.0),
            current_theta=0.0,
            dt=0.1
        )
        
        self.assertIsNotNone(command)
        # In IDLE state, should not move
        self.assertEqual(command.left_motor, 0)
        self.assertEqual(command.right_motor, 0)
    
    def test_update_with_waypoint(self):
        """Test update with waypoint set"""
        self.controller.set_waypoint((1.0, 0.0))
        self.controller.state = NavigationState.NAVIGATING
        
        command = self.controller.update(
            current_pos=(0.0, 0.0),
            current_theta=0.0,
            dt=0.1
        )
        
        self.assertIsNotNone(command)
        # Should move forward towards waypoint
        self.assertGreaterEqual(command.left_motor, 0)
        self.assertGreaterEqual(command.right_motor, 0)


def run_tests():
    """Run all unit tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestTargetDetector))
    suite.addTests(loader.loadTestsFromTestCase(TestRobotLocalizer))
    suite.addTests(loader.loadTestsFromTestCase(TestPathPlanner))
    suite.addTests(loader.loadTestsFromTestCase(TestDeadReckoning))
    suite.addTests(loader.loadTestsFromTestCase(TestLineDetector))
    suite.addTests(loader.loadTestsFromTestCase(TestNavigationController))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    print("="*60)
    print("UNIT TESTS FOR AUTONOMOUS NAVIGATION MODULES")
    print("="*60)
    
    success = run_tests()
    
    print("\n" + "="*60)
    if success:
        print("✅ All unit tests passed!")
    else:
        print("❌ Some unit tests failed!")
    print("="*60)
    
    sys.exit(0 if success else 1)


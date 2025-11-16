#!/usr/bin/env python3
"""
Matplotlib Visualization Tests

Tests the visualization components that use matplotlib.
"""

import sys
import os
import numpy as np
import unittest
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for testing
import matplotlib.pyplot as plt

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from simulator_visualization import ArenaVisualizer
from simulator import SimulatedRobot, SimulatedRobotState
from path_planner import PathPlanner, Obstacle


class TestArenaVisualizer(unittest.TestCase):
    """Test Arena Visualization Module"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.visualizer = ArenaVisualizer(arena_bounds=((0, 0), (2.5, 4.0)))
    
    def tearDown(self):
        """Clean up after tests"""
        plt.close('all')
    
    def test_initialization(self):
        """Test visualizer initializes correctly"""
        self.assertIsNotNone(self.visualizer)
        self.assertIsNotNone(self.visualizer.fig)
        self.assertIsNotNone(self.visualizer.ax)
        self.assertEqual(self.visualizer.arena_bounds, ((0, 0), (2.5, 4.0)))
    
    def test_draw_arena(self):
        """Test drawing arena boundaries"""
        try:
            self.visualizer.draw_arena()
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"draw_arena() raised {e}")
    
    def test_draw_robot(self):
        """Test drawing robot"""
        robot_state = SimulatedRobotState(
            x=1.0,
            y=2.0,
            theta=0.5
        )
        try:
            self.visualizer.draw_robot(robot_state)
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"draw_robot() raised {e}")
    
    def test_draw_obstacles(self):
        """Test drawing obstacles"""
        obstacles = [
            (1.0, 1.0, 0.2),  # (x, y, radius)
            (2.0, 2.0, 0.15)
        ]
        try:
            self.visualizer.draw_obstacles(obstacles)
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"draw_obstacles() raised {e}")
    
    def test_draw_targets(self):
        """Test drawing targets"""
        targets = [
            (1.0, 2.0, 'blue'),
            (2.0, 3.0, 'green'),
            (0.5, 1.5, 'red')
        ]
        try:
            self.visualizer.draw_targets(targets)
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"draw_targets() raised {e}")
    
    def test_draw_path(self):
        """Test drawing robot path"""
        path = [
            (0.5, 0.5),
            (1.0, 1.0),
            (1.5, 1.5),
            (2.0, 2.0)
        ]
        try:
            self.visualizer.draw_path(path)
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"draw_path() raised {e}")
    
    def test_draw_goal_line(self):
        """Test drawing goal line"""
        start = (0.5, 3.5)  # Start point
        end = (2.0, 3.5)    # End point
        try:
            self.visualizer.draw_goal_line(start, end)
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"draw_goal_line() raised {e}")
    
    def test_draw_sensor_obstacles(self):
        """Test drawing sensor-detected obstacles"""
        sensor_obstacles = [
            (1.2, 1.3, 0.15),  # (x, y, radius)
            (2.1, 2.2, 0.10)
        ]
        try:
            self.visualizer.draw_sensor_obstacles(sensor_obstacles)
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"draw_sensor_obstacles() raised {e}")
    
    def test_update_status(self):
        """Test updating status text"""
        status_text = "State: NAVIGATING\nPosition: (1.0, 2.0)\nDistance: 1.5m"
        try:
            self.visualizer.update_status(status_text)
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"update_status() raised {e}")
    
    def test_clear(self):
        """Test clearing visualization"""
        # Draw some elements first
        robot_state = SimulatedRobotState(x=1.0, y=2.0, theta=0.0)
        self.visualizer.draw_robot(robot_state)
        self.visualizer.draw_path([(0.5, 0.5), (1.0, 1.0)])
        
        try:
            self.visualizer.clear()
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"clear() raised {e}")
    
    def test_full_visualization(self):
        """Test complete visualization with all elements"""
        try:
            # Draw arena
            self.visualizer.draw_arena()
            
            # Draw robot
            robot_state = SimulatedRobotState(x=1.0, y=2.0, theta=0.5)
            self.visualizer.draw_robot(robot_state)
            
            # Draw obstacles
            obstacles = [
                (1.5, 1.5, 0.2)  # (x, y, radius)
            ]
            self.visualizer.draw_obstacles(obstacles)
            
            # Draw targets
            targets = [(2.0, 3.0, 'blue')]
            self.visualizer.draw_targets(targets)
            
            # Draw path
            path = [(0.5, 0.5), (1.0, 1.0), (1.5, 1.5), (2.0, 2.0)]
            self.visualizer.draw_path(path)
            
            # Draw goal line
            self.visualizer.draw_goal_line((0.5, 3.5), (2.0, 3.5))
            
            # Update status
            self.visualizer.update_status("Test Status")
            
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"Full visualization raised {e}")
    
    def test_save_figure(self):
        """Test saving figure to file"""
        try:
            # Draw some elements
            self.visualizer.draw_arena()
            robot_state = SimulatedRobotState(x=1.0, y=2.0, theta=0.0)
            self.visualizer.draw_robot(robot_state)
            
            # Save to temporary file
            test_file = '/tmp/test_visualization.png'
            self.visualizer.fig.savefig(test_file, dpi=100, bbox_inches='tight')
            
            # Check file was created
            self.assertTrue(os.path.exists(test_file))
            
            # Clean up
            if os.path.exists(test_file):
                os.remove(test_file)
        except Exception as e:
            self.fail(f"save_figure() raised {e}")


class TestVisualizationIntegration(unittest.TestCase):
    """Test visualization with simulator integration"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.visualizer = ArenaVisualizer(arena_bounds=((0, 0), (2.5, 4.0)))
        self.robot = SimulatedRobot(initial_pos=(0.5, 0.5), initial_theta=0.0)
        self.planner = PathPlanner()
    
    def tearDown(self):
        """Clean up after tests"""
        plt.close('all')
    
    def test_visualize_robot_movement(self):
        """Test visualizing robot movement"""
        try:
            # Draw arena
            self.visualizer.draw_arena()
            
            # Move robot
            self.robot.set_motors(50, 50)
            self.robot.update(0.5)
            
            # Draw robot at new position
            state = self.robot.get_state()
            self.visualizer.draw_robot(state)
            
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"visualize_robot_movement() raised {e}")
    
    def test_visualize_path_planning(self):
        """Test visualizing path planning"""
        try:
            # Plan path
            start = (0.5, 0.5)
            goal = (2.0, 3.0)
            self.planner.replan_path(start, goal)
            
            # Draw arena
            self.visualizer.draw_arena()
            
            # Draw path
            if self.planner.current_path:
                path = [(wp.x, wp.y) for wp in self.planner.current_path]
                self.visualizer.draw_path(path)
            
            # Draw robot
            robot_state = SimulatedRobotState(x=start[0], y=start[1], theta=0.0)
            self.visualizer.draw_robot(robot_state)
            
            # Draw goal
            self.visualizer.draw_targets([(goal[0], goal[1], 'blue')])
            
            # Should not raise exception
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"visualize_path_planning() raised {e}")


def run_tests():
    """Run all visualization tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestArenaVisualizer))
    suite.addTests(loader.loadTestsFromTestCase(TestVisualizationIntegration))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    print("="*60)
    print("MATPLOTLIB VISUALIZATION TESTS")
    print("="*60)
    
    success = run_tests()
    
    print("\n" + "="*60)
    if success:
        print("✅ All visualization tests passed!")
    else:
        print("❌ Some visualization tests failed!")
    print("="*60)
    
    sys.exit(0 if success else 1)


#!/usr/bin/env python3
"""
Visual Test Scenarios for Robot Navigation

Runs multiple visual test scenarios to demonstrate robot capabilities.
"""

import asyncio
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from simulator import SimulatedRobot
from simulator_visualization import ArenaVisualizer
from mock_robot import MockGalaxyRVR
from path_planner import PathPlanner, Obstacle
from navigation_controller import NavigationController, NavigationState
from obstacle_map import ObstacleMap, get_default_obstacle_map


async def scenario_1_basic_movement():
    """Scenario 1: Basic robot movement"""
    print("\n" + "="*60)
    print("SCENARIO 1: Basic Robot Movement")
    print("="*60)
    
    robot = SimulatedRobot(initial_pos=(0.5, 0.5), initial_theta=0.0)
    visualizer = ArenaVisualizer(arena_bounds=((0, 0), (2.5, 4.0)))
    visualizer.draw_arena()
    
    path_history = []
    print("Moving robot forward and rotating...")
    
    # Move forward
    for i in range(20):
        robot.set_motors(50, 50)
        robot.update(0.1)
        state = robot.get_state()
        path_history.append((state.x, state.y))
        
        visualizer.clear()
        visualizer.draw_arena()
        visualizer.draw_path(path_history)
        visualizer.draw_robot(state)
        visualizer.update_status(f"Scenario 1: Moving Forward\nStep: {i+1}/20\nPosition: ({state.x:.2f}, {state.y:.2f})")
        plt.draw()
        plt.pause(0.05)
    
    # Rotate
    for i in range(10):
        robot.set_motors(-50, 50)
        robot.update(0.1)
        state = robot.get_state()
        
        visualizer.clear()
        visualizer.draw_arena()
        visualizer.draw_path(path_history)
        visualizer.draw_robot(state)
        visualizer.update_status(f"Scenario 1: Rotating\nStep: {i+1}/10\nAngle: {state.theta:.2f} rad")
        plt.draw()
        plt.pause(0.05)
    
    print("✅ Scenario 1 Complete!")
    plt.pause(2.0)
    plt.close()


async def scenario_2_path_planning():
    """Scenario 2: Path planning visualization"""
    print("\n" + "="*60)
    print("SCENARIO 2: Path Planning")
    print("="*60)
    
    robot = SimulatedRobot(initial_pos=(0.5, 0.5), initial_theta=0.0)
    planner = PathPlanner()
    visualizer = ArenaVisualizer(arena_bounds=((0, 0), (2.5, 4.0)))
    visualizer.draw_arena()
    
    start = (0.5, 0.5)
    goal = (2.0, 3.0)
    
    # Plan path
    planner.replan_path(start, goal)
    
    # Draw everything
    visualizer.draw_targets([(goal[0], goal[1], 'blue')])
    if planner.current_path:
        path = [(wp.x, wp.y) for wp in planner.current_path]
        visualizer.draw_path(path)
    
    state = robot.get_state()
    visualizer.draw_robot(state)
    visualizer.update_status("Scenario 2: Path Planning\nPath from start to goal planned")
    plt.draw()
    plt.pause(3.0)
    
    print("✅ Scenario 2 Complete!")
    plt.close()


async def scenario_3_obstacle_avoidance():
    """Scenario 3: Obstacle avoidance"""
    print("\n" + "="*60)
    print("SCENARIO 3: Obstacle Avoidance")
    print("="*60)
    
    robot = SimulatedRobot(initial_pos=(0.5, 0.5), initial_theta=0.0)
    planner = PathPlanner()
    visualizer = ArenaVisualizer(arena_bounds=((0, 0), (2.5, 4.0)))
    visualizer.draw_arena()
    
    # Use pre-mapped obstacles (known positions, variable properties)
    obstacle_map = get_default_obstacle_map()
    pre_mapped_obstacles = obstacle_map.get_obstacles()
    
    obstacles = [(obs.x, obs.y, obs.radius) for obs in pre_mapped_obstacles]
    obstacle_colors = [obs.color or 'red' for obs in pre_mapped_obstacles]
    
    for obs in pre_mapped_obstacles:
        robot.add_obstacle(obs.x, obs.y, obs.radius)
        planner.add_obstacle(obs)
    
    visualizer.draw_obstacles(obstacles, obstacle_colors=obstacle_colors)
    
    start = (0.5, 0.5)
    goal = (2.0, 3.0)
    
    # Plan path (should avoid obstacles)
    planner.replan_path(start, goal)
    
    visualizer.draw_targets([(goal[0], goal[1], 'blue')])
    if planner.current_path:
        path = [(wp.x, wp.y) for wp in planner.current_path]
        visualizer.draw_path(path)
    
    state = robot.get_state()
    visualizer.draw_robot(state)
    visualizer.update_status("Scenario 3: Obstacle Avoidance\nPath planned avoiding obstacles")
    plt.draw()
    plt.pause(3.0)
    
    print("✅ Scenario 3 Complete!")
    plt.close()


async def scenario_4_full_navigation():
    """Scenario 4: Full navigation to goal"""
    print("\n" + "="*60)
    print("SCENARIO 4: Full Navigation to Goal")
    print("="*60)
    print("Watch the robot navigate to the goal at the top...")
    
    robot = SimulatedRobot(initial_pos=(0.5, 0.5), initial_theta=0.0)
    mock_robot = MockGalaxyRVR(robot)
    planner = PathPlanner()
    controller = NavigationController(mock_robot)
    visualizer = ArenaVisualizer(arena_bounds=((0, 0), (2.5, 4.0)))
    visualizer.draw_arena()
    
    # Use pre-mapped obstacles (known positions, variable properties)
    obstacle_map = get_default_obstacle_map()
    pre_mapped_obstacles = obstacle_map.get_obstacles()
    
    obstacles = [(obs.x, obs.y, obs.radius) for obs in pre_mapped_obstacles]
    obstacle_colors = [obs.color or 'red' for obs in pre_mapped_obstacles]
    
    for obs in pre_mapped_obstacles:
        robot.add_obstacle(obs.x, obs.y, obs.radius)
        planner.add_obstacle(obs)
    
    visualizer.draw_obstacles(obstacles, obstacle_colors=obstacle_colors)
    
    # Set goal
    goal = (1.25, 3.5)  # Top center
    controller.set_target(goal)
    visualizer.draw_targets([(goal[0], goal[1], 'blue')])
    visualizer.draw_goal_line((0.5, 3.5), (2.0, 3.5))
    
    path_history = []
    max_iterations = 150
    
    print("Starting navigation...")
    for i in range(max_iterations):
        state = robot.get_state()
        current_pos = (state.x, state.y)
        path_history.append(current_pos)
        
        # Check if reached goal
        distance_to_goal = np.sqrt(
            (current_pos[0] - goal[0])**2 + 
            (current_pos[1] - goal[1])**2
        )
        
        if distance_to_goal < 0.2:
            print(f"✅ Goal reached at iteration {i}!")
            break
        
        # Plan and navigate
        planner.replan_path(current_pos, goal)
        waypoint = planner.get_next_waypoint()
        
        if waypoint:
            controller.set_waypoint((waypoint.x, waypoint.y))
            controller.state = NavigationState.NAVIGATING
        
        command = controller.update(current_pos, state.theta, 0.1)
        robot.set_motors(command.left_motor, command.right_motor)
        robot.update(0.1)
        
        # Update visualization
        visualizer.clear()
        visualizer.draw_arena()
        visualizer.draw_obstacles(obstacles)
        visualizer.draw_targets([(goal[0], goal[1], 'blue')])
        visualizer.draw_goal_line((0.5, 3.5), (2.0, 3.5))
        visualizer.draw_path(path_history)
        
        # Draw waypoints
        if planner.current_path:
            waypoint_path = [(wp.x, wp.y) for wp in planner.current_path if not wp.reached]
            if waypoint_path:
                wp_x = [p[0] for p in waypoint_path]
                wp_y = [p[1] for p in waypoint_path]
                visualizer.ax.scatter(wp_x, wp_y, c='orange', s=100, marker='s', 
                          alpha=0.6, label='Waypoints', zorder=2)
        
        state = robot.get_state()
        visualizer.draw_robot(state)
        
        status = f"Scenario 4: Navigation to Goal\n"
        status += f"Iteration: {i}/{max_iterations}\n"
        status += f"Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})\n"
        status += f"Distance: {distance_to_goal:.2f}m\n"
        status += f"State: {controller.state.value}"
        visualizer.update_status(status)
        
        plt.draw()
        plt.pause(0.03)
        
        if i % 20 == 0:
            print(f"  Iteration {i}: pos=({current_pos[0]:.2f}, {current_pos[1]:.2f}), dist={distance_to_goal:.2f}m")
    
    final_state = robot.get_state()
    final_pos = (final_state.x, final_state.y)
    final_distance = np.sqrt((final_pos[0] - goal[0])**2 + (final_pos[1] - goal[1])**2)
    
    print(f"\nFinal position: ({final_pos[0]:.2f}, {final_pos[1]:.2f})")
    print(f"Distance to goal: {final_distance:.2f}m")
    print("✅ Scenario 4 Complete!")
    
    visualizer.update_status(f"Scenario 4: Complete!\nFinal distance: {final_distance:.2f}m")
    plt.draw()
    plt.pause(3.0)
    plt.close()


async def main():
    """Run all visual scenarios"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Visual Robot Test Scenarios')
    parser.add_argument('--scenario', type=int, choices=[1, 2, 3, 4],
                       help='Run specific scenario (1-4)')
    parser.add_argument('--all', action='store_true',
                       help='Run all scenarios')
    
    args = parser.parse_args()
    
    scenarios = {
        1: scenario_1_basic_movement,
        2: scenario_2_path_planning,
        3: scenario_3_obstacle_avoidance,
        4: scenario_4_full_navigation
    }
    
    print("\n" + "="*60)
    print("VISUAL ROBOT TEST SCENARIOS")
    print("="*60)
    print("Each scenario will show a matplotlib window with visualization.")
    print("Watch the robot navigate, plan paths, and avoid obstacles!")
    print("="*60)
    
    try:
        if args.scenario:
            await scenarios[args.scenario]()
        else:
            # Run all scenarios
            for i, scenario_func in scenarios.items():
                await scenario_func()
                print(f"\nCompleted scenario {i}/4")
            
            print("\n" + "="*60)
            print("✅ ALL VISUAL SCENARIOS COMPLETE!")
            print("="*60)
            
    except KeyboardInterrupt:
        print("\n\n⚠️  Tests interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        plt.close('all')


if __name__ == "__main__":
    asyncio.run(main())


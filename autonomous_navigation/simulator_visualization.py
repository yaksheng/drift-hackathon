"""
Simulator Visualization

Visualizes the simulated arena, robot, targets, and navigation paths.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from typing import List, Tuple, Optional
from simulator import SimulatedRobot, SimulatedRobotState


class ArenaVisualizer:
    """Visualizes the simulated arena and robot"""
    
    def __init__(self, 
                 arena_bounds: Tuple[Tuple[float, float], Tuple[float, float]],
                 figsize: Tuple[int, int] = (12, 8)):
        """
        Initialize visualizer
        
        Args:
            arena_bounds: ((min_x, min_y), (max_x, max_y))
            figsize: Figure size (width, height)
        """
        self.arena_bounds = arena_bounds
        self.fig, self.ax = plt.subplots(figsize=figsize)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(arena_bounds[0][0] - 0.5, arena_bounds[1][0] + 0.5)
        self.ax.set_ylim(arena_bounds[0][1] - 0.5, arena_bounds[1][1] + 0.5)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Robot Navigation Simulation')
        self.ax.grid(True, alpha=0.3)
        
        # Store plot elements
        self.robot_circle = None
        self.robot_arrow = None
        self.targets_scatter = None
        self.path_line = None
        self.obstacle_circles = []
        self.line_segments = []
        self.status_text = None
        
    def draw_arena(self):
        """Draw arena boundaries"""
        min_x, min_y = self.arena_bounds[0]
        max_x, max_y = self.arena_bounds[1]
        
        # Draw arena rectangle
        rect = patches.Rectangle(
            (min_x, min_y),
            max_x - min_x,
            max_y - min_y,
            linewidth=2,
            edgecolor='black',
            facecolor='lightgray',
            alpha=0.3
        )
        self.ax.add_patch(rect)
    
    def draw_obstacles(self, obstacles: List[Tuple[float, float, float]]):
        """Draw obstacles"""
        # Clear existing obstacles
        for circle in self.obstacle_circles:
            circle.remove()
        self.obstacle_circles = []
        
        for x, y, radius in obstacles:
            circle = patches.Circle(
                (x, y),
                radius,
                color='red',
                alpha=0.5,
                edgecolor='darkred',
                linewidth=2
            )
            self.ax.add_patch(circle)
            self.obstacle_circles.append(circle)
    
    def draw_lines(self, lines: List[Tuple[float, float, float, float]], 
                   stop_line_index: Optional[int] = None):
        """
        Draw lines in the arena
        
        Args:
            lines: List of (x1, y1, x2, y2) line coordinates
            stop_line_index: Index of line to stop at (highlighted)
        """
        # Clear existing lines
        for line in self.line_segments:
            line.remove()
        self.line_segments = []
        
        for i, (x1, y1, x2, y2) in enumerate(lines):
            color = 'green' if i == stop_line_index else 'blue'
            linewidth = 3 if i == stop_line_index else 2
            linestyle = '--' if i == stop_line_index else '-'
            
            line = self.ax.plot(
                [x1, x2],
                [y1, y2],
                color=color,
                linewidth=linewidth,
                linestyle=linestyle,
                label=f'Line {i+1}' if i == stop_line_index else None
            )[0]
            self.line_segments.append(line)
        
        if stop_line_index is not None:
            self.ax.legend()
    
    def draw_robot(self, state: SimulatedRobotState):
        """Draw robot at current position"""
        # Remove old robot elements
        if self.robot_circle:
            self.robot_circle.remove()
        if self.robot_arrow:
            self.robot_arrow.remove()
        
        # Draw robot body (circle)
        self.robot_circle = patches.Circle(
            (state.x, state.y),
            0.15,  # robot radius
            color='blue',
            alpha=0.7,
            edgecolor='darkblue',
            linewidth=2
        )
        self.ax.add_patch(self.robot_circle)
        
        # Draw orientation arrow
        arrow_length = 0.2
        end_x = state.x + arrow_length * np.cos(state.theta)
        end_y = state.y + arrow_length * np.sin(state.theta)
        
        self.robot_arrow = self.ax.annotate(
            '',
            xy=(end_x, end_y),
            xytext=(state.x, state.y),
            arrowprops=dict(arrowstyle='->', color='darkblue', lw=2)
        )
    
    def draw_targets(self, targets: List[Tuple[float, float, str]]):
        """
        Draw targets
        
        Args:
            targets: List of (x, y, color) target positions
        """
        if targets:
            x_coords = [t[0] for t in targets]
            y_coords = [t[1] for t in targets]
            colors = [t[2] for t in targets]
            
            if self.targets_scatter:
                self.targets_scatter.remove()
            
            self.targets_scatter = self.ax.scatter(
                x_coords, y_coords,
                c=colors,
                s=200,
                marker='o',
                edgecolors='black',
                linewidths=2,
                alpha=0.7,
                label='Targets'
            )
            self.ax.legend()
    
    def draw_path(self, path: List[Tuple[float, float]]):
        """
        Draw planned path
        
        Args:
            path: List of (x, y) waypoints
        """
        if path:
            if self.path_line:
                self.path_line.remove()
            
            x_coords = [p[0] for p in path]
            y_coords = [p[1] for p in path]
            
            self.path_line = self.ax.plot(
                x_coords, y_coords,
                'g--',
                linewidth=2,
                alpha=0.5,
                label='Planned Path'
            )[0]
            self.ax.legend()
    
    def update_status(self, text: str):
        """Update status text"""
        if self.status_text:
            self.status_text.remove()
        
        self.status_text = self.ax.text(
            0.02, 0.98,
            text,
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        )
    
    def clear(self):
        """Clear all dynamic elements"""
        if self.robot_circle:
            self.robot_circle.remove()
            self.robot_circle = None
        if self.robot_arrow:
            self.robot_arrow.remove()
            self.robot_arrow = None
        if self.targets_scatter:
            self.targets_scatter.remove()
            self.targets_scatter = None
        if self.path_line:
            self.path_line.remove()
            self.path_line = None
        if self.status_text:
            self.status_text.remove()
            self.status_text = None
    
    def show(self, block: bool = True):
        """Show the plot"""
        plt.show(block=block)
    
    def save(self, filename: str):
        """Save the plot to file"""
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')


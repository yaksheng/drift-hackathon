"""
Obstacle Map Configuration

Pre-mapped obstacles for the arena. In real-world conditions, obstacles
are known positions but may have variable sizes, positions, and colors.
"""

from typing import List, Tuple, Optional
import json
import os
import sys

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from autonomous_navigation.path_planner import Obstacle


class ObstacleMap:
    """Manages pre-mapped obstacles for the arena"""
    
    def __init__(self, map_file: Optional[str] = None):
        """
        Initialize obstacle map
        
        Args:
            map_file: Path to JSON file with obstacle configuration
        """
        self.obstacles: List[Obstacle] = []
        self.map_file = map_file
        
        if map_file and os.path.exists(map_file):
            self.load_from_file(map_file)
        else:
            # Default pre-mapped obstacles (typical arena setup)
            self.load_default_map()
    
    def load_default_map(self):
        """Load default pre-mapped obstacles"""
        # Default obstacles - known positions but variable properties
        default_obstacles = [
            {
                'x': 1.0,
                'y': 1.5,
                'radius': 0.2,  # Variable: 0.15-0.25m
                'color': 'red',  # Variable: red, blue, green, etc.
                'confidence': 1.0
            },
            {
                'x': 1.5,
                'y': 2.0,
                'radius': 0.15,  # Variable: 0.10-0.20m
                'color': 'blue',
                'confidence': 1.0
            },
            {
                'x': 2.0,
                'y': 1.0,
                'radius': 0.18,  # Variable: 0.15-0.22m
                'color': 'green',
                'confidence': 1.0
            }
        ]
        
        for obs_data in default_obstacles:
            obstacle = Obstacle(
                x=obs_data['x'],
                y=obs_data['y'],
                radius=obs_data['radius'],
                confidence=obs_data['confidence'],
                color=obs_data.get('color')
            )
            self.obstacles.append(obstacle)
    
    def load_from_file(self, filepath: str):
        """
        Load obstacles from JSON file
        
        File format:
        {
            "obstacles": [
                {
                    "x": 1.0,
                    "y": 1.5,
                    "radius": 0.2,
                    "color": "red",
                    "confidence": 1.0
                },
                ...
            ]
        }
        """
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            self.obstacles = []
            for obs_data in data.get('obstacles', []):
                obstacle = Obstacle(
                    x=obs_data['x'],
                    y=obs_data['y'],
                    radius=obs_data['radius'],
                    confidence=obs_data.get('confidence', 1.0),
                    color=obs_data.get('color')
                )
                self.obstacles.append(obstacle)
        except Exception as e:
            print(f"Warning: Could not load obstacle map from {filepath}: {e}")
            self.load_default_map()
    
    def save_to_file(self, filepath: str):
        """Save obstacles to JSON file"""
        data = {
            'obstacles': [
                {
                    'x': obs.x,
                    'y': obs.y,
                    'radius': obs.radius,
                    'confidence': obs.confidence,
                    'color': obs.color
                }
                for obs in self.obstacles
            ]
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    def get_obstacles(self) -> List[Obstacle]:
        """Get all pre-mapped obstacles"""
        return self.obstacles.copy()
    
    def add_obstacle(self, x: float, y: float, radius: float, 
                    color: Optional[str] = None, confidence: float = 1.0):
        """Add a pre-mapped obstacle"""
        obstacle = Obstacle(
            x=x,
            y=y,
            radius=radius,
            confidence=confidence,
            color=color
        )
        self.obstacles.append(obstacle)
    
    def apply_variations(self, position_variance: float = 0.1,
                        radius_variance: float = 0.05,
                        color_options: Optional[List[str]] = None):
        """
        Apply variations to obstacle properties (simulating real-world differences)
        
        Args:
            position_variance: Maximum position variation in meters
            radius_variance: Maximum radius variation in meters
            color_options: List of possible colors
        """
        import random
        
        if color_options is None:
            color_options = ['red', 'blue', 'green', 'yellow', 'orange']
        
        for obstacle in self.obstacles:
            # Vary position slightly
            obstacle.x += random.uniform(-position_variance, position_variance)
            obstacle.y += random.uniform(-position_variance, position_variance)
            
            # Vary radius slightly
            obstacle.radius += random.uniform(-radius_variance, radius_variance)
            obstacle.radius = max(0.1, obstacle.radius)  # Minimum radius
            
            # Randomize color if not set
            if obstacle.color is None:
                obstacle.color = random.choice(color_options)


def get_default_obstacle_map() -> ObstacleMap:
    """Get default obstacle map with pre-mapped obstacles"""
    return ObstacleMap()


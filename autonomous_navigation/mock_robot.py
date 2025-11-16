"""
Mock Robot Interface

Provides a GalaxyRVR-compatible interface using the simulator.
Allows existing navigation code to work with simulated robot.
"""

import asyncio
from typing import Optional
from simulator import SimulatedRobot, SimulatedRobotState


class MockGalaxyRVR:
    """Mock GalaxyRVR that mimics the real robot interface"""
    
    def __init__(self, simulated_robot: SimulatedRobot):
        """
        Initialize mock robot
        
        Args:
            simulated_robot: SimulatedRobot instance
        """
        self.simulated_robot = simulated_robot
        self.connected = True
        self.running = True
        
        # Interface compatibility
        self.left_motor = 0
        self.right_motor = 0
        self.servo_angle = 90
        
        # Sensor data (read-only, updated from simulator)
        self.battery_voltage = None
        self.ultrasonic_distance = None
        self.ir_left = None
        self.ir_right = None
        
        # Update sensor readings
        self._update_sensors()
    
    async def connect(self):
        """Connect to robot (always succeeds in simulation)"""
        self.connected = True
        print("✓ Connected to simulated robot")
        return True
    
    async def disconnect(self):
        """Disconnect from robot"""
        self.connected = False
        self.running = False
        print("✓ Disconnected from simulated robot")
    
    async def send(self):
        """Send motor commands to robot"""
        if not self.connected:
            return False
        
        # Apply commands to simulated robot
        self.simulated_robot.set_motors(self.left_motor, self.right_motor)
        self.simulated_robot.set_servo(self.servo_angle)
        
        # Update simulator
        self.simulated_robot.update()
        
        # Update sensor readings
        self._update_sensors()
        
        return True
    
    def set_motors(self, left: int, right: int):
        """Set motor speeds"""
        self.left_motor = max(-100, min(100, int(left)))
        self.right_motor = max(-100, min(100, int(right)))
    
    def set_servo(self, angle: int):
        """Set servo angle"""
        self.servo_angle = max(0, min(180, int(angle)))
    
    def stop(self):
        """Stop all motors"""
        self.left_motor = 0
        self.right_motor = 0
    
    def forward(self, speed: int = 60):
        """Move forward"""
        speed = max(0, min(100, speed))
        self.left_motor = speed
        self.right_motor = speed
    
    def backward(self, speed: int = 60):
        """Move backward"""
        speed = max(0, min(100, speed))
        self.left_motor = -speed
        self.right_motor = -speed
    
    def turn_left(self, speed: int = 60):
        """Rotate left"""
        speed = max(0, min(100, speed))
        self.left_motor = -speed
        self.right_motor = speed
    
    def turn_right(self, speed: int = 60):
        """Rotate right"""
        speed = max(0, min(100, speed))
        self.left_motor = speed
        self.right_motor = -speed
    
    def _update_sensors(self):
        """Update sensor readings from simulator"""
        state = self.simulated_robot.get_state()
        self.battery_voltage = state.battery_voltage
        self.ultrasonic_distance = state.ultrasonic_distance
        self.ir_left = state.ir_left
        self.ir_right = state.ir_right
    
    def get_position(self):
        """Get current robot position"""
        return self.simulated_robot.get_position()
    
    def get_orientation(self):
        """Get current robot orientation"""
        return self.simulated_robot.get_orientation()


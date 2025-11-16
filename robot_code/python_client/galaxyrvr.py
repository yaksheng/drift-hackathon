"""
GalaxyRVR Base Library

This module provides a simple API to control the GalaxyRVR robot.
Import this library into your own scripts to easily control the robot.

Example usage:
    from galaxyrvr import GalaxyRVR

    async def main():
        robot = GalaxyRVR("192.168.1.216")
        await robot.connect()

        # Move forward
        robot.forward(speed=60)
        await robot.send()

        # Stop
        robot.stop()
        await robot.send()

        await robot.disconnect()
"""

import asyncio
import websockets
import json


class GalaxyRVR:
    """Base class for controlling GalaxyRVR robot"""

    def __init__(self, robot_ip="192.168.1.216", port=8765):
        """
        Initialize the GalaxyRVR controller

        Args:
            robot_ip: IP address of the ESP32-CAM
            port: WebSocket port (default: 8765)
        """
        self.robot_ip = robot_ip
        self.ws_url = f"ws://{robot_ip}:{port}"
        self.camera_url = f"http://{robot_ip}:9000/mjpg"
        self.websocket = None

        # Control variables
        self.left_motor = 0
        self.right_motor = 0
        self.servo_angle = 90

        # Track last sent values to avoid redundant sends
        self._last_left_motor = None
        self._last_right_motor = None
        self._last_servo_angle = None

        # Sensor data (read-only, updated from robot)
        self.battery_voltage = None     # BV: Battery voltage
        self.ultrasonic_distance = None # O: Ultrasonic distance in cm
        self.ir_left = None             # N: Left IR sensor (1=obstacle, 0=clear)
        self.ir_right = None            # P: Right IR sensor (1=obstacle, 0=clear)

        # Status
        self.connected = False
        self.running = True
        self._receive_task = None
        self._is_stop_signal = False
        
        # Obstacle avoidance mode flags
        self._obstacle_avoidance_enabled = False
        self._obstacle_following_enabled = False
        self._last_obstacle_avoidance_enabled = False
        self._last_obstacle_following_enabled = False

    async def connect(self):
        """Establish WebSocket connection to robot"""
        try:
            self.websocket = await websockets.connect(self.ws_url)
            self.connected = True
            print(f"✓ Connected to robot at {self.ws_url}")

            # Start receive task to consume pong messages
            self._receive_task = asyncio.create_task(self._receive_messages())
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            self.connected = False
            return False

    async def disconnect(self):
        """Close WebSocket connection"""
        self.running = False

        # Cancel receive task
        if self._receive_task:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass

        # Stop motors before disconnecting
        self.stop()
        await self.send()

        if self.websocket:
            await self.websocket.close()
            self.connected = False
            print("✓ Disconnected from robot")

    async def _receive_messages(self):
        """Background task to receive messages from robot"""
        while self.connected and self.running:
            try:
                message = await self.websocket.recv()

                print(f"← Received: {message}")

                # Parse sensor data from JSON messages
                if message != "pong" and not message.startswith("pong "):
                    try:
                        data = json.loads(message)

                        # Update sensor values if present
                        if "BV" in data:
                            self.battery_voltage = data["BV"]
                        if "O" in data:
                            self.ultrasonic_distance = data["O"]
                        if "N" in data:
                            self.ir_left = data["N"]
                        if "P" in data:
                            self.ir_right = data["P"]

                        # print(self.ultrasonic_distance)

                    except (json.JSONDecodeError, TypeError):
                        # Not JSON or null, ignore
                        pass

            except Exception as e:
                if self.running:
                    print(f"Receive error: {e}")
                break

    async def reconnect(self):
        """Reconnect to the robot after connection loss"""
        print("⟳ Reconnecting...")

        # Cancel receive task
        if self._receive_task:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass

        if self.websocket:
            try:
                await self.websocket.close()
            except:
                pass

        self.connected = False
        await asyncio.sleep(0.5)
        await self.connect()

        if self.connected:
            # Resend current state
            self._last_left_motor = None
            self._last_right_motor = None
            self._last_servo_angle = None

    async def send(self):
        """Send current motor and servo values to robot"""
        if not self.websocket or not self.connected:
            return False

        try:
            # Check if values changed (including obstacle avoidance mode changes)
            values_changed = (
                self.left_motor != self._last_left_motor or
                self.right_motor != self._last_right_motor or
                self.servo_angle != self._last_servo_angle or
                self._obstacle_avoidance_enabled != self._last_obstacle_avoidance_enabled or
                self._obstacle_following_enabled != self._last_obstacle_following_enabled
            )

            if not self._is_stop_signal:
                # Only send if something changed
                if not values_changed:
                    return True

            # Create JSON command with all regions
            command = {}
            regions = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M',
                      'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']

            for region in regions:
                command[region] = None

            # Set motor and servo values
            command["D"] = self.servo_angle   # Servo
            
            # Control obstacle avoidance modes via REGION_E and REGION_F
            # REGION_E: Obstacle avoidance mode (MODE_OBSTACLE_AVOIDANCE)
            # REGION_F: Obstacle following mode (MODE_OBSTACLE_FOLLOWING)
            # Arduino checks getSwitch(REGION_E) which reads boolean from JSON
            if self._obstacle_avoidance_enabled:
                command["E"] = 1  # Enable obstacle avoidance mode (1 = True/ON)
                command["F"] = 0  # Disable obstacle following (0 = False/OFF)
                # Don't send motor commands when in obstacle avoidance mode
                # Arduino will control motors automatically via obstacleAvoidance()
                command["K"] = None
                command["Q"] = None
            elif self._obstacle_following_enabled:
                command["E"] = 0  # Disable obstacle avoidance
                command["F"] = 1  # Enable obstacle following mode (1 = True/ON)
                # Don't send motor commands when in obstacle following mode
                command["K"] = None
                command["Q"] = None
            else:
                # Normal manual control mode
                command["E"] = 0  # Disable obstacle avoidance (0 = False/OFF)
                command["F"] = 0  # Disable obstacle following (0 = False/OFF)
                command["K"] = self.left_motor    # Left motor
                command["Q"] = self.right_motor   # Right motor

            # Send as JSON
            message = json.dumps(command)
            await self.websocket.send(message)

            # Update tracking variables
            self._last_left_motor = self.left_motor
            self._last_right_motor = self.right_motor
            self._last_servo_angle = self.servo_angle
            self._last_obstacle_avoidance_enabled = self._obstacle_avoidance_enabled
            self._last_obstacle_following_enabled = self._obstacle_following_enabled
            self._is_stop_signal = False

            return True

        except Exception as e:
            print(f"Send error: {e}")
            self.connected = False
            await self.reconnect()
            self._is_stop_signal = False
            return False

    def set_motors(self, left, right):
        """
        Set motor powers directly

        Args:
            left: Left motor power (-100 to 100)
            right: Right motor power (-100 to 100)
        """
        self.left_motor = max(-100, min(100, int(left)))
        self.right_motor = max(-100, min(100, int(right)))

    def set_servo(self, angle):
        """
        Set servo angle

        Args:
            angle: Servo angle (0-180, typically 90 for center)
        """
        self.servo_angle = max(0, min(180, int(angle)))

    def stop(self):
        """Stop all motors"""
        self._is_stop_signal = True
        self.left_motor = 0
        self.right_motor = 0
    
    def enable_obstacle_avoidance(self):
        """
        Enable Arduino's built-in obstacle avoidance mode
        
        This activates MODE_OBSTACLE_AVOIDANCE on the Arduino,
        which uses the obstacleAvoidance() function.
        """
        self._obstacle_avoidance_enabled = True
    
    def disable_obstacle_avoidance(self):
        """
        Disable Arduino's built-in obstacle avoidance mode
        
        Returns control to MODE_APP_CONTROL for manual motor control.
        """
        self._obstacle_avoidance_enabled = False
    
    def enable_obstacle_following(self):
        """
        Enable Arduino's built-in obstacle following mode
        
        This activates MODE_OBSTACLE_FOLLOWING on the Arduino,
        which uses the obstacleFollowing() function.
        """
        self._obstacle_following_enabled = True
    
    def disable_obstacle_following(self):
        """
        Disable Arduino's built-in obstacle following mode
        """
        self._obstacle_following_enabled = False

    def forward(self, speed=60):
        """
        Move forward

        Args:
            speed: Speed (0-100)
        """
        speed = max(0, min(100, speed))
        self.left_motor = speed
        self.right_motor = speed

    def backward(self, speed=60):
        """
        Move backward

        Args:
            speed: Speed (0-100)
        """
        speed = max(0, min(100, speed))
        self.left_motor = -speed
        self.right_motor = -speed

    def turn_left(self, speed=60):
        """
        Rotate left in place

        Args:
            speed: Speed (0-100)
        """
        speed = max(0, min(100, speed))
        self.left_motor = -speed
        self.right_motor = speed

    def turn_right(self, speed=60):
        """
        Rotate right in place

        Args:
            speed: Speed (0-100)
        """
        speed = max(0, min(100, speed))
        self.left_motor = speed
        self.right_motor = -speed

    def forward_left(self, speed=60):
        """
        Move forward and left diagonally

        Args:
            speed: Speed (0-100)
        """
        speed = max(0, min(100, speed))
        self.left_motor = int(speed * 0.3)
        self.right_motor = speed

    def forward_right(self, speed=60):
        """
        Move forward and right diagonally

        Args:
            speed: Speed (0-100)
        """
        speed = max(0, min(100, speed))
        self.left_motor = speed
        self.right_motor = int(speed * 0.3)

    def backward_left(self, speed=60):
        """
        Move backward and left diagonally

        Args:
            speed: Speed (0-100)
        """
        speed = max(0, min(100, speed))
        self.left_motor = int(-speed * 0.3)
        self.right_motor = -speed

    def backward_right(self, speed=60):
        """
        Move backward and right diagonally

        Args:
            speed: Speed (0-100)
        """
        speed = max(0, min(100, speed))
        self.left_motor = -speed
        self.right_motor = int(-speed * 0.3)


# Convenience function for quick setup
async def create_robot(robot_ip="192.168.1.216", port=8765):
    """
    Create and connect to a GalaxyRVR robot

    Args:
        robot_ip: IP address of the ESP32-CAM
        port: WebSocket port (default: 8765)

    Returns:
        Connected GalaxyRVR instance or None if connection failed
    """
    robot = GalaxyRVR(robot_ip, port)
    if await robot.connect():
        return robot
    return None

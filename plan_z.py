"""
GalaxyRVR Move Forward for 9 Seconds

This script uses the GalaxyRVR library to make the robot
move forward for a fixed duration.
"""

import asyncio
import sys
import os

# Add robot_code/python_client to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'robot_code', 'python_client'))
from galaxyrvr import GalaxyRVR
# We don't need the KeyboardController for this script
# from galaxyrvr_keyboard import KeyboardController


async def main():
    """Main entry point"""

    # Configuration
    # robot_ip = "192.168.1.216"
    robot_ip = "192.168.1.113" ## 2nd robot
    # robot_ip = "192.168.4.1"
    port = 8765

    print("=" * 60)
    print("GalaxyRVR - Move Forward for 9 Seconds")
    print("=" * 60)

    # Create and connect to robot
    robot = GalaxyRVR(robot_ip, port)
    if not await robot.connect():
        print("Failed to connect. Exiting...")
        return

    # --- Autonomous Command Section ---
    try:
        print("Command: Moving forward at speed 60...")
        
        # Tell the robot to move forward at a speed of 60.
        # This speed is based on the "normal_speed=60" from your original script.
        robot.forward(speed=60)
        await robot.send()

        print("Waiting for 9 seconds...")
        await asyncio.sleep(9)

        print("Command: Stopping robot...")
        # Tell the robot to stop
        robot.stop()
        await robot.send()
        
        # Give a brief moment for the stop command to register before disconnecting
        await asyncio.sleep(0.5)

    except Exception as e:
        print(f"An error occurred during movement: {e}")
    # --- End of Command Section ---

    # Cleanup
    print("Disconnecting...")
    await robot.disconnect()
    print("Done!")


if __name__ == "__main__":
    asyncio.run(main())
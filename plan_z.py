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
    robot_ip = "192.168.0.113" ## 2nd robot
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
        # First, ensure we're in a clean state by sending stop
        print("Resetting robot state...")
        robot.stop()
        await robot.send()
        await asyncio.sleep(0.2)
        
        # Set servo to ensure mode is set (this helps trigger MODE_APP_CONTROL)
        robot.set_servo(90)
        
        print("Command: Moving forward at speed 80...")
        # Use set_motors directly with a higher speed to ensure movement
        # Speed 80 should be well above the minimum threshold
        robot.set_motors(80, 80)
        print(f"Motor values: left={robot.left_motor}, right={robot.right_motor}")
        await robot.send()
        print("Command sent!")

        print("Waiting for 9 seconds...")
        # Send commands continuously to keep the robot moving
        # The robot may need periodic updates to maintain movement
        start_time = asyncio.get_event_loop().time()
        while (asyncio.get_event_loop().time() - start_time) < 9.0:
            # Force send by resetting tracking variables
            robot._last_left_motor = None
            robot._last_right_motor = None
            # Ensure motors are still set correctly
            robot.set_motors(80, 80)
            await robot.send()  # Resend command periodically
            await asyncio.sleep(0.1)  # Send every 100ms

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
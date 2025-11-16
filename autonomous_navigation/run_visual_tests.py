#!/usr/bin/env python3
"""
Run Visual Robot Tests

Simple wrapper to run visual tests using the existing simulation.
"""

import sys
import os
import subprocess

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

def main():
    print("="*60)
    print("VISUAL ROBOT TESTS")
    print("="*60)
    print("\nThis will run the simulation with visualization enabled.")
    print("You will see:")
    print("  - Robot position and orientation")
    print("  - Planned path (green line)")
    print("  - Waypoints (orange squares)")
    print("  - Obstacles (red circles)")
    print("  - Goal line at top (blue line)")
    print("  - Real-time status updates")
    print("\n" + "="*60)
    
    # Run the simulation with visualization
    print("\nStarting visual simulation...")
    print("The matplotlib window will open showing the robot navigation.")
    print("Press Ctrl+C to stop.\n")
    
    try:
        # Use the existing simulate.py script
        script_path = os.path.join(os.path.dirname(__file__), '..', 'simulate.py')
        result = subprocess.run(
            ['python3', script_path, '--stop-at-line', '1', '--max-iterations', '200'],
            cwd=os.path.dirname(os.path.dirname(__file__))
        )
        return result.returncode
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        return 0
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())


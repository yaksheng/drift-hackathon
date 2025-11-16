#!/usr/bin/env python3
"""
Simulation Wrapper Script

Convenience script to run simulation from project root.
"""

import sys
import os

# Add autonomous_navigation to path
autonomous_nav_path = os.path.join(os.path.dirname(__file__), 'autonomous_navigation')
sys.path.insert(0, autonomous_nav_path)

# Change to autonomous_navigation directory for relative imports
original_cwd = os.getcwd()
os.chdir(autonomous_nav_path)

try:
    # Import and run main simulation
    if __name__ == "__main__":
        from simulate_navigation import main
        import asyncio
        asyncio.run(main())
finally:
    # Restore original directory
    os.chdir(original_cwd)


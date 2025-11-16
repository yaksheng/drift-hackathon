#!/usr/bin/env python3
"""
Quick Calibration Script

Convenience script to run calibration from project root.
"""

import sys
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(script_dir))

# Change to script directory for relative imports
original_cwd = os.getcwd()
os.chdir(script_dir)

try:
    from calibration_tools import main
    if __name__ == "__main__":
        main()
finally:
    os.chdir(original_cwd)


#!/usr/bin/env python3
"""
Quick script to run line following navigation with real camera
"""

import sys
import os
import asyncio

# Add autonomous_navigation to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'autonomous_navigation'))

from line_following_navigation import main

if __name__ == "__main__":
    print("Starting Line Following Navigation with Real Camera...")
    print("Camera: http://192.168.0.21:8000/")
    print("Robot: 192.168.0.113")
    print()
    asyncio.run(main())


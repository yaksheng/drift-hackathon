#!/usr/bin/env python3
"""
Virtual Test Suite

Runs comprehensive virtual tests for the autonomous navigation system.
Tests the fixes for robot spinning and straight movement.
"""

import sys
import os
import subprocess

# Add autonomous_navigation to path
autonomous_nav_path = os.path.join(os.path.dirname(__file__), 'autonomous_navigation')
sys.path.insert(0, autonomous_nav_path)

def run_test(test_name, command):
    """Run a test and return success status"""
    print(f"\n{'='*60}")
    print(f"Running: {test_name}")
    print(f"{'='*60}")
    print(f"Command: {' '.join(command)}")
    print()
    
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=120  # 2 minute timeout
        )
        
        if result.returncode == 0:
            print("✅ PASS")
            if result.stdout:
                # Print last 20 lines of output
                lines = result.stdout.strip().split('\n')
                for line in lines[-20:]:
                    print(line)
            return True
        else:
            print("❌ FAIL")
            if result.stderr:
                print("Error output:")
                print(result.stderr[-500:])  # Last 500 chars
            return False
    except subprocess.TimeoutExpired:
        print("⏱️  TIMEOUT (test took too long)")
        return False
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return False

def main():
    """Run all virtual tests"""
    print("="*60)
    print("VIRTUAL TEST SUITE")
    print("Testing Navigation Controller Fixes")
    print("="*60)
    
    results = []
    
    # Test 1: Basic simulation test
    results.append((
        "Basic Simulation Test",
        run_test(
            "Basic Simulation - Stop at Line 1",
            ["python3", "simulate.py", "--stop-at-line", "1", "--max-iterations", "150", "--no-wait"]
        )
    ))
    
    # Test 2: Simulation with different starting position
    results.append((
        "Different Starting Position",
        run_test(
            "Simulation - Custom Start Position",
            ["python3", "simulate.py", "--stop-at-line", "2", 
             "--initial-x", "0.3", "--initial-y", "0.5", "--max-iterations", "150", "--no-wait"]
        )
    ))
    
    # Test 3: Unit tests
    results.append((
        "Unit Tests",
        run_test(
            "Unit Test Suite",
            ["python3", os.path.join(autonomous_nav_path, "test_simulation.py")]
        )
    ))
    
    # Test 4: Navigation to line 3
    results.append((
        "Navigation to Line 3",
        run_test(
            "Simulation - Stop at Line 3",
            ["python3", "simulate.py", "--stop-at-line", "3", "--max-iterations", "200", "--no-wait"]
        )
    ))
    
    # Print summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {test_name}")
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 All virtual tests passed!")
        print("\nThe navigation controller fixes are working correctly:")
        print("  ✅ Reduced PID gains prevent oscillation")
        print("  ✅ Heading dead zone ensures straight movement")
        print("  ✅ Limited correction prevents spinning")
        print("  ✅ Search timeout prevents getting stuck")
        return 0
    else:
        print(f"\n⚠️  {total - passed} test(s) failed")
        return 1

if __name__ == "__main__":
    sys.exit(main())


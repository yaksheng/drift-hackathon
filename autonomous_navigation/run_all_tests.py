#!/usr/bin/env python3
"""
Test Runner - Runs all test suites

Usage:
    python3 run_all_tests.py [--unit] [--integration] [--simulation] [--all]
"""

import sys
import os
import argparse

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))


def run_unit_tests():
    """Run unit tests"""
    print("\n" + "="*60)
    print("RUNNING UNIT TESTS")
    print("="*60)
    from test_unit import run_tests
    return run_tests()


def run_integration_tests():
    """Run integration tests"""
    print("\n" + "="*60)
    print("RUNNING INTEGRATION TESTS")
    print("="*60)
    from test_integration import run_tests
    return run_tests()


def run_simulation_tests():
    """Run simulation tests"""
    print("\n" + "="*60)
    print("RUNNING SIMULATION TESTS")
    print("="*60)
    import asyncio
    from test_simulation import SimulationTester
    
    tester = SimulationTester()
    success = asyncio.run(tester.run_all_tests())
    return success


def run_visualization_tests():
    """Run visualization tests"""
    print("\n" + "="*60)
    print("RUNNING VISUALIZATION TESTS")
    print("="*60)
    from test_visualization import run_tests
    return run_tests()


def main():
    parser = argparse.ArgumentParser(description='Run test suites')
    parser.add_argument('--unit', action='store_true',
                       help='Run unit tests only')
    parser.add_argument('--integration', action='store_true',
                       help='Run integration tests only')
    parser.add_argument('--simulation', action='store_true',
                       help='Run simulation tests only')
    parser.add_argument('--visualization', action='store_true',
                       help='Run visualization tests only')
    parser.add_argument('--all', action='store_true',
                       help='Run all tests')
    
    args = parser.parse_args()
    
    # Default to all if no specific test type specified
    if not any([args.unit, args.integration, args.simulation, args.visualization]):
        args.all = True
    
    results = {}
    
    if args.unit or args.all:
        results['unit'] = run_unit_tests()
    
    if args.integration or args.all:
        results['integration'] = run_integration_tests()
    
    if args.simulation or args.all:
        results['simulation'] = run_simulation_tests()
    
    if args.visualization or args.all:
        results['visualization'] = run_visualization_tests()
    
    # Print summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    all_passed = True
    for test_type, passed in results.items():
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{test_type.upper():15s}: {status}")
        if not passed:
            all_passed = False
    
    print("="*60)
    
    if all_passed:
        print("\n🎉 All tests passed!")
        return 0
    else:
        print("\n⚠️  Some tests failed. Review output above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())


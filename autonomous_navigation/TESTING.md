# Testing Guide

This document describes the testing infrastructure for the autonomous navigation system.

## Test Structure

The test suite is organized into three levels:

### 1. Unit Tests (`test_unit.py`)
Tests individual modules in isolation:
- **TargetDetector**: Color detection, target creation
- **RobotLocalizer**: Position tracking, transform matrices
- **PathPlanner**: Path planning, obstacle avoidance
- **DeadReckoning**: Odometry, camera correction
- **LineDetector**: Line detection
- **NavigationController**: State machine, PID control

**Run unit tests:**
```bash
python3 autonomous_navigation/test_unit.py
```

### 2. Integration Tests (`test_integration.py`)
Tests modules working together:
- Target detection + localization integration
- Path planning + navigation controller integration
- Dead reckoning + navigation integration
- Full navigation loop

**Run integration tests:**
```bash
python3 autonomous_navigation/test_integration.py
```

### 3. Simulation Tests (`test_simulation.py`)
End-to-end tests in simulated environment:
- Robot initialization and movement
- Sensor simulation
- Path planning
- Navigation controller
- Full navigation simulation (CLI mode)

**Run simulation tests:**
```bash
python3 autonomous_navigation/test_simulation.py
```

## Running All Tests

Use the test runner to execute all test suites:

```bash
# Run all tests
python3 autonomous_navigation/run_all_tests.py --all

# Run specific test suite
python3 autonomous_navigation/run_all_tests.py --unit
python3 autonomous_navigation/run_all_tests.py --integration
python3 autonomous_navigation/run_all_tests.py --simulation
```

## Test Coverage

### Current Coverage:
- ✅ Unit tests: 23 tests covering all core modules
- ✅ Integration tests: 4 test suites
- ✅ Simulation tests: 6 comprehensive tests

### Modules Tested:
- [x] Target Detection
- [x] Robot Localization
- [x] Path Planning
- [x] Navigation Controller
- [x] Dead Reckoning
- [x] Line Detection
- [x] Simulator
- [x] Mock Robot

## Writing New Tests

### Unit Test Example:
```python
class TestMyModule(unittest.TestCase):
    def setUp(self):
        self.module = MyModule()
    
    def test_feature(self):
        result = self.module.do_something()
        self.assertEqual(result, expected_value)
```

### Integration Test Example:
```python
class TestModuleIntegration(unittest.TestCase):
    def setUp(self):
        self.module1 = Module1()
        self.module2 = Module2()
    
    def test_modules_work_together(self):
        result = self.module1.process(self.module2.get_data())
        self.assertIsNotNone(result)
```

## Test Best Practices

1. **Isolation**: Each test should be independent
2. **Clear Names**: Test names should describe what they test
3. **Setup/Teardown**: Use setUp() and tearDown() for common setup
4. **Assertions**: Use specific assertions (assertEqual, assertIsNotNone, etc.)
5. **Mocking**: Use mocks for external dependencies (robot, camera)

## Continuous Integration

Tests can be integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions
- name: Run tests
  run: |
    cd autonomous_navigation
    python3 run_all_tests.py --all
```

## Troubleshooting

### Common Issues:

1. **Import Errors**: Ensure parent directory is in Python path
2. **Mock Robot**: Ensure mock robot has all required attributes
3. **Async Tests**: Use `asyncio.run()` for async test functions
4. **Time-dependent Tests**: Add small delays for time-sensitive tests

## Test Results

Last run: All 23 unit tests passing ✅


# Test Suite Architecture

This directory contains the integration and structural tests for the
`franka_bringup` package. Tests are organized by what they verify and
what hardware they require.

## Test Categories

| Category | Requires Hardware | Framework | Purpose |
|----------|:-:|---|---|
| Fake-hardware integration | ❌ | launch_testing | Verify controller lifecycle with `mock_components/GenericSystem` |
| Interface validation | ❌ | pytest | Assert URDF xacro expands to correct ros2_control interfaces |
| Launch file parsing | ❌ | pytest | Verify all launch files import and generate valid descriptions |
| Teleop plumbing | ❌ | launch_testing | Verify Joy→cmd_vel pipeline works |
| Controller ordering | ❌ | launch_testing | Verify startup ordering, no error states |
| Hardware smoke tests | ✅ | launch_testing | Run controllers on real robot for fixed duration |
| Gazebo integration | ❌* | launch_testing | Controller tests against Gazebo simulation |

\* Requires Gazebo installed in the environment.

## File Layout

```
test/
├── config/                              # YAML configs for test scenarios
│   ├── test_use_fake_hardware.config.yaml   # FR3 single-arm (use_fake_hardware=false)
│   ├── test_fake_hardware_fr3.config.yaml   # FR3 single-arm (fake hw)
│   ├── test_fake_hardware_fr3_duo.config.yaml
│   ├── test_fake_hardware_mobile_fr3_duo.config.yaml
│   ├── test_0.config.yaml ... test_1.config.yaml  # Real-hardware configs
│   └── ...
├── test_fake_hardware_controllers.py    # FR3 controller lifecycle (fake hw)
├── test_fake_hardware_fr3_duo.py        # FR3 Duo integration (fake hw)
├── test_fake_hardware_mobile_fr3_duo.py # Mobile FR3 Duo integration (fake hw)
├── test_franka_arm_interfaces.py        # Single-arm URDF interface validation
├── test_fr3_duo_interfaces.py           # Dual-arm URDF interface validation
├── test_mobile_fr3_duo_interfaces.py    # Mobile URDF interface validation
├── test_controller_activation_order.py  # Verifies startup ordering, no error states
├── test_teleop_plumbing.py              # Joy → cmd_vel pipeline validation
├── test_launch_file_parsing.py          # Launch file structural validation
├── test_hardware_example_controllers.py # Real-hardware controller tests
├── test_hardware_generic_controller.py  # Real-hardware generic test framework
├── test_gazebo_wrapper_expansion.py     # Gazebo URDF expansion test
└── ros2_control_test_helpers.py         # Shared URDF parsing utilities
```

## Shared Testing Utilities

Located in `franka_bringup/franka_bringup/testing/`:

- **`fake_hardware_test_base.py`** — `FakeHardwareTestBase` class providing
  ROS lifecycle management, joint_state waiting, and error assertion for all
  fake-hardware tests.
- **`controller_service_client.py`** — Service client wrapper for
  controller_manager operations (load, configure, switch, unload).
- **`controller_test_utils.py`** — Higher-level utilities for hardware tests
  (move_to_start workflow, smoke test runner).

## Fake-Hardware Test Design

All fake-hardware tests inherit from `FakeHardwareTestBase` and share:

1. **ROS lifecycle** — `rclpy.init()` in `setUpClass`, shutdown in
   `tearDownClass`, per-test node creation in `setUp`/`tearDown`.
2. **Stack readiness check** — `wait_for_stack_ready()` waits for
   controller_manager services and joint_state_broadcaster activation.
3. **Joint state verification** — `wait_for_joint_states(min_joints=N)`
   subscribes to `/joint_states` and waits for messages with enough joints.
4. **Error output assertion** — `assert_no_errors_in_output(proc_output)`
   with optional `ignore_patterns` for expected errors.

### Known Limitations

- **swerve_drive_controller** cannot activate with `mock_components` because
  it requires `0/cartesian_pose_state` interfaces from real TMR hardware.
  The Mobile FR3 Duo test filters this expected error.
- **Franka GPIO controllers** (cartesian_pose, cartesian_velocity, etc.)
  cannot activate because `GenericSystem` doesn't provide GPIO interfaces.
  These are tested at load-only level.
- **`franka_fr3_moveit_config`** cannot be declared as a `test_depend` in
  `franka_bringup/package.xml` because it would create a circular dependency.
  The launch parsing test for MoveIt files works in full-workspace builds (CI)
  but may fail in isolated `--packages-up-to franka_bringup` builds.

## Running Tests

```bash
# All no-hardware tests (CI default)
colcon test --packages-select franka_bringup \
  --ctest-args --exclude-regex 'test_hardware|test_gazebo_launch|test_gazebo_contact'

# Only fake-hardware integration tests
colcon test --packages-select franka_bringup \
  --ctest-args --tests-regex 'test_fake_hardware|test_controller_activation'

# Only interface/structural tests
colcon test --packages-select franka_bringup \
  --ctest-args --tests-regex 'test_franka_arm|test_fr3_duo|test_mobile_fr3_duo|test_launch'

# Hardware tests (requires robot_ip, see scripts/run_hardware_tests.sh)
colcon test --packages-select franka_bringup \
  --ctest-args --tests-regex 'test_hardware' -- --robot_ip <ip>
```

## Adding New Tests

1. For a new fake-hardware test: inherit from `FakeHardwareTestBase`,
   define test methods, and provide `generate_test_description()`.
2. For a new interface test: use `ros2_control_test_helpers.py` utilities
   to expand xacro and assert on the parsed hardware components.
3. Register in `CMakeLists.txt` using `add_launch_test()` or
   `ament_add_pytest_test()`.

# Repository Guidelines

## Project Structure & Module Organization

- `libneupan/`: ROS-independent C++17 implementation of DUNE, NRMP, and PAN. Public headers live in `include/neupan/`, implementations in `src/`, GoogleTest suites and NPTF fixtures in `tests/`, exported weights in `models/`, and conversion/benchmark tools in `tools/`.
- `neupan_cpp_ros/`: ROS 2 Humble planner and reference A* nodes, with `config/`, `launch/`, and manual Python drivers in `test/`.
- `docs/media/` stores demonstrations. Keep `README.md` and `README_en.md` aligned when changing documented behavior.

## Build, Test, and Development Commands

Run from the repository root in Linux/WSL. Install Eigen, OSQP/OsqpEigen, yaml-cpp, and GoogleTest; ROS integration requires Humble.

```bash
cmake -S libneupan -B build/core -DNEUPAN_BUILD_TESTS=ON
cmake --build build/core -j
ctest --test-dir build/core --output-on-failure
./build/core/neupan_bench
```

These configure/build the standalone core, run its tests, and benchmark recorded frames.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to neupan_cpp_ros
source install/setup.bash
ros2 launch neupan_cpp_ros neupan.launch.py
```

This builds both packages and starts the planner plus reference global planner.

## Coding Style & Naming Conventions

Follow existing C++ style: two-space indentation, attached braces, `PascalCase` types, `camelCase` methods, `snake_case` variables/files, and trailing underscores for private members. Python uses four spaces and `snake_case`. Keep algorithm logic in the core and ROS transport in the wrapper. No formatter/linter configuration is currently checked in.

## Testing Guidelines

Name C++ tests `libneupan/tests/test_<feature>.cpp`, use `TEST(Suite, Behavior)`, and register targets in CMake. Preserve Python-reference parity tolerances; measure performance with benchmarks rather than timing assertions. No coverage percentage is configured.

With the launch running in an isolated ROS environment, run `python3 neupan_cpp_ros/test/smoke_test.py` separately. These drivers publish synthetic sensor/TF data and are not registered with CTest.

## Commit & Pull Request Guidelines

Use observed prefixes: `feat:`, `fix:`, `docs:`, `test:`, `refactor:`; optional scopes include `feat(ros):`. Keep subjects concise. PRs should explain behavior changes, link relevant issues, and report commands/results; include visualization evidence when applicable.

## Configuration & Remote Work

Keep tracked `models/*.bin` and `tests/data/*.nptf`; model footprints must match planner YAML. Exclude build/install/log artifacts.

For Jetson work, use desktop `ssh jetson-orin` with Windows SSH configuration/keys for every remote operation. Read and confirm paths before editing; avoid deletion or changes outside the project unless explicitly requested. Keep Codex and model interactions on the desktop.

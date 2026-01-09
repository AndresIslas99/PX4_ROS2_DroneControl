# Contributing to PX4 ROS 2 Drone Control Workspace

Thank you for your interest in contributing to this project! This document provides guidelines and information for contributors.

## How to Contribute

### Reporting Bugs

Before submitting a bug report:

1. Check the [Troubleshooting Guide](docs/TROUBLESHOOTING.md) for common issues
2. Search existing [Issues](https://github.com/AndresIslas99/PX4_ROS2_DroneControl/issues) to avoid duplicates
3. Try running `./diagnose_and_fix.sh` to identify the problem

When reporting a bug, include:

- Operating system and version
- ROS 2 distribution (e.g., Humble)
- PX4 version
- Steps to reproduce the issue
- Expected vs actual behavior
- Relevant logs (from `/tmp/px4_startup.log`, `ros2 topic list`, etc.)

### Suggesting Features

Feature requests are welcome! Please:

1. Check existing issues to avoid duplicates
2. Describe the use case and benefit
3. Provide examples if possible

### Submitting Pull Requests

1. **Fork the repository** and create your branch from `main`
2. **Follow the code style** (see below)
3. **Test your changes** thoroughly
4. **Update documentation** if needed
5. **Write a clear PR description** explaining what and why

## Development Setup

```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/PX4_ROS2_DroneControl.git
cd PX4_ROS2_DroneControl

# Build the workspace
source /opt/ros/humble/setup.bash
colcon build

# Source the workspace
source install/setup.bash

# Run tests
./test_all_features.sh
```

## Code Style

### Python

- Follow [PEP 8](https://pep8.org/) style guidelines
- Use meaningful variable and function names
- Add docstrings to functions and classes
- Use type hints where appropriate

```python
def calculate_distance(point_a: tuple, point_b: tuple) -> float:
    """Calculate Euclidean distance between two points.

    Args:
        point_a: First point as (x, y, z) tuple
        point_b: Second point as (x, y, z) tuple

    Returns:
        Distance between the points
    """
    ...
```

### C++

- Follow the [ROS 2 C++ Style Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use `snake_case` for functions and variables
- Use `CamelCase` for class names
- Include header guards or `#pragma once`

```cpp
class DroneController : public rclcpp::Node
{
public:
    DroneController();

private:
    void publish_setpoint();
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
};
```

### ROS 2 Conventions

- Follow [REP-103](https://www.ros.org/reps/rep-0103.html) for units and coordinate frames
- Use NED (North-East-Down) frame for PX4 compatibility
- Name topics following ROS 2 conventions

## Project Structure

When adding new features:

- **New ROS 2 packages**: Add to `src/` directory
- **Python scripts**: Add to appropriate package's `scripts/` or module directory
- **C++ nodes**: Add source to `src/` and headers to `include/`
- **Launch files**: Add to package's `launch/` directory
- **Documentation**: Add to `docs/` directory

## Testing

Before submitting a PR:

1. **Build succeeds**: `colcon build`
2. **System tests pass**: `./test_all_features.sh`
3. **Manual testing**: Verify your feature works with the GUI and/or Web GCS
4. **No regressions**: Ensure existing functionality still works

## Commit Messages

Use clear, descriptive commit messages:

```
feat: add waypoint mission support to web GCS

- Implement waypoint creation UI
- Add mission upload to PX4
- Include progress tracking during mission

Closes #123
```

Prefixes:
- `feat:` - New feature
- `fix:` - Bug fix
- `docs:` - Documentation only
- `style:` - Code style (formatting, etc.)
- `refactor:` - Code refactoring
- `test:` - Adding tests
- `chore:` - Maintenance tasks

## Questions?

If you have questions about contributing:

1. Check existing documentation in `docs/`
2. Open a discussion or issue on GitHub
3. Review the [PX4 documentation](https://docs.px4.io/)
4. Review the [ROS 2 documentation](https://docs.ros.org/en/humble/)

## License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.

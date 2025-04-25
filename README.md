# Coordination Manager

This project develops a mid-level coordination manager for handling critical scenarios.

---

## Prerequisites

### Required Tools
- **C++ Compiler**: A modern C++ compiler (e.g., GCC or Clang).
- **SQLite**: For storing aggregated data. Install with:
  ```bash
  sudo apt install sqlite3 libsqlite3-dev
  ```
- **ROS 2**: ROS 2 Humble or later, including its build tools and dependencies.

---

## Build and Run Instructions

### Build the Project
1. Source your ROS 2 Humble installation:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Navigate to your ROS 2 workspace:
3. Build the workspace using `colcon`:
   ```bash
   colcon build
   ```

### Run the Node
1. Source the workspace after building:
   ```bash
   source install/setup.bash
   ```
2. Run the `coordination_manager_launch`:
   ```bash
   ros2 launch coordination_manager coordination_manager_launch
   ```

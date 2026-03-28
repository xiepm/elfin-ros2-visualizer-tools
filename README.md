# Elfin ROS2 Visualizer Workspace

This repository is a complete ROS2 workspace layout for Elfin robots.

After cloning, you can build directly from the repository root:

```bash
cd elfin-ros2-visualizer-tools
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
bash run_elfin_visualize.sh elfin5
```

## Workspace Layout

```text
elfin-ros2-visualizer-tools/
  control_elfin.py
  elfin_visualize.launch.py
  elfin_visualize.rviz
  run_elfin_visualize.sh
  src/
    elfin_robot_ros2/
```

## Included Visualization Fixes

- RViz loads the robot model from a generated URDF file instead of relying on `/robot_description` topic timing
- `joint_states` publishing is decoupled from the Tk GUI event loop, so dragging sliders is more stable
- switching robot model from the control panel restarts the full visualization chain
- stale `rviz2`, `robot_state_publisher`, and `joint_state_publisher` processes are cleaned before relaunch

## Supported Models

- `elfin3`
- `elfin5`
- `elfin5_l`
- `elfin10`
- `elfin10_l`
- `elfin15`

## Notes

- Switch robot models from the control panel dropdown, not from RViz properties.
- Temporary generated files are written to `/tmp/`.

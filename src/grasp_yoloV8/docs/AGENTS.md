# Repository Guidelines

## Project Structure & Module Organization

This repository contains two related ROS codebases. Use `rm_yolo_ros2_ws/src/ros2_rm_robot/` as the active ROS 2 Humble workspace source tree. Its packages include `rm_driver` for hardware communication, `rm_control` for MoveIt/control bridging, `rm_bringup` for launch orchestration, `rm_description` for URDF, meshes, RViz, and joint configs, `rm_gazebo` for simulation, `rm_moveit2_config/*` for robot-specific MoveIt 2 configs, and `rm_ros_interfaces` for custom messages. Example nodes live in `rm_example/` and `rm_arm_examples/`.

`original_yolov8_ros1/` is the original ROS 1/Noetic YOLOv8 vision-grab reference, including `rm_robot/`, `vi_grab/`, `vi_msgs/`, and visual assets in `pic/`. Treat `rm_yolo_ros2_ws/build/`, `install/`, and `log/` as generated colcon output.

## Build, Test, and Development Commands

Run ROS 2 commands from `rm_yolo_ros2_ws/`:

```bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
colcon test
colcon test-result --verbose
```

Build `rm_ros_interfaces` first after changing messages so dependent packages can resolve generated types. Launch examples after sourcing the workspace, for example `ros2 launch rm_bringup rm_65_gazebo.launch.py` for simulation or `ros2 launch rm_bringup rm_65_bringup.launch.py` for hardware.

For the ROS 1 vision tree, install Python dependencies from `original_yolov8_ros1/requirements.txt` when using the YOLOv8 demo.

## Coding Style & Naming Conventions

Follow existing ROS package conventions. Use C++ package code under `src/` with public headers under `include/`; prefer `snake_case` for package names, launch files, YAML files, and ROS topic/config identifiers. Keep launch files named by robot model and purpose, such as `rm_65_gazebo.launch.py`. Preserve model-specific prefixes already used in URDF and MoveIt config directories (`rm_65`, `rm_75`, `rm_eco65`, `rm_gen72`).

## Testing Guidelines

Most ROS 2 packages enable `ament_lint_auto` under `BUILD_TESTING`; run `colcon test` before submitting changes. Add focused tests beside the package being changed when introducing new logic, and at minimum verify build, lint, and one relevant launch path. For message changes, rebuild `rm_ros_interfaces` and any package that consumes the changed interface.

## Commit & Pull Request Guidelines

Nested histories use short imperative or scoped documentation commits, for example `Fix case sensitivity in arm64 library path` and `doc:add version`. Keep commits focused and mention the affected package when useful, such as `rm_driver: add UDP config validation`.

Pull requests should describe the changed package, target robot model, tested commands, and whether hardware, Gazebo, or both were exercised. Include screenshots or logs for RViz/Gazebo or vision changes, and link any related issue or upstream RealMan documentation.

## Safety & Configuration Tips

Do not commit generated `build/`, `install/`, or `log/` artifacts. Keep robot IPs, controller versions, and hardware-specific parameters in package config YAML files, and document non-default values in the PR. For real-arm testing, confirm workspace sourcing and launch arguments before enabling motion.

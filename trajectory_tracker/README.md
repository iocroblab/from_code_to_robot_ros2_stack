# `trajectory_tracker`

ROS 2 Python node that tracks a TF frame over time and publishes its recent path as a `sensor_msgs/msg/PointCloud2`. It is intended for visualizing the manipulator tool trajectory in RViz.

## What it does

- Looks up the transform from `world_frame` to `tracked_frame`.
- Samples that transform at a fixed rate.
- Stores the last `num_points` positions.
- Publishes the path as a yellow point cloud on `/trajectory_path_live`.

Default parameters:

- `tracked_frame`: `tool0`
- `world_frame`: `world`
- `publish_rate`: `30.0`
- `num_points`: `200`

Because the publisher uses transient-local durability, RViz can display the latest cloud even if it subscribes after the node starts.

## Build

From the workspace root:

```bash
colcon build --packages-select trajectory_tracker
source install/setup.bash
```

## Launch

Run the node directly:

```bash
ros2 run trajectory_tracker track_trajectory
```

Example with custom parameters:

```bash
ros2 run trajectory_tracker track_trajectory --ros-args -p tracked_frame:=tool0 -p world_frame:=world -p publish_rate:=50.0
```

Example with a custom history length:

```bash
ros2 run trajectory_tracker track_trajectory --ros-args -p num_points:=500
```

## Typical use

Launch the manipulator first so the TF tree exists, then start the tracker:

```bash
ros2 launch threelink_manipulator spawn_robot.launch.py
ros2 run trajectory_tracker track_trajectory
```

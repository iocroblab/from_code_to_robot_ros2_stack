# `safety_nodes`

Small ROS 2 safety helper package for forcing controller command topics back to zero when command publishing stops unexpectedly.

## What it does

The `safety_nullifier` node subscribes to controller command topics, watches for recent non-zero commands, and if the command stream times out it publishes several all-zero command messages to the same topic. This is useful when a teleop or control node exits without explicitly stopping the robot.

By default it monitors:

- `/forward_effort_controller/commands`
- `/forward_velocity_controller/commands`

Main parameters:

- `effort_topic`: effort controller command topic
- `velocity_topic`: velocity controller command topic
- `detection_timeout`: how long to wait before forcing zeros
- `active_detection_rate`: timer rate for the currently active controller
- `inactive_detection_rate`: timer rate for inactive controllers
- `num_joints`: number of values written into the zero command
- `zero_burst_count`: how many zero messages to publish per stop event

## Build

From the workspace root:

```bash
colcon build --packages-select safety_nodes
source install/setup.bash
```

## Launch

Run the node directly:

```bash
ros2 run safety_nodes safety_nullifier
```

For the `threelink_manipulator` package, override `num_joints` to `3`:

```bash
ros2 run safety_nodes safety_nullifier --ros-args -p num_joints:=3
```

Example with explicit topics:

```bash
ros2 run safety_nodes safety_nullifier --ros-args \
  -p effort_topic:=/forward_effort_controller/commands \
  -p velocity_topic:=/forward_velocity_controller/commands \
  -p num_joints:=3
```

## Typical use

Start the simulator or robot controllers first, then run `safety_nullifier` in the background so it can watch the active command topics continuously.

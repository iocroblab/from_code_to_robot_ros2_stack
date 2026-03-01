# `tutorialteleop`

Keyboard teleoperation package for publishing `geometry_msgs/msg/Twist` commands and a small mode/status topic for downstream nodes.

## What it does

The node publishes:

- `geometry_msgs/msg/Twist` on `cmd_vel` by default
- `std_msgs/msg/String` on `teleop_mode`

`teleop_mode` reports the active reference frame:

- `Base`
- `EndEffector`

The teleop interface supports:

- `W/A/S/D` for planar motion
- `Space` for positive Z
- `-` for negative Z
- `Shift` with motion keys for `2x` speed
- `Tab` to toggle between linear and rotation mode
- `M` to toggle the reference frame
- `Ctrl-C` to quit

Executable:

- `tutorial_teleop`: focused local teleop window, only reacts while that window is focused

## Build

From the workspace root:

```bash
colcon build --packages-select tutorialteleop
source install/setup.bash
```

## Launch

Run the main teleop node:

```bash
ros2 run tutorialteleop tutorial_teleop
```

Example with custom speeds and topic:

```bash
ros2 run tutorialteleop tutorial_teleop --ros-args -p linear_speed:=0.1 -p angular_speed:=0.8 -p topic:=/cmd_vel
```

## Notes

This package does not include a launch file. It is usually started in a separate terminal after the robot simulation or control stack is already running.

The node opens a small local Tk window and only reacts while that window is focused. This also allows proper key hold/release handling and combinations such as `W+A`.

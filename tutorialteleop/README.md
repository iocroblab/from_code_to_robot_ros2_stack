# `tutorialteleop`

Keyboard teleoperation package for publishing `geometry_msgs/msg/Twist` commands and a small mode/status topic for downstream nodes.

## What it does

Both executables publish:

- `geometry_msgs/msg/Twist` on `cmd_vel` by default
- `std_msgs/msg/String` on `teleop_mode`

`teleop_mode` reports the active reference frame:

- `Base`
- `EndEffector`

The teleop interface supports:

- `W/A/S/D` for planar motion
- `Space` for positive Z
- `Tab` to toggle between linear and rotation mode
- `M` to toggle the reference frame
- `Shift` to double the command speed
- `Q` or `Esc` to quit

Executables:

- `tutorial_teleop`: terminal-friendly version using `-` for negative Z
- `tutorial_teleop_global`: alternate version using `Ctrl` for negative Z

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

Run the alternate variant:

```bash
ros2 run tutorialteleop tutorial_teleop_global
```

Example with custom speeds and topic:

```bash
ros2 run tutorialteleop tutorial_teleop --ros-args -p linear_speed:=0.1 -p angular_speed:=0.8 -p topic:=/cmd_vel
```

## Notes

This package does not include a launch file. It is usually started in a separate terminal after the robot simulation or control stack is already running.

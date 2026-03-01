# `threelink_manipulator`

ROS 2 Jazzy package for a simple 3-link manipulator simulated in Gazebo Harmonic. It contains the robot description, controller configuration, a launch file that starts the simulation stack, and a payload service node that can spawn, attach, detach, and delete a box payload in the simulator.

## What it does

- Publishes the robot description from `urdf/threelink.urdf.xacro`.
- Starts `robot_state_publisher`.
- Starts Gazebo with `worlds/freespace.sdf`.
- Spawns the robot into Gazebo through `ros_gz_sim`.
- Starts `joint_state_broadcaster` and one initial forward controller.
- Bridges `/clock`, Gazebo world services, and the payload attach/detach topics.
- Optionally starts RViz with `rviz/view_robot.rviz`.
- Optionally starts `payload_attach_server.py`, which exposes `/payload/set_mass`.

The default launch setup activates `forward_effort_controller`, so command messages are expected on:

- `/forward_effort_controller/commands`

Other controllers are also configured and can be started manually:

- `/forward_position_controller/commands`
- `/forward_velocity_controller/commands`

## Build

From the workspace root:

```bash
colcon build --packages-select threelink_manipulator
source install/setup.bash
```

## Launch

Start the full simulation:

```bash
ros2 launch threelink_manipulator spawn_robot.launch.py
```

Useful launch arguments:

```bash
ros2 launch threelink_manipulator spawn_robot.launch.py gazebo_gui:=true
ros2 launch threelink_manipulator spawn_robot.launch.py launch_rviz:=false
ros2 launch threelink_manipulator spawn_robot.launch.py initial_joint_controller:=forward_velocity_controller
ros2 launch threelink_manipulator spawn_robot.launch.py launch_payload_service:=false
```

## Payload service

The payload server uses:

- Service: `/payload/set_mass`
- Type: `example_interfaces/srv/AddTwoInts`

Request fields are interpreted as:

- `a`: payload mass in grams
- `b`: mode, where `1` means attach/spawn and `0` means detach/delete

Examples:

```bash
ros2 service call /payload/set_mass example_interfaces/srv/AddTwoInts "{a: 500, b: 1}"
ros2 service call /payload/set_mass example_interfaces/srv/AddTwoInts "{a: 0, b: 0}"
```

When enabled, the payload node also publishes a marker on `/payload_marker` and uses `/payload/attach` and `/payload/detach` to trigger the Gazebo detachable joint flow.

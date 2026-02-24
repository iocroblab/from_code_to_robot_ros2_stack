#!/usr/bin/env python3

import subprocess
import time
import math
from typing import Optional, Tuple

import rclpy
from example_interfaces.srv import AddTwoInts
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from ros_gz_interfaces.msg import EntityFactory
from ros_gz_interfaces.srv import ControlWorld, DeleteEntity, SpawnEntity
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class PayloadAttachServer(Node):
    def __init__(self) -> None:
        super().__init__('payload_attach_server')

        # Use a reentrant callback group so service callbacks can wait on service clients.
        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.world_name = self.declare_parameter('world_name', 'empty').value
        self.reference_frame = self.declare_parameter('reference_frame', 'world').value
        # Prefer tool0 for spawn pose; detachable joint still attaches to elbow_link via the plugin.
        self.tool_frame = self.declare_parameter('tool_frame', 'tool0').value
        self.fallback_tool_frame = self.declare_parameter('fallback_tool_frame', 'elbow_link').value

        # Tool-frame offsets (in tool frame).
        self.tool_offset_x = float(self.declare_parameter('tool_offset_x', 0.0).value)
        self.tool_offset_y = float(self.declare_parameter('tool_offset_y', 0.0).value)
        self.tool_offset_z = float(self.declare_parameter('tool_offset_z', 0.0).value)
        # When enabled, offset in +X by half payload length.
        self.tool_offset_x_auto = bool(self.declare_parameter('tool_offset_x_auto', True).value)
        self.attach_clearance = float(self.declare_parameter('attach_clearance', 0.0).value)
        # Use the direction toward a forward frame (tool0) instead of assuming +X.
        self.forward_frame = str(self.declare_parameter('forward_frame', 'tool0').value)
        self.use_forward_direction = bool(self.declare_parameter('use_forward_direction', True).value)
        # Account for robot spawn offset in Gazebo when TF is still rooted at world.
        self.robot_spawn_offset_x = float(self.declare_parameter('robot_spawn_offset_x', 0.0).value)
        self.robot_spawn_offset_y = float(self.declare_parameter('robot_spawn_offset_y', 0.0).value)
        self.robot_spawn_offset_z = float(self.declare_parameter('robot_spawn_offset_z', 0.0).value)

        self.payload_name = self.declare_parameter('payload_name', 'payload_box').value
        # This must match the detachable joint plugin configuration.
        self.payload_link_name = self.declare_parameter('payload_link_name', 'payload_link').value
        if self.payload_link_name != 'payload_link':
            self.get_logger().warn(
                f"payload_link_name must be 'payload_link' for the DetachableJoint plugin; overriding '{self.payload_link_name}'."
            )
            self.payload_link_name = 'payload_link'

        self.payload_size_x = float(self.declare_parameter('payload_size_x', 0.2).value)
        self.payload_size_y = float(self.declare_parameter('payload_size_y', 0.2).value)
        self.payload_size_z = float(self.declare_parameter('payload_size_z', 0.2).value)
        self.tool_offset_x_effective = (
            (self.payload_size_x / 2.0) + self.attach_clearance if self.tool_offset_x_auto else self.tool_offset_x
        )
        if self.tool_offset_x_auto:
            self.get_logger().info(
                f'Using automatic tool_offset_x={self.tool_offset_x_effective:.3f} '
                f'(payload_size_x/2 + clearance={self.attach_clearance:.3f}).'
            )

        self.marker_topic = self.declare_parameter('marker_topic', '/payload_marker').value
        self.attach_topic = self.declare_parameter('attach_topic', '/payload/attach').value
        self.detach_topic = self.declare_parameter('detach_topic', '/payload/detach').value

        self.service_timeout_sec = float(self.declare_parameter('service_timeout_sec', 8.0).value)
        self.pause_wait_sec = float(self.declare_parameter('pause_wait_sec', 1.0).value)
        self.broadcast_rate = float(self.declare_parameter('broadcast_rate', 20.0).value)
        # Robustness knobs for the detachable joint lifecycle.
        self.post_spawn_wait_sec = float(self.declare_parameter('post_spawn_wait_sec', 0.5).value)
        self.post_detach_wait_sec = float(self.declare_parameter('post_detach_wait_sec', 0.3).value)
        self.attach_burst_count = int(self.declare_parameter('attach_burst_count', 5).value)
        self.detach_burst_count = int(self.declare_parameter('detach_burst_count', 5).value)
        self.burst_interval_sec = float(self.declare_parameter('burst_interval_sec', 0.05).value)
        self.trigger_step_count = int(self.declare_parameter('trigger_step_count', 5).value)
        self.detach_before_attach = bool(self.declare_parameter('detach_before_attach', True).value)
        # Allow the physics system to process detach events before deletion.
        self.detach_unpause_sec = float(self.declare_parameter('detach_unpause_sec', 0.2).value)
        self.marker_lifetime_sec = float(self.declare_parameter('marker_lifetime_sec', 0.5).value)
        self.marker_delete_burst_count = int(self.declare_parameter('marker_delete_burst_count', 3).value)
        self.marker_delete_interval_sec = float(self.declare_parameter('marker_delete_interval_sec', 0.05).value)

        # Ensure simulation time is used unless the user explicitly overrides it.
        if 'use_sim_time' not in self._parameter_overrides:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.effective_tool_frame = self._resolve_tool_frame()

        # QoS for latched marker publisher
        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = ReliabilityPolicy.RELIABLE

        # Publishers
        self.attach_pub = self.create_publisher(Empty, self.attach_topic, 10)
        self.detach_pub = self.create_publisher(Empty, self.detach_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, marker_qos)

        # Gazebo services
        control_service = f'/world/{self.world_name}/control'
        spawn_service = f'/world/{self.world_name}/create'
        delete_service = f'/world/{self.world_name}/remove'

        self.control_client = self.create_client(ControlWorld, control_service, callback_group=self.callback_group)
        self.spawn_client = self.create_client(SpawnEntity, spawn_service, callback_group=self.callback_group)
        self.delete_client = self.create_client(DeleteEntity, delete_service, callback_group=self.callback_group)

        # State
        self.payload_present = False
        self.marker_timer: Optional[rclpy.timer.Timer] = None
        self.marker_active = False
        self._forward_direction_warned = False

        # Service server
        self.srv = self.create_service(
            AddTwoInts,
            '/payload/set_mass',
            self.handle_set_mass,
            callback_group=self.callback_group,
        )

        # Bridge check and startup logs
        self._check_sim_processes()
        self._check_bridge_status()
        self.get_logger().info(
            f'Ready: /payload/set_mass (AddTwoInts). world={self.world_name} '
            f'reference_frame={self.reference_frame} tool_frame={self.effective_tool_frame}'
        )

    def _resolve_tool_frame(self) -> str:
        """Resolve tool frame; fall back to elbow_link if tool0 is missing."""
        requested_tool = str(self.tool_frame)
        timeout = Duration(seconds=min(self.service_timeout_sec, 1.0))

        try:
            if self.tf_buffer.can_transform(self.reference_frame, requested_tool, Time(), timeout):
                return requested_tool
        except Exception:
            # TF buffer may not be ready yet; we will retry on demand.
            return requested_tool

        if requested_tool == 'tool0':
            fallback = str(self.fallback_tool_frame)
            try:
                if self.tf_buffer.can_transform(self.reference_frame, fallback, Time(), timeout):
                    self.get_logger().warn(
                        f"tool_frame '{requested_tool}' not available; falling back to '{fallback}' based on TF availability."
                    )
                    return fallback
            except Exception:
                return requested_tool

        return requested_tool

    def _check_bridge_status(self) -> None:
        """Check for a parameter_bridge process that bridges Empty attach/detach topics."""
        bridge_cmd = (
            'ros2 run ros_gz_bridge parameter_bridge '
            '/payload/attach@std_msgs/msg/Empty]gz.msgs.Empty '
            '/payload/detach@std_msgs/msg/Empty]gz.msgs.Empty'
        )

        try:
            result = subprocess.run(
                ['ps', '-ef'],
                check=False,
                capture_output=True,
                text=True,
            )
            output = result.stdout
        except Exception as exc:  # pragma: no cover - best effort only
            self.get_logger().warn(f'Unable to inspect running processes for ros_gz_bridge: {exc}')
            self.get_logger().warn('Ensure the attach/detach bridge is running:')
            self.get_logger().warn(bridge_cmd)
            return

        has_parameter_bridge = 'ros_gz_bridge/parameter_bridge' in output or 'ros_gz_bridge parameter_bridge' in output
        attach_ros_to_gz = '/payload/attach@std_msgs/msg/Empty]gz.msgs.Empty' in output
        detach_ros_to_gz = '/payload/detach@std_msgs/msg/Empty]gz.msgs.Empty' in output
        attach_bidir = '/payload/attach@std_msgs/msg/Empty@gz.msgs.Empty' in output
        detach_bidir = '/payload/detach@std_msgs/msg/Empty@gz.msgs.Empty' in output
        attach_gz_to_ros = '/payload/attach@std_msgs/msg/Empty[gz.msgs.Empty' in output
        detach_gz_to_ros = '/payload/detach@std_msgs/msg/Empty[gz.msgs.Empty' in output
        has_bool_attach = '/payload/attach@std_msgs/msg/Bool' in output
        has_bool_detach = '/payload/detach@std_msgs/msg/Bool' in output

        has_correct_attach = attach_ros_to_gz or attach_bidir
        has_correct_detach = detach_ros_to_gz or detach_bidir

        if has_correct_attach and has_correct_detach:
            self.get_logger().info('Detected ros_gz_bridge parameter_bridge for ROS->Gazebo Empty attach/detach topics.')
            return

        if attach_gz_to_ros or detach_gz_to_ros:
            self.get_logger().error(
                'Detected attach/detach bridge in Gazebo->ROS direction. '
                'DetachableJoint requires ROS->Gazebo for triggers.'
            )

        if has_parameter_bridge and (has_bool_attach or has_bool_detach):
            self.get_logger().error(
                'Detected a parameter_bridge for /payload attach/detach using Bool. '
                'DetachableJoint expects Empty; update the bridge command.'
            )
        else:
            self.get_logger().error(
                'No ros_gz_bridge parameter_bridge detected for /payload/attach and /payload/detach as Empty.'
            )

        self.get_logger().error('Attach/detach triggers will not reach Gazebo without this bridge:')
        self.get_logger().error(bridge_cmd)

    def _check_sim_processes(self) -> None:
        """Warn loudly if multiple Gazebo / bridge instances appear to be running."""
        try:
            result = subprocess.run(
                ['ps', '-ef'],
                check=False,
                capture_output=True,
                text=True,
            )
            lines = result.stdout.splitlines()
        except Exception as exc:  # pragma: no cover - best effort only
            self.get_logger().warn(f'Unable to inspect running processes for Gazebo/bridge instances: {exc}')
            return

        gz_sim_lines = [
            line for line in lines
            if 'gz sim -' in line and 'gz sim server' not in line and 'gz sim gui' not in line
        ]
        bridge_lines = [line for line in lines if 'ros_gz_bridge/parameter_bridge' in line]

        if len(gz_sim_lines) > 1:
            self.get_logger().error(
                f'Detected multiple Gazebo Sim instances ({len(gz_sim_lines)}). '
                'Attachment can fail or appear to detach immediately. Stop old instances and relaunch.'
            )
        if len(bridge_lines) > 1:
            self.get_logger().error(
                f'Detected multiple ros_gz_bridge parameter_bridge instances ({len(bridge_lines)}). '
                'Services/topics may be routed to the wrong simulation.'
            )

    def _services_ready(self) -> bool:
        """Wait for Gazebo services; return False if any are unavailable."""
        clients = [
            ('control', self.control_client),
            ('spawn', self.spawn_client),
            ('delete', self.delete_client),
        ]
        for name, client in clients:
            if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self.get_logger().error(
                    f"Service '{name}' not available after {self.service_timeout_sec:.2f} seconds."
                )
                return False
        return True

    def _call_service(self, client, request, label: str) -> Tuple[bool, Optional[object]]:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout_sec)
        if not future.done():
            self.get_logger().error(f'{label} service call timed out after {self.service_timeout_sec:.2f} seconds.')
            return False, None
        response = future.result()
        if response is None:
            self.get_logger().error(f'{label} service call failed with no response.')
            return False, None
        return True, response

    def _pause_world(self, pause: bool) -> bool:
        req = ControlWorld.Request()
        req.world_control.pause = pause
        ok, resp = self._call_service(self.control_client, req, 'control_world')
        if not ok:
            return False
        if not resp.success:
            self.get_logger().error(f'Failed to set pause={pause} via /world/{self.world_name}/control.')
            return False
        time.sleep(self.pause_wait_sec)
        return True

    def _step_world(self, steps: int) -> bool:
        step_count = max(1, int(steps))
        req = ControlWorld.Request()
        req.world_control.pause = True
        req.world_control.step = True
        req.world_control.multi_step = step_count
        ok, resp = self._call_service(self.control_client, req, 'control_world_step')
        if not ok:
            return False
        if not resp.success:
            self.get_logger().error(f'Failed to step world by {step_count} steps.')
            return False
        time.sleep(self.pause_wait_sec)
        self.get_logger().info(f'Stepped world by {step_count} steps while paused.')
        return True

    def _unpause_briefly(self, duration_sec: float, reason: str) -> bool:
        unpause_time = max(0.0, float(duration_sec))
        if unpause_time <= 0.0:
            return True
        self.get_logger().info(f'Unpausing for {unpause_time:.2f}s to process {reason}...')
        if not self._pause_world(False):
            return False
        time.sleep(unpause_time)
        if not self._pause_world(True):
            return False
        return True

    def _lookup_tool_transform(self) -> Optional[object]:
        timeout = Duration(seconds=self.service_timeout_sec)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.effective_tool_frame,
                Time(),
                timeout=timeout,
            )
            self.get_logger().info(f'TF lookup succeeded: {self.reference_frame} -> {self.effective_tool_frame}.')
            return transform
        except TransformException as exc:
            # Try the fallback on-demand if the initial choice fails.
            if self.effective_tool_frame != self.fallback_tool_frame and self.tool_frame == 'tool0':
                try:
                    fallback = str(self.fallback_tool_frame)
                    transform = self.tf_buffer.lookup_transform(
                        self.reference_frame,
                        fallback,
                        Time(),
                        timeout=timeout,
                    )
                    self.effective_tool_frame = fallback
                    self.get_logger().warn(
                        f"TF lookup failed for '{self.tool_frame}'; switching tool_frame to '{fallback}'."
                    )
                    return transform
                except TransformException:
                    pass

            self.get_logger().error(
                f'TF lookup failed: {self.reference_frame} -> {self.effective_tool_frame}: {exc}'
            )
            return None

    @staticmethod
    def _quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> Tuple[Tuple[float, float, float], ...]:
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        return (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )

    @classmethod
    def _rotate_vector(cls, vec: Tuple[float, float, float], quat: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        rot = cls._quat_to_rot_matrix(*quat)
        vx, vy, vz = vec
        rx = rot[0][0] * vx + rot[0][1] * vy + rot[0][2] * vz
        ry = rot[1][0] * vx + rot[1][1] * vy + rot[1][2] * vz
        rz = rot[2][0] * vx + rot[2][1] * vy + rot[2][2] * vz
        return rx, ry, rz

    @staticmethod
    def _box_inertia(mass: float, sx: float, sy: float, sz: float) -> Tuple[float, float, float]:
        ixx = (mass / 12.0) * (sy * sy + sz * sz)
        iyy = (mass / 12.0) * (sx * sx + sz * sz)
        izz = (mass / 12.0) * (sx * sx + sy * sy)
        return ixx, iyy, izz

    def _forward_direction_tool_frame(self) -> Tuple[float, float, float]:
        """Best-effort unit direction from tool_frame toward forward_frame, expressed in tool_frame."""
        if not self.use_forward_direction or self.forward_frame == self.effective_tool_frame:
            return (1.0, 0.0, 0.0)

        timeout = Duration(seconds=min(self.service_timeout_sec, 1.0))
        try:
            tf_tool_to_forward = self.tf_buffer.lookup_transform(
                self.effective_tool_frame,
                self.forward_frame,
                Time(),
                timeout=timeout,
            )
            dx = tf_tool_to_forward.transform.translation.x
            dy = tf_tool_to_forward.transform.translation.y
            dz = tf_tool_to_forward.transform.translation.z
            norm = math.sqrt(dx * dx + dy * dy + dz * dz)
            if norm > 1e-6:
                return (dx / norm, dy / norm, dz / norm)
        except TransformException as exc:
            if not self._forward_direction_warned:
                self.get_logger().warn(
                    f"Forward direction lookup failed for {self.effective_tool_frame} -> {self.forward_frame}: {exc}. "
                    "Falling back to +X axis for tool_offset_x."
                )
                self._forward_direction_warned = True

        return (1.0, 0.0, 0.0)

    def _build_payload_sdf(self, mass_kg: float) -> str:
        sx = self.payload_size_x
        sy = self.payload_size_y
        sz = self.payload_size_z
        ixx, iyy, izz = self._box_inertia(mass_kg, sx, sy, sz)

        sdf = f"""
<sdf version="1.9">
  <model name="{self.payload_name}">
    <static>false</static>
    <link name="{self.payload_link_name}">
      <inertial>
        <mass>{mass_kg:.6f}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx>
          <iyy>{iyy:.6f}</iyy>
          <izz>{izz:.6f}</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <collision name="payload_collision">
        <geometry>
          <box>
            <size>{sx:.6f} {sy:.6f} {sz:.6f}</size>
          </box>
        </geometry>
      </collision>
      <visual name="payload_visual">
        <geometry>
          <box>
            <size>{sx:.6f} {sy:.6f} {sz:.6f}</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 1.0 0.0 1.0</ambient>
          <diffuse>1.0 1.0 0.0 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()
        return sdf

    def _compute_spawn_pose(self, transform) -> Pose:
        pose = Pose()
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        forward_dir = self._forward_direction_tool_frame()
        forward_vec = (
            forward_dir[0] * self.tool_offset_x_effective,
            forward_dir[1] * self.tool_offset_x_effective,
            forward_dir[2] * self.tool_offset_x_effective,
        )
        # Keep tool_offset_y/z as explicit lateral offsets in the tool frame.
        offset_tool = (
            forward_vec[0],
            forward_vec[1] + self.tool_offset_y,
            forward_vec[2] + self.tool_offset_z,
        )
        offset_world = self._rotate_vector(offset_tool, (qx, qy, qz, qw))

        pose.position.x = tx + offset_world[0] + self.robot_spawn_offset_x
        pose.position.y = ty + offset_world[1] + self.robot_spawn_offset_y
        pose.position.z = tz + offset_world[2] + self.robot_spawn_offset_z

        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _spawn_payload(self, mass_kg: float, spawn_pose: Pose) -> bool:
        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory()
        req.entity_factory.name = self.payload_name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf = self._build_payload_sdf(mass_kg)
        req.entity_factory.pose = spawn_pose
        req.entity_factory.relative_to = self.reference_frame

        ok, resp = self._call_service(self.spawn_client, req, 'spawn_entity')
        if not ok:
            return False
        if not resp.success:
            self.get_logger().error(f'SpawnEntity reported failure for model {self.payload_name}.')
            return False
        self.get_logger().info(f'Spawned payload model {self.payload_name}.')
        self.payload_present = True
        return True

    def _delete_payload(self) -> bool:
        req = DeleteEntity.Request()
        req.entity.name = self.payload_name
        req.entity.type = req.entity.MODEL

        ok, resp = self._call_service(self.delete_client, req, 'delete_entity')
        if not ok:
            return False
        if not resp.success:
            self.get_logger().error(f'DeleteEntity reported failure for model {self.payload_name}.')
            return False
        self.get_logger().info(f'Deleted payload model {self.payload_name}.')
        self.payload_present = False
        return True

    def _publish_burst(self, publisher, count: int, label: str) -> bool:
        burst_count = max(1, int(count))
        try:
            for _ in range(burst_count):
                publisher.publish(Empty())
                time.sleep(self.burst_interval_sec)
            self.get_logger().info(f'Published {label} trigger burst ({burst_count} msgs).')
            return True
        except Exception as exc:  # pragma: no cover - publisher errors are rare
            self.get_logger().error(f'Failed to publish {label} trigger burst: {exc}')
            return False

    def _publish_attach(self) -> bool:
        return self._publish_burst(self.attach_pub, self.attach_burst_count, f'attach on {self.attach_topic}')

    def _publish_detach(self) -> bool:
        return self._publish_burst(self.detach_pub, self.detach_burst_count, f'detach on {self.detach_topic}')

    def _best_effort_detach(self, reason: str) -> None:
        """Detach and step a few ticks to ensure the detachable joint is released."""
        self.get_logger().info(f'Best-effort detach before {reason}...')
        self._publish_detach()
        self._step_world(self.trigger_step_count)
        self._unpause_briefly(self.detach_unpause_sec, f'detach ({reason})')
        self._step_world(self.trigger_step_count)
        time.sleep(self.post_detach_wait_sec)

    def _marker_template(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.effective_tool_frame
        marker.ns = 'payload'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.frame_locked = True
        marker.action = Marker.ADD

        forward_dir = self._forward_direction_tool_frame()
        marker.pose.position.x = float(forward_dir[0] * self.tool_offset_x_effective)
        marker.pose.position.y = float(forward_dir[1] * self.tool_offset_x_effective + self.tool_offset_y)
        marker.pose.position.z = float(forward_dir[2] * self.tool_offset_x_effective + self.tool_offset_z)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = float(self.payload_size_x)
        marker.scale.y = float(self.payload_size_y)
        marker.scale.z = float(self.payload_size_z)

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.lifetime = Duration(seconds=self.marker_lifetime_sec).to_msg()
        return marker

    def _start_marker(self) -> None:
        self.marker_active = True
        marker = self._marker_template()
        marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(marker)
        self.get_logger().info(f'Published payload marker on {self.marker_topic}.')

        if self.broadcast_rate <= 0.0:
            self.get_logger().warn('broadcast_rate <= 0; marker will not be refreshed periodically.')
            return

        if self.marker_timer is not None:
            return

        period = 1.0 / self.broadcast_rate
        self.marker_timer = self.create_timer(period, self._marker_timer_cb, callback_group=self.callback_group)
        self.get_logger().info(f'Marker timer started at {self.broadcast_rate:.1f} Hz.')

    def _marker_timer_cb(self) -> None:
        if not self.marker_active:
            return
        marker = self._marker_template()
        marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(marker)

    def _stop_marker(self) -> None:
        self.marker_active = False
        if self.marker_timer is not None:
            self.marker_timer.cancel()
            self.marker_timer = None

        # Clear any stale markers first, then follow up with DELETE bursts.
        delete_all = Marker()
        delete_all.header.frame_id = self.effective_tool_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.ns = 'payload'
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        delete_all.lifetime = Duration(seconds=0.0).to_msg()
        self.marker_pub.publish(delete_all)

        burst_count = max(1, int(self.marker_delete_burst_count))
        for _ in range(burst_count):
            marker = self._marker_template()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.action = Marker.DELETE
            marker.lifetime = Duration(seconds=0.0).to_msg()
            self.marker_pub.publish(marker)
            time.sleep(self.marker_delete_interval_sec)
        self.get_logger().info(f'Deleted payload marker on {self.marker_topic} (burst={burst_count}).')

    def _handle_detach_delete(self) -> int:
        self.get_logger().info('Detach requested: pausing Gazebo...')
        if not self._pause_world(True):
            return -3

        detach_ok = self._publish_detach()
        if not detach_ok:
            self._pause_world(False)
            return -5

        if not self._step_world(self.trigger_step_count):
            self._pause_world(False)
            return -3

        if not self._unpause_briefly(self.detach_unpause_sec, 'detach before delete'):
            self._pause_world(False)
            return -3

        if not self._step_world(self.trigger_step_count):
            self._pause_world(False)
            return -3

        self.get_logger().info(f'Waiting {self.post_detach_wait_sec:.2f}s after detach trigger...')
        time.sleep(self.post_detach_wait_sec)

        delete_ok = self._delete_payload()
        if not delete_ok:
            if self.payload_present:
                self._stop_marker()
                self._pause_world(False)
                return -4
            self.get_logger().warn('DeleteEntity failed but payload was not marked present; continuing.')
            self.payload_present = False

        self._stop_marker()

        self.get_logger().info('Resuming Gazebo...')
        if not self._pause_world(False):
            return -3

        self.get_logger().info('Detach + delete completed successfully.')
        return 0

    def _handle_attach_spawn(self, mass_kg: float) -> int:
        if self.detach_before_attach:
            # Perform detach while running, then pause once for TF lookup + spawn.
            self.get_logger().info('Best-effort detach before attach (pre-pause)...')
            detach_ok = self._publish_detach()
            if not detach_ok:
                return -5
            if self.detach_unpause_sec > 0.0:
                self.get_logger().info(
                    f'Waiting {self.detach_unpause_sec:.2f}s while running to process detach before pausing...'
                )
                time.sleep(self.detach_unpause_sec)

        self.get_logger().info(f'Attach requested ({mass_kg:.3f} kg): pausing Gazebo...')
        if not self._pause_world(True):
            return -3

        transform = self._lookup_tool_transform()
        if transform is None:
            self._pause_world(False)
            return -1

        spawn_pose = self._compute_spawn_pose(transform)

        if not self.payload_present:
            spawned = self._spawn_payload(mass_kg, spawn_pose)
            if not spawned:
                # Best-effort cleanup and retry once in case the model already exists.
                self.get_logger().warn('Spawn failed; attempting delete-and-respawn once...')
                self._best_effort_detach('respawn cleanup')
                _ = self._delete_payload()
                spawned = self._spawn_payload(mass_kg, spawn_pose)
                if not spawned:
                    self._pause_world(False)
                    return -2
        else:
            self.get_logger().info('Payload already marked present; skipping spawn and triggering attach.')

        if not self._step_world(self.trigger_step_count):
            self._pause_world(False)
            return -3

        self.get_logger().info(f'Waiting {self.post_spawn_wait_sec:.2f}s for payload readiness...')
        time.sleep(self.post_spawn_wait_sec)

        attach_ok = self._publish_attach()
        if not attach_ok:
            self._pause_world(False)
            return -5

        if not self._step_world(self.trigger_step_count):
            self._pause_world(False)
            return -3

        self._start_marker()

        self.get_logger().info('Resuming Gazebo...')
        if not self._pause_world(False):
            return -3

        self.get_logger().info('Attach + spawn completed successfully.')
        return 0

    def handle_set_mass(self, request: AddTwoInts.Request, response: AddTwoInts.Response) -> AddTwoInts.Response:
        if not self._services_ready():
            response.sum = -10
            return response

        mass_grams = int(request.a)
        mode = int(request.b)
        mass_kg = mass_grams / 1000.0

        self.get_logger().info(f'Received /payload/set_mass request: a={mass_grams} g, b={mode}.')

        detach_requested = mode == 0 or mass_grams == 0
        attach_requested = mode == 1 and mass_grams > 0

        if detach_requested and not attach_requested:
            status = self._handle_detach_delete()
            response.sum = status
            return response

        if not attach_requested:
            self.get_logger().error(f'Unsupported command mode b={mode}. Use 1=attach or 0=detach.')
            response.sum = -6
            return response

        status = self._handle_attach_spawn(mass_kg)
        response.sum = status
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PayloadAttachServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

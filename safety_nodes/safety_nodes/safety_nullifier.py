import sys
from functools import partial
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray


class SafetyNullifier(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            'safety_nullifier',
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('effort_topic', '/forward_effort_controller/commands')
        self.declare_parameter('velocity_topic', '/forward_velocity_controller/commands')
        self.declare_parameter('active_detection_rate', 15.0)
        self.declare_parameter('inactive_detection_rate', 3.0)
        self.declare_parameter('detection_timeout', 0.3)
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('zero_burst_count', 8)

        self.detection_timeout = self.get_parameter('detection_timeout').get_parameter_value().double_value
        self.num_joints = self.get_parameter('num_joints').get_parameter_value().integer_value
        self.zero_burst_count = self.get_parameter('zero_burst_count').get_parameter_value().integer_value
        self.active_detection_rate = self.get_parameter('active_detection_rate').get_parameter_value().double_value
        self.inactive_detection_rate = self.get_parameter('inactive_detection_rate').get_parameter_value().double_value

        effort_topic = self.get_parameter('effort_topic').get_parameter_value().string_value
        velocity_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value

        self.controllers = {}
        self.active_controller = None
        self._init_controller(
            name='effort',
            topic=effort_topic,
        )
        self._init_controller(
            name='velocity',
            topic=velocity_topic,
        )

    def _init_controller(self, name: str, topic: str):
        self.controllers[name] = {
            'topic': topic,
            'last_msg_time': None,
            'non_zero_seen': False,
        }
        self.controllers[name]['publisher'] = self.create_publisher(Float64MultiArray, topic, 10)
        self.controllers[name]['subscription'] = self.create_subscription(
            Float64MultiArray,
            topic,
            partial(self.command_callback, name),
            10,
        )
        self._reset_timer(name, self.inactive_detection_rate)

    def command_callback(self, controller: str, msg: Float64MultiArray):
        state = self.controllers[controller]
        state['last_msg_time'] = self.get_clock().now()
        if any(abs(val) > 1e-9 for val in msg.data):
            state['non_zero_seen'] = True
            self._set_active_controller(controller)

    def timer_callback(self, controller: str):
        state = self.controllers[controller]
        if not state['non_zero_seen'] or state['last_msg_time'] is None:
            return

        now = self.get_clock().now()
        elapsed = (now - state['last_msg_time']).nanoseconds / 1e9
        if elapsed < self.detection_timeout:
            return

        msg = Float64MultiArray()
        msg.data = [0.0] * self.num_joints
        for _ in range(self.zero_burst_count):
            state['publisher'].publish(msg)

        state['non_zero_seen'] = False
        self._set_active_controller(None)

    def _reset_timer(self, controller: str, rate: float):
        state = self.controllers[controller]
        timer = state.get('timer')
        if timer is not None:
            timer.cancel()

        effective_rate = rate if rate > 0.0 else 1.0
        state['timer'] = self.create_timer(
            1.0 / effective_rate,
            partial(self.timer_callback, controller),
        )

    def _set_active_controller(self, controller: Optional[str]):
        if controller is not None and controller not in self.controllers:
            return

        if controller == self.active_controller:
            return

        self.active_controller = controller
        for name in self.controllers:
            rate = self.active_detection_rate if name == controller else self.inactive_detection_rate
            self._reset_timer(name, rate)


def main(args=None):
    raw_args = sys.argv[1:] if args is None else args
    override_tokens = [a for a in raw_args if ':=' in a and not a.startswith('--')]
    ros_args = [a for a in raw_args if a not in override_tokens]

    parameter_overrides = []
    for token in override_tokens:
        name, value_str = token.split(':=', 1)
        value: Parameter = Parameter(name=name, value=value_str)
        if value_str.isdigit():
            value = Parameter(name=name, value=int(value_str))
        else:
            try:
                value_float = float(value_str)
                value = Parameter(name=name, value=value_float)
            except ValueError:
                if value_str.lower() in ('true', 'false'):
                    value = Parameter(name=name, value=value_str.lower() == 'true')
        parameter_overrides.append(value)

        # remove the override token from ros args if present
        if token in ros_args:
            ros_args.remove(token)

    rclpy.init(args=ros_args)
    node = SafetyNullifier(parameter_overrides=parameter_overrides)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

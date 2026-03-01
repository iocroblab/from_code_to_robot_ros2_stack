#!/usr/bin/env python3
import threading
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WINDOW_BG = "#f4f1e8"
PANEL_BG = "#fffdf7"
ACCENT = "#1f5f5b"
TEXT = "#1f2421"


def _movement_token(keysym: str):
    mapping = {
        "w": "w",
        "a": "a",
        "s": "s",
        "d": "d",
        "space": "z_plus",
        "minus": "z_minus",
        "underscore": "z_minus",
        "KP_Subtract": "z_minus",
    }
    return mapping.get(keysym)


def _normalized_keysym(keysym: str) -> str:
    if len(keysym) == 1:
        return keysym.lower()
    return keysym


def _event_token(event) -> str | None:
    keysym = _normalized_keysym(event.keysym)
    if keysym in ("Shift_L", "Shift_R"):
        return "shift"
    if keysym == "Tab":
        return "Tab"
    if keysym in ("m", "M"):
        return "m"
    return _movement_token(keysym)


class TeleopWindow:
    def __init__(self, node: "TutorialTeleop"):
        self.node = node
        self.root = tk.Tk()
        self.root.title("tutorialteleop")
        self.root.configure(bg=WINDOW_BG, padx=18, pady=18)
        self.root.geometry("600x280")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._ignore_close)

        title = tk.Label(
            self.root,
            text="Teleoperation",
            bg=WINDOW_BG,
            fg=TEXT,
            font=("TkDefaultFont", 18, "bold"),
        )
        title.pack(anchor="w")

        subtitle = tk.Label(
            self.root,
            text="Focus this window to drive. Use Ctrl-C here or in the terminal to quit.",
            bg=WINDOW_BG,
            fg=TEXT,
            font=("TkDefaultFont", 10),
            justify="left",
            anchor="w",
            wraplength=540,
        )
        subtitle.pack(anchor="w", pady=(2, 12))

        panel = tk.Frame(self.root, bg=PANEL_BG, highlightbackground=ACCENT, highlightthickness=2)
        panel.pack(fill="both", expand=True)

        self.status_var = tk.StringVar()
        self.mapping_var = tk.StringVar()
        self.focus_var = tk.StringVar(value="Focus: active")

        self.status_label = tk.Label(
            panel,
            textvariable=self.status_var,
            justify="left",
            anchor="w",
            bg=PANEL_BG,
            fg=TEXT,
            font=("TkDefaultFont", 12, "bold"),
        )
        self.status_label.pack(fill="x", padx=16, pady=(16, 10))

        self.mapping_label = tk.Label(
            panel,
            textvariable=self.mapping_var,
            justify="left",
            anchor="w",
            bg=PANEL_BG,
            fg=TEXT,
            font=("TkFixedFont", 12),
        )
        self.mapping_label.pack(fill="x", padx=16)

        self.focus_label = tk.Label(
            panel,
            textvariable=self.focus_var,
            justify="left",
            anchor="w",
            bg=PANEL_BG,
            fg=ACCENT,
            font=("TkDefaultFont", 10, "bold"),
        )
        self.focus_label.pack(fill="x", padx=16, pady=(14, 16))

        self.root.bind("<KeyPress>", self.node.on_key_press)
        self.root.bind("<KeyRelease>", self.node.on_key_release)
        self.root.bind("<Control-c>", self.node.on_ctrl_c)
        self.root.bind("<Control-C>", self.node.on_ctrl_c)
        self.root.bind("<FocusIn>", self._on_focus_in)
        self.root.bind("<FocusOut>", self._on_focus_out)
        self.root.focus_force()
        self.root.after(50, self._focus_window)
        self.refresh()

    def _ignore_close(self):
        return

    def _focus_window(self):
        try:
            self.root.focus_force()
        except tk.TclError:
            pass

    def _on_focus_in(self, _event):
        self.focus_var.set("Focus: active")

    def _on_focus_out(self, _event):
        self.focus_var.set("Focus: inactive")
        self.node.clear_pressed_keys()

    def refresh(self):
        mode = self.node.mode_str()
        frame = self.node.frame_str()
        if mode == "Linear":
            z_pos = "+Z"
            z_neg = "-Z"
            wx = "+X / -X"
            wy = "-Y / +Y"
        else:
            z_pos = "+RotZ"
            z_neg = "-RotZ"
            wx = "+RotX / -RotX"
            wy = "-RotY / +RotY"

        self.status_var.set(f"Mode: {mode}    Frame: {frame}")
        self.mapping_var.set(
            "W / S -> {wx}\n"
            "A / D -> {wy}\n"
            "Space / - (minus) -> {zp} / {zn}\n"
            "Shift doubles speed, Tab toggles mode, M toggles reference frame".format(
                wx=wx,
                wy=wy,
                zp=z_pos,
                zn=z_neg,
            )
        )

    def pump(self):
        self.root.update_idletasks()
        self.root.update()

    def destroy(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass


class TutorialTeleop(Node):
    def __init__(self):
        super().__init__("tutorial_teleop")

        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("topic", "cmd_vel")
        self.declare_parameter("release_debounce_ms", 160)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.release_debounce_ms = int(self.get_parameter("release_debounce_ms").value)
        topic = str(self.get_parameter("topic").value)

        self.cmd_pub = self.create_publisher(Twist, topic, 10)
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mode_pub = self.create_publisher(String, "teleop_mode", mode_qos)

        self.mode_rotation = False
        self.ref_frame_is_ee = False
        self._pressed_tokens = set()
        self._keycode_to_token = {}
        self._lock = threading.Lock()
        self._pending_release = {}
        self._shutdown_requested = False

        self.window = TeleopWindow(self)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)
        self._publish_ref_frame()
        self.get_logger().info(
            'Teleop active. Focus the teleop window to drive. Press Ctrl-C in the window or terminal to quit.'
        )

    def mode_str(self) -> str:
        return "Rotation" if self.mode_rotation else "Linear"

    def frame_str(self) -> str:
        return "EndEffector" if self.ref_frame_is_ee else "Base"

    def _publish_ref_frame(self):
        self.mode_pub.publish(String(data=self.frame_str()))

    def _toggle_mode(self):
        self.mode_rotation = not self.mode_rotation
        self.window.refresh()

    def _toggle_frame(self):
        self.ref_frame_is_ee = not self.ref_frame_is_ee
        self._publish_ref_frame()
        self.window.refresh()

    def on_key_press(self, event):
        keycode = event.keycode
        pending = self._pending_release.pop(keycode, None)
        if pending is not None:
            try:
                self.window.root.after_cancel(pending)
            except tk.TclError:
                pass

        token = _event_token(event)
        if token is None:
            return

        with self._lock:
            already_down = keycode in self._keycode_to_token
            self._keycode_to_token[keycode] = token
            self._pressed_tokens.add(token)

        if token == "Tab" and not already_down:
            self._toggle_mode()
            return

        if token == "m" and not already_down:
            self._toggle_frame()

    def on_key_release(self, event):
        keycode = event.keycode
        pending = self._pending_release.pop(keycode, None)
        if pending is not None:
            try:
                self.window.root.after_cancel(pending)
            except tk.TclError:
                pass

        self._pending_release[keycode] = self.window.root.after(
            self.release_debounce_ms,
            lambda kc=keycode: self._finalize_key_release(kc),
        )

    def _finalize_key_release(self, keycode):
        self._pending_release.pop(keycode, None)
        with self._lock:
            token = self._keycode_to_token.pop(keycode, None)
            if token is None:
                return
            if token not in self._keycode_to_token.values():
                self._pressed_tokens.discard(token)

    def on_ctrl_c(self, _event):
        self._shutdown_requested = True
        return "break"

    def shutdown_requested(self) -> bool:
        return self._shutdown_requested

    def clear_pressed_keys(self):
        with self._lock:
            self._pressed_tokens.clear()
            self._keycode_to_token.clear()
        for after_id in self._pending_release.values():
            try:
                self.window.root.after_cancel(after_id)
            except tk.TclError:
                pass
        self._pending_release.clear()

    def _active_keys(self):
        with self._lock:
            return set(self._pressed_tokens)

    def _tick(self):
        keys = self._active_keys()
        twist = Twist()
        shift = "shift" in keys
        shift_boost = shift and any(token in keys for token in ("w", "a", "s", "d", "z_plus", "z_minus"))
        linear_speed = self.linear_speed * (2.0 if shift_boost else 1.0)
        angular_speed = self.angular_speed * (2.0 if shift_boost else 1.0)

        if not self.mode_rotation:
            twist.linear.x = (+linear_speed if "w" in keys else 0.0) + (-linear_speed if "s" in keys else 0.0)
            twist.linear.y = (-linear_speed if "a" in keys else 0.0) + (+linear_speed if "d" in keys else 0.0)
            twist.linear.z = (+linear_speed if "z_plus" in keys else 0.0) + (-linear_speed if "z_minus" in keys else 0.0)
        else:
            twist.angular.x = (+angular_speed if "w" in keys else 0.0) + (-angular_speed if "s" in keys else 0.0)
            twist.angular.y = (-angular_speed if "a" in keys else 0.0) + (+angular_speed if "d" in keys else 0.0)
            twist.angular.z = (+angular_speed if "z_plus" in keys else 0.0) + (-angular_speed if "z_minus" in keys else 0.0)

        self.cmd_pub.publish(twist)
        self._publish_ref_frame()

    def pump_ui(self):
        self.window.pump()

    def destroy_node(self):
        self.clear_pressed_keys()
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        self.window.destroy()
        return super().destroy_node()


def main():
    rclpy.init()
    node = TutorialTeleop()
    try:
        while rclpy.ok() and not node.shutdown_requested():
            node.pump_ui()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

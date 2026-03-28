#!/usr/bin/env python3
import argparse
import math
import subprocess
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk
import xml.etree.ElementTree as ET

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


MODEL_OPTIONS = ("elfin3", "elfin5", "elfin5_l", "elfin10", "elfin10_l", "elfin15")
JOINT_NAMES = [f"elfin_joint{i}" for i in range(1, 7)]
WORKSPACE_ROOT = Path(__file__).resolve().parent
RELAUNCH_SCRIPT = WORKSPACE_ROOT / "run_elfin_visualize.sh"
DEFAULT_LIMITS_RAD = [
    (-3.14, 3.14),
    (-2.35, 2.35),
    (-2.61, 2.61),
    (-3.14, 3.14),
    (-2.56, 2.56),
    (-3.14, 3.14),
]


def radians_to_degrees(values):
    return [math.degrees(value) for value in values]


def degrees_to_radians(values):
    return [math.radians(value) for value in values]


def load_joint_limits(model):
    xacro_path = (
        Path(__file__).resolve().parent
        / "src/elfin_robot_ros2/elfin_description/urdf"
        / f"{model}.urdf.xacro"
    )
    if not xacro_path.exists():
        return DEFAULT_LIMITS_RAD

    root = ET.parse(xacro_path).getroot()
    limits_by_joint = {}
    for joint in root.findall("joint"):
        joint_name = joint.attrib.get("name")
        if joint_name not in JOINT_NAMES:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        try:
            limits_by_joint[joint_name] = (
                float(limit.attrib["lower"]),
                float(limit.attrib["upper"]),
            )
        except (KeyError, ValueError):
            continue

    return [limits_by_joint.get(joint_name, default) for joint_name, default in zip(JOINT_NAMES, DEFAULT_LIMITS_RAD)]


class ElfinControllerNode(Node):
    def __init__(self, action_name, fallback_topic):
        super().__init__("elfin_controller_panel")
        self._joint_state_lock = threading.Lock()
        self._latest_joint_positions = [0.0] * len(JOINT_NAMES)
        self._latest_joint_map = {}

        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self._joint_state_pub = self.create_publisher(JointState, fallback_topic, 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, action_name)

    def _joint_state_callback(self, msg):
        joint_map = {
            name: position
            for name, position in zip(msg.name, msg.position)
            if name in JOINT_NAMES
        }
        if not joint_map:
            return

        with self._joint_state_lock:
            self._latest_joint_map = joint_map
            for index, joint_name in enumerate(JOINT_NAMES):
                if joint_name in joint_map:
                    self._latest_joint_positions[index] = joint_map[joint_name]

    def get_latest_joint_positions(self):
        with self._joint_state_lock:
            return list(self._latest_joint_positions)

    def publish_joint_states(self, positions_rad):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = list(positions_rad)
        self._joint_state_pub.publish(msg)

    def action_server_ready(self, timeout_sec):
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def send_trajectory_goal(self, positions_rad, duration_sec):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = list(positions_rad)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"Sending trajectory goal: {['%.3f' % value for value in positions_rad]}"
        )
        return self._action_client.send_goal_async(goal)


class ElfinControlPanel:
    def __init__(self, args):
        rclpy.init()
        self.node = ElfinControllerNode(args.action_name, args.fallback_topic)
        self.executor_thread = threading.Thread(target=self._spin, daemon=True)
        self.executor_thread.start()
        self._command_lock = threading.Lock()
        self._commanded_positions_rad = [0.0] * len(JOINT_NAMES)
        self._running = True

        self.duration_sec = args.duration
        self.auto_mode = args.mode
        self.current_model = args.model
        self.publish_rate_hz = args.publish_rate
        self.slider_vars = []
        self.slider_widgets = []
        self.value_labels = []
        self.limit_labels = []
        self.updating_from_code = False

        self.root = tk.Tk()
        self.root.title("Elfin Joint Controller")
        self.root.geometry("880x520")
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self.status_var = tk.StringVar(value="等待控制指令")
        self.model_var = tk.StringVar(value=self.current_model)
        self.duration_var = tk.DoubleVar(value=self.duration_sec)

        self._build_ui()
        self._apply_model_limits(self.current_model, keep_values=False)
        self._refresh_from_joint_states()
        self.publisher_thread = threading.Thread(target=self._publish_joint_state_loop, daemon=True)
        self.publisher_thread.start()

    def _spin(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except ExternalShutdownException:
            pass

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=16)
        main_frame.pack(fill=tk.BOTH, expand=True)

        top_bar = ttk.Frame(main_frame)
        top_bar.pack(fill=tk.X, pady=(0, 12))

        ttk.Label(top_bar, text="机械臂型号").pack(side=tk.LEFT)
        model_combo = ttk.Combobox(
            top_bar, textvariable=self.model_var, values=MODEL_OPTIONS, state="readonly", width=12
        )
        model_combo.pack(side=tk.LEFT, padx=(8, 16))
        model_combo.bind("<<ComboboxSelected>>", self._on_model_changed)

        ttk.Label(top_bar, text="轨迹时间(s)").pack(side=tk.LEFT)
        duration_spin = ttk.Spinbox(
            top_bar,
            from_=0.2,
            to=10.0,
            increment=0.1,
            textvariable=self.duration_var,
            width=8,
        )
        duration_spin.pack(side=tk.LEFT, padx=(8, 16))

        ttk.Label(top_bar, text="控制模式").pack(side=tk.LEFT)
        mode_text = "自动优先动作接口，失败回退到 joint_states"
        ttk.Label(top_bar, text=mode_text).pack(side=tk.LEFT, padx=(8, 0))

        slider_frame = ttk.Frame(main_frame)
        slider_frame.pack(fill=tk.BOTH, expand=True)

        for index, joint_name in enumerate(JOINT_NAMES):
            row = ttk.Frame(slider_frame)
            row.pack(fill=tk.X, pady=6)

            ttk.Label(row, text=f"J{index + 1}", width=4).pack(side=tk.LEFT)
            ttk.Label(row, text=joint_name, width=14).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(
                row,
                orient=tk.HORIZONTAL,
                variable=var,
                from_=-180.0,
                to=180.0,
                command=lambda _value, joint_index=index: self._on_slider_changed(joint_index),
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

            value_label = ttk.Label(row, text="0.0 deg", width=12)
            value_label.pack(side=tk.LEFT, padx=(8, 8))

            limit_label = ttk.Label(row, text="[-180.0, 180.0]", width=20)
            limit_label.pack(side=tk.LEFT)

            self.slider_vars.append(var)
            self.slider_widgets.append(slider)
            self.value_labels.append(value_label)
            self.limit_labels.append(limit_label)

        button_bar = ttk.Frame(main_frame)
        button_bar.pack(fill=tk.X, pady=(12, 12))

        ttk.Button(button_bar, text="发送到控制器", command=self._send_current_pose).pack(side=tk.LEFT)
        ttk.Button(button_bar, text="回零位", command=self._go_home).pack(side=tk.LEFT, padx=8)
        ttk.Button(button_bar, text="读取当前关节", command=self._refresh_from_joint_states).pack(side=tk.LEFT)

        ttk.Label(main_frame, textvariable=self.status_var, anchor=tk.W).pack(fill=tk.X)

    def _on_model_changed(self, _event=None):
        selected_model = self.model_var.get()
        if selected_model == self.current_model:
            return

        self.status_var.set(f"正在切换到型号 {selected_model}，将自动重启可视化...")
        self.root.update_idletasks()
        self._relaunch_with_model(selected_model)

    def _relaunch_with_model(self, model):
        subprocess.Popen(
            ["bash", str(RELAUNCH_SCRIPT), model],
            cwd=str(WORKSPACE_ROOT),
            start_new_session=True,
        )
        self.root.after(100, self._close)

    def _apply_model_limits(self, model, keep_values):
        limits_deg = radians_to_degrees_pair(load_joint_limits(model))
        current_values = self._get_slider_values_deg() if keep_values else [0.0] * len(JOINT_NAMES)

        self.updating_from_code = True
        try:
            for index, ((lower_deg, upper_deg), slider, var, limit_label) in enumerate(
                zip(limits_deg, self.slider_widgets, self.slider_vars, self.limit_labels)
            ):
                slider.configure(from_=lower_deg, to=upper_deg)
                clamped_value = min(max(current_values[index], lower_deg), upper_deg)
                var.set(clamped_value)
                self._update_value_label(index)
                limit_label.configure(text=f"[{lower_deg:.1f}, {upper_deg:.1f}] deg")
        finally:
            self.updating_from_code = False

    def _on_slider_changed(self, joint_index):
        self._update_value_label(joint_index)
        if self.updating_from_code:
            return
        self._sync_commanded_positions_from_sliders()
        self.status_var.set(
            "已修改关节角，点击“发送到控制器”执行，或在没有控制器时回退到可视化模式"
        )

    def _update_value_label(self, joint_index):
        value_deg = self.slider_vars[joint_index].get()
        value_rad = math.radians(value_deg)
        self.value_labels[joint_index].configure(text=f"{value_deg:.1f} deg / {value_rad:.3f} rad")

    def _get_slider_values_deg(self):
        return [var.get() for var in self.slider_vars]

    def _get_slider_values_rad(self):
        return degrees_to_radians(self._get_slider_values_deg())

    def _refresh_from_joint_states(self):
        positions_rad = self.node.get_latest_joint_positions()
        limits_deg = radians_to_degrees_pair(load_joint_limits(self.current_model))
        positions_deg = radians_to_degrees(positions_rad)

        self.updating_from_code = True
        try:
            for index, value_deg in enumerate(positions_deg):
                lower_deg, upper_deg = limits_deg[index]
                self.slider_vars[index].set(min(max(value_deg, lower_deg), upper_deg))
                self._update_value_label(index)
        finally:
            self.updating_from_code = False

        self._set_commanded_positions_rad(positions_rad)
        self.status_var.set("已从 /joint_states 同步当前关节角")

    def _go_home(self):
        self.updating_from_code = True
        try:
            for index, var in enumerate(self.slider_vars):
                var.set(0.0)
                self._update_value_label(index)
        finally:
            self.updating_from_code = False

        self._set_commanded_positions_rad([0.0] * len(JOINT_NAMES))
        self.status_var.set("已回到零位，点击“发送到控制器”执行")

    def _send_current_pose(self):
        self.duration_sec = float(self.duration_var.get())
        positions_rad = self._get_slider_values_rad()
        if self.auto_mode != "auto":
            self.node.publish_joint_states(positions_rad)
            self.status_var.set("已发送到 joint_states 回退话题")
            return

        if self.node.action_server_ready(timeout_sec=0.4):
            future = self.node.send_trajectory_goal(positions_rad, self.duration_sec)
            future.add_done_callback(self._handle_goal_response)
            self.status_var.set("已发送轨迹目标到 FollowJointTrajectory")
            return

        self.node.publish_joint_states(positions_rad)
        self.status_var.set("未发现动作服务器，已回退到 joint_states 可视化模式")

    def _sync_commanded_positions_from_sliders(self):
        self._set_commanded_positions_rad(self._get_slider_values_rad())

    def _set_commanded_positions_rad(self, positions_rad):
        with self._command_lock:
            self._commanded_positions_rad = list(positions_rad)

    def _get_commanded_positions_rad(self):
        with self._command_lock:
            return list(self._commanded_positions_rad)

    def _publish_joint_state_loop(self):
        interval_sec = 1.0 / max(self.publish_rate_hz, 1.0)
        while self._running and rclpy.ok():
            if self.auto_mode == "joint_state_only":
                self.node.publish_joint_states(self._get_commanded_positions_rad())
            threading.Event().wait(interval_sec)

    def _handle_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.root.after(0, lambda: self.status_var.set(f"发送轨迹失败: {exc}"))
            return

        if not goal_handle.accepted:
            self.root.after(0, lambda: self.status_var.set("控制器拒绝了轨迹目标"))
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_goal_result)
        self.root.after(0, lambda: self.status_var.set("轨迹目标已被控制器接受"))

    def _handle_goal_result(self, future):
        try:
            result = future.result().result
            error_code = getattr(result, "error_code", 0)
            error_string = getattr(result, "error_string", "")
        except Exception as exc:
            self.root.after(0, lambda: self.status_var.set(f"读取轨迹结果失败: {exc}"))
            return

        if error_code == 0:
            self.root.after(0, lambda: self.status_var.set("轨迹执行完成"))
            return

        message = f"轨迹执行失败，error_code={error_code}"
        if error_string:
            message = f"{message}, error={error_string}"
        self.root.after(0, lambda: self.status_var.set(message))

    def _close(self):
        self._running = False
        self.root.quit()
        self.root.destroy()
        self.node.destroy_node()
        rclpy.shutdown()
        self.publisher_thread.join(timeout=1.0)
        self.executor_thread.join(timeout=1.0)

    def run(self):
        self.root.mainloop()


def radians_to_degrees_pair(limits_rad):
    return [(math.degrees(lower), math.degrees(upper)) for lower, upper in limits_rad]


def parse_args():
    parser = argparse.ArgumentParser(description="Elfin mechanical arm joint controller")
    parser.add_argument("--model", choices=MODEL_OPTIONS, default="elfin3", help="Elfin robot model")
    parser.add_argument(
        "--action-name",
        default="/elfin_arm_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action name",
    )
    parser.add_argument(
        "--fallback-topic",
        default="/joint_states",
        help="Fallback topic used when trajectory controller is unavailable",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="Trajectory execution time in seconds",
    )
    parser.add_argument(
        "--mode",
        choices=("auto", "joint_state_only"),
        default="auto",
        help="auto tries the trajectory action first, joint_state_only publishes only JointState",
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=30.0,
        help="JointState publish rate in Hz when mode is joint_state_only",
    )
    return parser.parse_args()


def main():
    panel = ElfinControlPanel(parse_args())
    panel.run()


if __name__ == "__main__":
    main()

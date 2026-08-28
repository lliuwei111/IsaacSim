#!/usr/bin/env python3
"""Interactive 2-task pick-and-place for myCobot Pro 450 + Isaac Sim.

One script, two grasp tasks chosen at startup:

  Task "tissue" (纸巾) —— maduo 第三个抓取点 → maduo 第一个放置点:
      pick  = (-3.8,  -360.0, 191.2, -179.6, 0.3, -129.4)
      place = (241.7,  204.6, 194.7, -176.0, 6.8, -82.2)

  Task "water" (矿泉水):
      pick  = (304.1, -300.7, 185.2, -174.1, 0.0, -100.0)
      place = (292.5,  233.8, 167.2,  -86.9, 43.9, -28.1)

At startup the node asks the user to type 纸巾 or 矿泉水.

IMPORTANT: `ros2 launch` does NOT forward keyboard input to the node, so
interactive typing only works with `ros2 run`.  Under launch, pass mode:=

Usage (interactive — typing works):
  ros2 run pro450_isaacsim pick_and_place_menu --ros-args -p ip:=1.95.73.178

Usage (non-interactive):
  ros2 launch pro450_isaacsim pick_and_place_menu.launch.py \
      ip:=1.95.73.178 mode:=water
"""

import math
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import pymycobot
from packaging import version

MIN_REQUIRE_VERSION = "4.0.1"

CURRENT_VERSION = pymycobot.__version__
print(f"current pymycobot library version: {CURRENT_VERSION}")
if version.parse(CURRENT_VERSION) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        f"The version of pymycobot library must be greater than {MIN_REQUIRE_VERSION} "
        f"or higher. Current version: {CURRENT_VERSION}. Please upgrade."
    )
print("pymycobot library version meets the requirements!")
from pymycobot import Pro450Client

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Pro450 joint limits (degrees), from URDF <limit lower="..." upper="...">
JOINT_LIMITS_DEG = [
    (-162.0, 162.0),   # joint1
    (-125.0, 125.0),   # joint2
    (-154.0, 154.0),   # joint3
    (-162.0, 162.0),   # joint4
    (-162.0, 162.0),   # joint5
    (-165.0, 165.0),   # joint6
]

# ---------------------------------------------------------------------------
# Gripper value range (adjust to match your physical hardware)
# Typical Pro450 gripper: 0=fully open, 100=fully closed (or vice-versa)
# Change these if your gripper behaves differently.
GRIPPER_OPEN = 95    # value for fully open
GRIPPER_CLOSE = 5    # value for fully closed (grasp)
GRIPPER_SPEED = 5
# ---------------------------------------------------------------------------

# Gripper drive joint (myGripper F100, NOT a regular arm joint).
# Kept in sync with slider_control.py / follow_display.py:
#   closed = 0 rad, fully open = -1.012 rad.
GRIPPER_DRIVE_JOINT = "joint2_left_joint"
GRIPPER_CLOSED_RAD = 0.0
GRIPPER_OPEN_RAD = -1.012

# Gripper arrival detection for _do_gripper's wait-to-reach loop.
# get_pro_gripper_status(): 0=Moving, 1=Stopped(no clamp), 2=Stopped(clamped).
GRIPPER_REACH_TOLERANCE = 3   # |actual - target| <= 3 counts -> reached (fallback)
GRIPPER_TIMEOUT = 6.0         # seconds to wait before giving up on arrival
GRIPPER_MIN_WAIT = 0.3        # minimum wait before trusting status (avoid stale stop)

# ---------------------------------------------------------------------------
# Task definitions: (x, y, z, roll, pitch, yaw) in mm/degrees
# ---------------------------------------------------------------------------
TASKS = {
    "tissue": {   # 纸巾 —— maduo 第三个抓取点 → maduo 第一个放置点
        "label":       "纸巾",
        "pick":        (-3.8, -360.0, 191.2, -179.6, 0.3, -129.4),
        "place":       (194.5, 357.1, 208.7, -176.6, -2.8, 46.2),
        "robot_speed": 5,
    },
    "water": {    # 矿泉水
        "label":       "矿泉水",
        "pick":        (304.1, -300.7, 185.2, -174.1, 0.0, -100.0),
        "place":       (292.5, 233.8, 167.2, -86.9, 43.9, -28.1),
        "robot_speed": 6,
    },
}


class PickPlaceState(Enum):
    """State machine states for pick-and-place execution."""

    IDLE = "idle"
    MOVING_TO_HOME = "moving_to_home"
    MOVING_TO_PRE_PICK = "moving_to_pre_pick"
    MOVING_TO_PICK = "moving_to_pick"
    CLOSING_GRIPPER = "closing_gripper"
    RETREATING_UP = "retreating_up"
    MOVING_TO_PRE_PLACE = "moving_to_pre_place"
    MOVING_TO_PLACE = "moving_to_place"
    OPENING_GRIPPER = "opening_gripper"
    RETREATING_AFTER_PLACE = "retreating_after_place"
    RETURNING_HOME = "returning_home"
    DONE = "done"
    ERROR = "error"


class PickAndPlaceNode(Node):
    """ROS2 node: interactive 2-task pick-and-place for Pro450 + Isaac Sim."""

    def __init__(self):
        super().__init__("pick_and_place_menu")

        # ---- Robot connection parameters ----
        self.declare_parameter("ip", "192.168.0.232")
        self.declare_parameter("port", 4500)

        # ---- Simulation sync ----
        self.declare_parameter("command_topic", "isaac_joint_commands")
        self.declare_parameter("sync_isaac", True)

        # ---- Task selection: "tissue" / "water" / "" (ask user) ----
        self.declare_parameter("mode", "")

        # ---- Read all parameters ----
        ip = self.get_parameter("ip").value
        port = self.get_parameter("port").value
        self.command_topic = self.get_parameter("command_topic").value
        self.sync_isaac = self.get_parameter("sync_isaac").value
        mode = self.get_parameter("mode").value

        # ---- Select task (interactive prompt if not specified) ----
        if mode not in TASKS:
            mode = self._ask_user()
        task = TASKS[mode]
        self.task_label = task["label"]
        self.pick_pos = list(task["pick"])
        self.place_pos = list(task["place"])

        # ---- Motion parameters ----
        self.declare_parameter("approach_height", 120.0)
        self.declare_parameter("gripper_wait_s", 1.5)
        self.approach_height = float(self.get_parameter("approach_height").value)
        self.gripper_wait_s = float(self.get_parameter("gripper_wait_s").value)
        self.robot_speed = int(task["robot_speed"])

        # ---- Home position (joint angles in degrees) ----
        home = []
        for i in range(1, 7):
            self.declare_parameter(f"home_j{i}", 0.0)
            home.append(float(self.get_parameter(f"home_j{i}").value))
        self.home_angles = home

        # ---- State machine ----
        self._state = PickPlaceState.IDLE
        self._state_sequence = []
        self._state_index = 0
        self._pending_target = None  # for wait-to-arrive logic

        # ---- Connect to real robot ----
        self.get_logger().info(f"Connecting Pro450: ip={ip}, port={port}")
        self.mycobot = Pro450Client(ip, port)
        time.sleep(0.05)
        if self.mycobot.is_power_on() != 1:
            self.get_logger().info("Powering on robot...")
            self.mycobot.power_on()
        time.sleep(0.05)
        if self.mycobot.get_fresh_mode() != 1:
            self.mycobot.set_fresh_mode(1)
        time.sleep(0.05)
        self.mycobot.set_limit_switch(2, 0)

        # ---- Isaac Sim publisher ----
        self.command_pub = None
        if self.sync_isaac:
            self.command_pub = self.create_publisher(JointState, self.command_topic, 10)
            self.get_logger().info(f"Isaac sync enabled on topic: {self.command_topic}")

        # ---- Auto-start timer (1s delay for everything to initialize) ----
        self._start_timer = self.create_timer(1.0, self._auto_start)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Pick-and-Place Node READY  [任务: {self.task_label}]")
        self.get_logger().info(f"  Pick:  {self.pick_pos}")
        self.get_logger().info(f"  Place: {self.place_pos}")
        self.get_logger().info(f"  Approach height: {self.approach_height} mm")
        self.get_logger().info(f"  Robot speed: {self.robot_speed}")
        self.get_logger().info(f"  Home:  {self.home_angles}")
        self.get_logger().info(f"  Sync Isaac: {self.sync_isaac}")
        self.get_logger().info("=" * 60)

    # ------------------------------------------------------------------
    #  Interactive task selection
    # ------------------------------------------------------------------

    def _ask_user(self) -> str:
        """Prompt the user to choose 纸巾 (1) or 矿泉水 (2).

        Accepts ONLY "1" or "2".  Any other input is an error — the robot
        does NOT move, and the user is re-prompted until a valid input is
        given.  Reads stdin in a background thread so the node never hangs.

        NOTE: `ros2 launch` does NOT forward keyboard input to the node
        (stdin is /dev/null under launch).  For interactive typing use:
            ros2 run pro450_isaacsim pick_and_place_menu --ros-args -p ip:=1.95.73.178
        or pass mode:=tissue / mode:=water to the launch file.
        """
        print("=" * 60)
        print("请选择要执行的抓取任务：")
        print("  1. 纸巾   (maduo 第三抓取点 → 第一放置点)")
        print("  2. 矿泉水")
        print("  请输入 1 或 2")
        print("=" * 60, flush=True)

        while True:
            _UNSET = object()
            answer = {"value": _UNSET}

            def _read():
                try:
                    answer["value"] = input().strip()
                except (EOFError, OSError):
                    answer["value"] = None

            t = threading.Thread(target=_read, daemon=True)
            t.start()
            t.join()

            choice = answer["value"]
            if choice is _UNSET:
                # input() returned without a value — should not normally happen
                self.get_logger().warn("未读取到输入，请重新输入")
                continue
            if choice is None:
                # stdin is not available (e.g. launched via ros2 launch)
                self.get_logger().warn(
                    "无法读取终端输入（ros2 launch 不转发键盘输入），"
                    "默认执行 纸巾 任务；如需交互请用 ros2 run"
                )
                return "tissue"
            if choice == "1":
                return "tissue"
            if choice == "2":
                return "water"
            # Invalid input: do NOT start the robot, re-prompt
            self.get_logger().warn(f"错误指令：{choice!r}，机械臂不会运动")
            print("=" * 60)
            print("输入错误，请重新选择：")
            print("  1. 纸巾   (maduo 第三抓取点 → 第一放置点)")
            print("  2. 矿泉水")
            print("  请输入 1 或 2")
            print("=" * 60, flush=True)

    # ------------------------------------------------------------------
    #  Auto-start & State Machine
    # ------------------------------------------------------------------

    def _auto_start(self):
        """One-shot timer: kick off the pick-and-place sequence."""
        self._start_timer.cancel()
        self.get_logger().info("Starting pick-and-place sequence...")
        self._build_sequence()
        self._setup_step_timer()

    def _build_sequence(self):
        """Build the state machine sequence: pick→place→home."""
        self._state_sequence = [
            PickPlaceState.MOVING_TO_HOME,
            PickPlaceState.MOVING_TO_PRE_PICK,
            PickPlaceState.MOVING_TO_PICK,
            PickPlaceState.CLOSING_GRIPPER,
            PickPlaceState.RETREATING_UP,
            PickPlaceState.MOVING_TO_PRE_PLACE,
            PickPlaceState.MOVING_TO_PLACE,
            PickPlaceState.OPENING_GRIPPER,
            PickPlaceState.RETREATING_AFTER_PLACE,
            PickPlaceState.RETURNING_HOME,
            PickPlaceState.DONE,
        ]
        self._state_index = 0

    def _setup_step_timer(self):
        """Create a timer that drives the state machine at ~20 Hz."""
        self._step_timer = self.create_timer(0.05, self._step_callback)

    def _step_callback(self):
        """State machine driver: called at 20 Hz, waits for each step to finish."""
        if self._state_index >= len(self._state_sequence):
            self._step_timer.cancel()
            return

        current_state = self._state_sequence[self._state_index]
        try:
            finished = self._execute_state(current_state)
        except Exception as e:
            self.get_logger().error(f"Error in state {current_state.value}: {e}")
            self._state = PickPlaceState.ERROR
            self._step_timer.cancel()
            return

        if finished:
            self._state_index += 1
            if self._state_index >= len(self._state_sequence):
                self.get_logger().info("=" * 60)
                self.get_logger().info("Pick-and-Place COMPLETE!  抓取放置完成!")
                self.get_logger().info("=" * 60)
                self._state = PickPlaceState.DONE
                self._step_timer.cancel()

    # ------------------------------------------------------------------
    #  State Handlers – each returns True when the step is finished
    # ------------------------------------------------------------------

    def _execute_state(self, state: PickPlaceState) -> bool:
        """Dispatch to the correct handler."""
        self._state = state

        if state == PickPlaceState.MOVING_TO_HOME:
            return self._do_move_joints(self.home_angles, "Moving to HOME position")
        elif state == PickPlaceState.MOVING_TO_PRE_PICK:
            pre_pick = self._make_pre_position(self.pick_pos)
            return self._do_move_ik(pre_pick, "Moving to PRE-PICK (above object)")
        elif state == PickPlaceState.MOVING_TO_PICK:
            return self._do_move_ik(self.pick_pos, "Moving to PICK (grasp object)")
        elif state == PickPlaceState.CLOSING_GRIPPER:
            return self._do_gripper(GRIPPER_CLOSE, "CLOSING gripper")
        elif state == PickPlaceState.RETREATING_UP:
            pre_pick = self._make_pre_position(self.pick_pos)
            return self._do_move_ik(pre_pick, "RETREATING up after grasp")
        elif state == PickPlaceState.MOVING_TO_PRE_PLACE:
            pre_place = self._make_pre_position(self.place_pos)
            return self._do_move_ik(pre_place, "Moving to PRE-PLACE (above target)")
        elif state == PickPlaceState.MOVING_TO_PLACE:
            return self._do_move_ik(self.place_pos, "Moving to PLACE (release object)")
        elif state == PickPlaceState.OPENING_GRIPPER:
            return self._do_gripper(GRIPPER_OPEN, "OPENING gripper")
        elif state == PickPlaceState.RETREATING_AFTER_PLACE:
            pre_place = self._make_pre_position(self.place_pos)
            return self._do_move_ik(pre_place, "RETREATING up after release")
        elif state == PickPlaceState.RETURNING_HOME:
            return self._do_move_joints(self.home_angles, "Returning to HOME")
        elif state == PickPlaceState.DONE:
            return True
        else:
            return True

    # ------------------------------------------------------------------
    #  Motion Primitives
    # ------------------------------------------------------------------

    def _clamp_angles(self, angles_deg: list) -> list:
        """Clamp joint angles to Pro450 hardware limits.  Returns clamped copy."""
        clamped = []
        for i, a in enumerate(angles_deg):
            lo, hi = JOINT_LIMITS_DEG[i]
            ca = max(lo, min(hi, float(a)))
            clamped.append(ca)
        return clamped

    def _make_pre_position(self, base_xyz_rpy: list) -> list:
        """Create an approach position: same xy/rpy, but z += approach_height."""
        pre = list(base_xyz_rpy)
        pre[2] = base_xyz_rpy[2] + self.approach_height
        return pre

    def _do_move_joints(self, target_degrees: list, label: str) -> bool:
        """Move to joint angles, wait for robot to physically reach target."""
        if not hasattr(self, '_pending_target') or self._pending_target is None:
            self.get_logger().info(f"[{label}] target joints(deg): {[round(j,1) for j in target_degrees]}")
            self._send_to_robot(target_degrees)
            self._pending_target = target_degrees
            self._pending_label = label
            self._move_start_time = time.time()
            return False

        # Check arrival
        current = self._get_robot_angles()
        if any(a == -1 for a in current):
            return False

        # Publish CURRENT robot angles to Isaac so simulation speed matches real robot
        self._publish_to_isaac(current)

        max_diff = max(abs(float(a) - float(b)) for a, b in zip(current, self._pending_target))
        elapsed = time.time() - self._move_start_time

        if elapsed > 30.0:
            self.get_logger().warn(f"[{label}] Timeout (30s), diff={max_diff:.1f}°, continuing")
            self._pending_target = None
            return True

        if max_diff < 3.0:
            self.get_logger().info(f"[{label}] Reached! diff={max_diff:.1f}°, time={elapsed:.1f}s")
            self._pending_target = None
            return True

        if int(elapsed) % 3 == 0 and elapsed - getattr(self, '_last_log', -10) > 2.0:
            self._last_log = elapsed
            self.get_logger().info(f"[{label}] Moving... diff={max_diff:.1f}°, {elapsed:.0f}s")
        return False

    def _do_move_ik(self, target_xyz_rpy: list, label: str) -> bool:
        """Compute IK, send command, then delegate to joint waiting logic."""
        if not hasattr(self, '_pending_target') or self._pending_target is None:
            self.get_logger().info(
                f"[{label}] target: x={target_xyz_rpy[0]:.1f} y={target_xyz_rpy[1]:.1f} "
                f"z={target_xyz_rpy[2]:.1f} rx={target_xyz_rpy[3]:.1f} "
                f"ry={target_xyz_rpy[4]:.1f} rz={target_xyz_rpy[5]:.1f}"
            )

            current_angles = self._get_robot_angles()
            if any(a == -1 for a in current_angles):
                return False

            try:
                ik_result = self.mycobot.solve_inv_kinematics(target_xyz_rpy, current_angles)
            except Exception as e:
                self.get_logger().error(f"[{label}] IK failed: {e}")
                self._state = PickPlaceState.ERROR
                return False

            if not isinstance(ik_result, (list, tuple)) or len(ik_result) < 6:
                self.get_logger().error(
                    f"[{label}] IK failed/invalid: {ik_result} "
                    f"(int error code means no IK solution for this target)"
                )
                self._state = PickPlaceState.ERROR
                return False

            target_deg = []
            for j in ik_result[:6]:
                try:
                    target_deg.append(float(j))
                except (ValueError, TypeError):
                    self.get_logger().error(f"[{label}] IK non-numeric value in {ik_result}")
                    self._state = PickPlaceState.ERROR
                    return False

            # Clamp to hardware limits before sending
            raw_deg = list(target_deg)
            target_deg = self._clamp_angles(target_deg)
            if raw_deg != target_deg:
                self.get_logger().warn(
                    f"[{label}] IK out of range, clamped: "
                    f"raw={[round(j,1) for j in raw_deg]} -> "
                    f"safe={[round(j,1) for j in target_deg]}"
                )

            self.get_logger().info(f"[{label}] IK result(deg): {[round(j,1) for j in target_deg]}")
            self._send_to_robot(target_deg)
            self._pending_target = target_deg
            self._pending_label = label
            self._move_start_time = time.time()
            return False

        return self._do_move_joints(self._pending_target, self._pending_label)

    def _do_gripper(self, value: int, label: str) -> bool:
        """Open/close the Pro450 gripper and wait until it physically reaches.

        Driven by the 20 Hz state-machine timer (non-blocking).  Each poll we
        stream the current arm + gripper pose to Isaac, so the simulated
        gripper follows the real gripper with no delay.  Returns True only
        once the gripper has reached (or timed out on) the target.
        """
        # ---- One-time setup (first call only, blocking is fine here) ----
        if not getattr(self, '_gripper_ready', False):
            self._do_gripper_setup(label)
            self._gripper_ready = True
            # After init, re-enter next tick to send the actual command.
            return False

        # ---- Send command once for this target ----
        if getattr(self, '_gripper_target', None) != value:
            self.get_logger().info(f"[{label}] gripper target: {value} ({'OPEN' if value >= 50 else 'CLOSE'})")
            self._send_gripper_command(value, label)
            self._gripper_target = value
            self._gripper_start_time = time.time()
            return False

        # ---- Poll arrival while streaming live pose to Isaac ----
        arm = self._get_robot_angles()
        if not any(a == -1 for a in arm):
            self._publish_to_isaac(arm)   # streams arm + gripper in real time

        elapsed = time.time() - self._gripper_start_time

        if elapsed < GRIPPER_MIN_WAIT:
            return False   # let the motor actually start before judging arrival

        if self._gripper_reached(value):
            self.get_logger().info(f"[{label}] gripper reached (target {value}), {elapsed:.1f}s")
            self._gripper_target = None
            return True

        if elapsed > GRIPPER_TIMEOUT:
            self.get_logger().warn(f"[{label}] gripper timeout {elapsed:.1f}s (target {value})")
            self._gripper_target = None
            return True

        return False

    def _do_gripper_setup(self, label: str):
        """One-time gripper enable + init (modbus mode / baud / recovery)."""
        self.get_logger().info(f"[{label}] Running one-time gripper setup...")

        # 1) Enable the gripper motor
        if hasattr(self.mycobot, "set_pro_gripper_enabled"):
            ret = self.mycobot.set_pro_gripper_enabled(1)
            self.get_logger().info(f"[{label}] set_pro_gripper_enabled(1) -> {ret}")
            time.sleep(0.3)

        # 2) Initialize gripper (handles modbus mode, baud rate, recovery)
        if hasattr(self.mycobot, "set_pro_gripper_init"):
            self.get_logger().info(f"[{label}] Calling set_pro_gripper_init()...")
            ret = self.mycobot.set_pro_gripper_init()
            self.get_logger().info(f"[{label}] set_pro_gripper_init() -> {ret}")
            time.sleep(2.0)  # init can take time

    def _send_gripper_command(self, value: int, label: str):
        """Send a gripper angle command, with open/close fallback wrappers."""
        ret = None
        if hasattr(self.mycobot, "set_pro_gripper_angle"):
            # value 0-100: 0=fully closed, 100=fully open
            ret = self.mycobot.set_pro_gripper_angle(value)
            self.get_logger().info(f"[{label}] set_pro_gripper_angle({value}) -> {ret}")

        # ---- Fallback: open/close wrappers ----
        if ret is None or ret != 1:
            try:
                if value >= 50 and hasattr(self.mycobot, "set_pro_gripper_open"):
                    ret = self.mycobot.set_pro_gripper_open()
                    self.get_logger().info(f"[{label}] set_pro_gripper_open() -> {ret}")
                elif value < 50 and hasattr(self.mycobot, "set_pro_gripper_close"):
                    ret = self.mycobot.set_pro_gripper_close()
                    self.get_logger().info(f"[{label}] set_pro_gripper_close() -> {ret}")
            except Exception as e:
                self.get_logger().warn(f"[{label}] gripper fallback error: {e}")

        if ret == 1:
            self.get_logger().info(f"[{label}] Gripper command SUCCESS")
        else:
            self.get_logger().warn(f"[{label}] Gripper command returned {ret} (1=success). "
                                   "Check hardware connection or gripper power.")

    def _gripper_reached(self, target: int) -> bool:
        """True when the gripper has finished moving toward `target`.

        Prefers the status register (0=Moving, non-zero=Stopped); falls back
        to comparing the reported angle against the target.
        """
        if hasattr(self.mycobot, "get_pro_gripper_status"):
            try:
                st = self.mycobot.get_pro_gripper_status()
                if isinstance(st, int) and st != 0:
                    return True
            except Exception:
                pass

        cur = self._get_gripper_angle()
        if cur is not None and abs(cur - target) <= GRIPPER_REACH_TOLERANCE:
            return True
        return False

    # ------------------------------------------------------------------
    #  Robot & Isaac Communication
    # ------------------------------------------------------------------

    def _send_to_robot(self, angles_deg: list):
        """Send joint angles (degrees) to the physical robot."""
        try:
            self.mycobot.send_angles([float(a) for a in angles_deg], self.robot_speed)
        except Exception as e:
            self.get_logger().error(f"Failed to send angles to robot: {e}")

    def _publish_to_isaac(self, angles_deg: list):
        """Publish joint angles (converted to radians) to Isaac Sim."""
        if self.command_pub is None:
            return

        names = list(JOINT_NAMES)
        positions = [math.radians(float(a)) for a in angles_deg[:6]]

        # Append the F100 gripper drive joint so the simulated gripper follows.
        gripper_cmd = self._get_gripper_angle()
        if gripper_cmd is not None:
            names.append(GRIPPER_DRIVE_JOINT)
            positions.append(self._gripper_cmd_to_rad(gripper_cmd))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        msg.velocity = []
        msg.effort = []
        self.command_pub.publish(msg)

    def _get_gripper_angle(self):
        """Read F100 gripper opening (0=closed .. 100=open), or None if invalid.

        Returns None when the gripper is not initialized (get_pro_gripper_angle
        returns -1) or the value is out of the 0..100 range.
        """
        if not hasattr(self.mycobot, "get_pro_gripper_angle"):
            return None
        try:
            cmd = self.mycobot.get_pro_gripper_angle()
        except Exception as e:
            self.get_logger().debug(f"Failed to read gripper angle: {e}")
            return None
        if isinstance(cmd, (int, float)) and 0 <= cmd <= 100:
            return float(cmd)
        return None

    def _gripper_cmd_to_rad(self, gripper_cmd):
        """Map F100 SDK opening (0=closed .. 100=open) to joint2_left_joint rad.

        Same mapping as slider_control.py / follow_display.py:
        closed = 0 rad, fully open = -1.012 rad.
        """
        ratio = max(0.0, min(1.0, float(gripper_cmd) / 100.0))
        return GRIPPER_CLOSED_RAD + ratio * (GRIPPER_OPEN_RAD - GRIPPER_CLOSED_RAD)

    def _get_robot_angles(self) -> list:
        """Read current joint angles (degrees) from the physical robot."""
        try:
            angles = self.mycobot.get_angles()
            if isinstance(angles, (list, tuple)) and len(angles) >= 6:
                return [float(a) for a in angles[:6]]
            else:
                self.get_logger().debug(f"get_angles() returned non-list: {type(angles).__name__} = {angles}")
        except Exception as e:
            self.get_logger().warn(f"Failed to read robot angles: {e}")
        return [-1.0] * 6


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

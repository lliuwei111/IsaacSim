#!/usr/bin/env python3
"""Synchronized Pick-and-Place for myCobot Pro 450 + Isaac Sim.

This node implements a complete pick-and-place pipeline that simultaneously
controls both the physical myCobot Pro 450 robot and the Isaac Sim simulation:

Architecture:
  Real Robot (Pro450) <── pymycobot ──> pick_and_place node ──> /joint_command ──> Isaac Sim
                                                                    (Articulation Controller)

Workflow:
  1. Load the object's USDA file into the Isaac Sim USD scene (see add_object_to_scene.py)
  2. Configure the Action Graph in Isaac Sim (ROS2 Subscribe JointState → Articulation Controller)
  3. Run this node with matching pick/place coordinates
  4. The robot executes: home → approach pick → grasp → retreat → approach place → release → home
  5. Every joint command is simultaneously published to /joint_command for Isaac Sim sync

Usage:
  ros2 launch pro450_isaacsim pick_and_place_maduo.launch.py ip:=1.95.73.178

All coordinates come from launch file default_values — no JSON needed.
"""

import math
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

# Gripper drive joint (myGripper F100, NOT a regular arm joint).
# Kept in sync with slider_control.py / follow_display_copy.py:
#   closed = 0 rad, fully open = -1.012 rad.
GRIPPER_DRIVE_JOINT = "joint2_left_joint"
GRIPPER_CLOSED_RAD = 0.0
GRIPPER_OPEN_RAD = -1.012

# Pro450 joint limits (degrees), from URDF <limit lower="..." upper="...">
# joint1: ±162°, joint2: ±125°, joint3: ±154°, joint4: ±162°, joint5: ±162°, joint6: ±165°
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

# Gripper arrival detection for _do_gripper's wait-to-reach loop.
# get_pro_gripper_status(): 0=Moving, 1=Stopped(no clamp), 2=Stopped(clamped).
GRIPPER_REACH_TOLERANCE = 3   # |actual - target| <= 3 counts -> reached (fallback)
GRIPPER_TIMEOUT = 6.0         # seconds to wait before giving up on arrival
GRIPPER_MIN_WAIT = 0.3        # minimum wait before trusting status (avoid stale stop)

class PickPlaceState(Enum):
    """State machine states for pick-and-place execution."""

    IDLE = "idle"
    MOVING_TO_HOME = "moving_to_home"
    MOVING_TO_PRE_PICK = "moving_to_pre_pick"     # above the object
    MOVING_TO_PICK = "moving_to_pick"              # down to grasp
    CLOSING_GRIPPER = "closing_gripper"
    RETREATING_UP = "retreating_up"                # lift after grasp
    MOVING_TO_PRE_PLACE = "moving_to_pre_place"    # above the drop location
    MOVING_TO_PLACE = "moving_to_place"            # down to release
    OPENING_GRIPPER = "opening_gripper"
    RETREATING_AFTER_PLACE = "retreating_after_place"
    MOVING_TO_PRE_PICK2 = "moving_to_pre_pick2"
    MOVING_TO_PICK2 = "moving_to_pick2"
    CLOSING_GRIPPER2 = "closing_gripper2"
    RETREATING_UP2 = "retreating_up2"
    MOVING_TO_PRE_PLACE2 = "moving_to_pre_place2"
    MOVING_TO_PLACE2 = "moving_to_place2"
    OPENING_GRIPPER2 = "opening_gripper2"
    RETREATING_AFTER_PLACE2 = "retreating_after_place2"
    MOVING_TO_PRE_PICK3 = "moving_to_pre_pick3"
    MOVING_TO_PICK3 = "moving_to_pick3"
    CLOSING_GRIPPER3 = "closing_gripper3"
    RETREATING_UP3 = "retreating_up3"
    MOVING_TO_PRE_PLACE3 = "moving_to_pre_place3"
    MOVING_TO_PLACE3 = "moving_to_place3"
    OPENING_GRIPPER3 = "opening_gripper3"
    RETREATING_AFTER_PLACE3 = "retreating_after_place3"
    RETURNING_HOME = "returning_home"
    DONE = "done"
    ERROR = "error"

class PickAndPlaceNode(Node):
    """ROS2 node: synchronized pick-and-place for Pro450 + Isaac Sim.

    Dual-output architecture:
      1. Real robot:  pymycobot.send_angles() over TCP to Pro450
      2. Simulation:  publishes sensor_msgs/JointState to /joint_command
         (consumed by Isaac Sim's ROS2 Subscribe Joint State + Articulation Controller)
    """

    def __init__(self):
        super().__init__("pick_and_place")

        # ---- Robot connection parameters ----
        self.declare_parameter("ip", "192.168.0.232")
        self.declare_parameter("port", 4500)

        # ---- Simulation sync ----
        self.declare_parameter("command_topic", "isaac_joint_commands")
        self.declare_parameter("sync_isaac", True)

        # ---- Read all parameters ----
        ip = self.get_parameter("ip").value
        port = self.get_parameter("port").value
        self.command_topic = self.get_parameter("command_topic").value
        self.sync_isaac = self.get_parameter("sync_isaac").value

        # ---- Pick / Place coordinates (from launch file, no JSON needed) ----
        _POS_DEFAULTS = {
            "pick1":  (147.66, -228.4, 188.2, -175.2, 0.0, -176.8),
            "place1": (241.7, 204.6, 194.7, -176.0, 6.8, -82.2),
            "pick2":  (321.2, -119.2, 194.5, -176.9, 0.0, -140.3),
            "place2": (239.2, 201.8, 255.4, 179.6, 5.4, -82.9),
            "pick3":  (-3.8, -360.0, 191.2, -179.6, 0.3, -129.4),
            "place3": (239.2, 201.8, 320.4, 179.6, 5.4, -82.9),  # place2 + z 65mm
        }
        for prefix, (x, y, z, roll, pitch, yaw) in _POS_DEFAULTS.items():
            self.declare_parameter(prefix + "_x", x)
            self.declare_parameter(prefix + "_y", y)
            self.declare_parameter(prefix + "_z", z)
            self.declare_parameter(prefix + "_roll", roll)
            self.declare_parameter(prefix + "_pitch", pitch)
            self.declare_parameter(prefix + "_yaw", yaw)

        def _get_pos(prefix):
            return [
                float(self.get_parameter(prefix + "_x").value),
                float(self.get_parameter(prefix + "_y").value),
                float(self.get_parameter(prefix + "_z").value),
                float(self.get_parameter(prefix + "_roll").value),
                float(self.get_parameter(prefix + "_pitch").value),
                float(self.get_parameter(prefix + "_yaw").value),
            ]

        self.pick_pos = _get_pos("pick1")
        self.place_pos = _get_pos("place1")
        self.pick2_pos = _get_pos("pick2")
        self.place2_pos = _get_pos("place2")
        self.pick3_pos = _get_pos("pick3")
        self.place3_pos = _get_pos("place3")

        # ---- Motion parameters ----
        self.declare_parameter("approach_height", 120.0)
        self.declare_parameter("robot_speed", 5)
        self.declare_parameter("gripper_wait_s", 1.5)
        self.approach_height = float(self.get_parameter("approach_height").value)
        self.robot_speed = int(self.get_parameter("robot_speed").value)
        self.gripper_wait_s = float(self.get_parameter("gripper_wait_s").value)

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
        self.get_logger().info("Pick-and-Place Node READY")
        self.get_logger().info(f"  Pick:  {self.pick_pos}")
        self.get_logger().info(f"  Place: {self.place_pos}")
        self.get_logger().info(f"  Approach height: {self.approach_height} mm")
        self.get_logger().info(f"  Home:  {self.home_angles}")
        self.get_logger().info(f"  Sync Isaac: {self.sync_isaac}")
        self.get_logger().info("=" * 60)

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
        """Build the state machine sequence: pick1→place1→pick2→place2→home."""
        self._state_sequence = [
            PickPlaceState.MOVING_TO_HOME,
            # ---- Round 1 ----
            PickPlaceState.MOVING_TO_PRE_PICK,
            PickPlaceState.MOVING_TO_PICK,
            PickPlaceState.CLOSING_GRIPPER,
            PickPlaceState.RETREATING_UP,
            PickPlaceState.MOVING_TO_PRE_PLACE,
            PickPlaceState.MOVING_TO_PLACE,
            PickPlaceState.OPENING_GRIPPER,
            PickPlaceState.RETREATING_AFTER_PLACE,
            # ---- Round 2 (no home in between) ----
            PickPlaceState.MOVING_TO_PRE_PICK2,
            PickPlaceState.MOVING_TO_PICK2,
            PickPlaceState.CLOSING_GRIPPER2,
            PickPlaceState.RETREATING_UP2,
            PickPlaceState.MOVING_TO_PRE_PLACE2,
            PickPlaceState.MOVING_TO_PLACE2,
            PickPlaceState.OPENING_GRIPPER2,
            PickPlaceState.RETREATING_AFTER_PLACE2,
            # ---- Round 3 (no home in between) ----
            PickPlaceState.MOVING_TO_PRE_PICK3,
            PickPlaceState.MOVING_TO_PICK3,
            PickPlaceState.CLOSING_GRIPPER3,
            PickPlaceState.RETREATING_UP3,
            PickPlaceState.MOVING_TO_PRE_PLACE3,
            PickPlaceState.MOVING_TO_PLACE3,
            PickPlaceState.OPENING_GRIPPER3,
            PickPlaceState.RETREATING_AFTER_PLACE3,
            # ---- Finally home ----
            PickPlaceState.RETURNING_HOME,
            PickPlaceState.DONE,
        ]
        self._state_index = 0

    def _setup_step_timer(self):
        """Create a timer that drives the state machine at ~20 Hz.

        Running at 20 Hz ensures smooth Isaac Sim sync by publishing
        intermediate joint positions that match the real robot's actual speed.
        """
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
        # ---- Round 2 ----
        elif state == PickPlaceState.MOVING_TO_PRE_PICK2:
            pre_pick2 = self._make_pre_position(self.pick2_pos)
            return self._do_move_ik(pre_pick2, "Moving to PRE-PICK2 (above object)")
        elif state == PickPlaceState.MOVING_TO_PICK2:
            return self._do_move_ik(self.pick2_pos, "Moving to PICK2 (grasp object)")
        elif state == PickPlaceState.CLOSING_GRIPPER2:
            return self._do_gripper(GRIPPER_CLOSE, "CLOSING gripper (round 2)")
        elif state == PickPlaceState.RETREATING_UP2:
            pre_pick2 = self._make_pre_position(self.pick2_pos)
            return self._do_move_ik(pre_pick2, "RETREATING up after grasp (round 2)")
        elif state == PickPlaceState.MOVING_TO_PRE_PLACE2:
            pre_place2 = self._make_pre_position(self.place2_pos)
            return self._do_move_ik(pre_place2, "Moving to PRE-PLACE2 (above target)")
        elif state == PickPlaceState.MOVING_TO_PLACE2:
            return self._do_move_ik(self.place2_pos, "Moving to PLACE2 (release object)")
        elif state == PickPlaceState.OPENING_GRIPPER2:
            return self._do_gripper(GRIPPER_OPEN, "OPENING gripper (round 2)")
        elif state == PickPlaceState.RETREATING_AFTER_PLACE2:
            pre_place2 = self._make_pre_position(self.place2_pos)
            return self._do_move_ik(pre_place2, "RETREATING up after release (round 2)")
        # ---- Round 3 ----
        elif state == PickPlaceState.MOVING_TO_PRE_PICK3:
            pre_pick3 = self._make_pre_position(self.pick3_pos)
            return self._do_move_ik(pre_pick3, "Moving to PRE-PICK3 (above object)")
        elif state == PickPlaceState.MOVING_TO_PICK3:
            return self._do_move_ik(self.pick3_pos, "Moving to PICK3 (grasp object)")
        elif state == PickPlaceState.CLOSING_GRIPPER3:
            return self._do_gripper(GRIPPER_CLOSE, "CLOSING gripper (round 3)")
        elif state == PickPlaceState.RETREATING_UP3:
            pre_pick3 = self._make_pre_position(self.pick3_pos)
            return self._do_move_ik(pre_pick3, "RETREATING up after grasp (round 3)")
        elif state == PickPlaceState.MOVING_TO_PRE_PLACE3:
            pre_place3 = self._make_pre_position(self.place3_pos)
            return self._do_move_ik(pre_place3, "Moving to PRE-PLACE3 (above target)")
        elif state == PickPlaceState.MOVING_TO_PLACE3:
            return self._do_move_ik(self.place3_pos, "Moving to PLACE3 (release object)")
        elif state == PickPlaceState.OPENING_GRIPPER3:
            return self._do_gripper(GRIPPER_OPEN, "OPENING gripper (round 3)")
        elif state == PickPlaceState.RETREATING_AFTER_PLACE3:
            pre_place3 = self._make_pre_position(self.place3_pos)
            return self._do_move_ik(pre_place3, "RETREATING up after release (round 3)")
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
        """Move to joint angles, wait for robot to physically reach target.

        First call: send command to robot only (NOT to Isaac).
        Subsequent calls: poll current angles, publish CURRENT angles to Isaac
        so the simulation matches the real robot's actual speed, and return
        True when within 3° tolerance.
        """
        if not hasattr(self, '_pending_target') or self._pending_target is None:
            self.get_logger().info(f"[{label}] target joints(deg): {[round(j,1) for j in target_degrees]}")
            self._send_to_robot(target_degrees)
            # DO NOT publish target to Isaac here — instead we publish the
            # robot's actual current angles on each poll cycle below, so the
            # simulation moves at the same speed as the real robot.
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

            if not ik_result or len(ik_result) < 6:
                self.get_logger().error(f"[{label}] IK invalid: {ik_result}")
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
            # DO NOT publish target to Isaac here — _do_move_joints will
            # publish the robot's actual current angles on each poll cycle.
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

        Same mapping as slider_control.py / follow_display_copy.py:
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
                # get_angles() may return an int (error code) instead of list
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

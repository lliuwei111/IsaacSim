import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '4.0.6'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import Pro450Client


ARM_JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)
GRIPPER_DRIVE_JOINT = "joint2_left_joint"
# URDF/USD: closed at 0 rad, fully open at -1.012 rad (~-58 deg).
GRIPPER_CLOSED_RAD = 0.0
GRIPPER_OPEN_RAD = -1.012
GRIPPER_MIN_INTERVAL_SEC = 0.3


class Slider_Subscriber(Node):
    """ROS2 node that mirrors Isaac joint states onto MyCobot Pro450 + myGripper F100."""

    def __init__(self):
        """Initialize the subscriber node and connect to MyCobotPro450."""
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            "isaac_joint_states",
            self.listener_callback,
            60
        )

        self.declare_parameter('ip', '192.168.0.232')
        self.declare_parameter('port', 4500)
        self.declare_parameter('arm_speed', 100)
        self.declare_parameter('gripper_id', 14)
        self.declare_parameter('gripper_speed', 90)

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        self.arm_speed = self.get_parameter("arm_speed").get_parameter_value().integer_value
        self.gripper_id = self.get_parameter("gripper_id").get_parameter_value().integer_value
        gripper_speed = self.get_parameter("gripper_speed").get_parameter_value().integer_value

        self.get_logger().info(
            "ip:%s, port:%d, arm_speed:%d, gripper_id:%d"
            % (ip, port, self.arm_speed, self.gripper_id)
        )
        self.mycobot_450 = Pro450Client(ip, port)
        time.sleep(0.05)
        if self.mycobot_450.is_power_on !=1:
            self.mycobot_450.power_on()
        time.sleep(0.05)
        if self.mycobot_450.get_fresh_mode() != 1:
            self.mycobot_450.set_fresh_mode(1)
        time.sleep(0.05)
        self.mycobot_450.set_limit_switch(2, 0)
        time.sleep(0.05)
        self.mycobot_450.set_pro_gripper_enabled(1, self.gripper_id)
        time.sleep(0.05)
        self.mycobot_450.set_pro_gripper_speed(gripper_speed, self.gripper_id)

        self.last_time = 0.0
        self.last_angles = None
        self.last_gripper_cmd = None
        self.last_gripper_time = 0.0

    def _joint_map(self, msg: JointState):
        return dict(zip(msg.name, msg.position))

    def _arm_angles_deg(self, joints):
        missing = [name for name in ARM_JOINT_NAMES if name not in joints]
        if missing:
            self.get_logger().error("missing arm joints: {}".format(missing))
            return None
        return [round(math.degrees(joints[name]), 1) for name in ARM_JOINT_NAMES]

    def _gripper_angle_cmd(self, joints):
        """Map joint2_left_joint radians to F100 SDK angle 0(close)~100(open)."""
        if GRIPPER_DRIVE_JOINT not in joints:
            return None
        span = GRIPPER_CLOSED_RAD - GRIPPER_OPEN_RAD
        ratio = (GRIPPER_CLOSED_RAD - joints[GRIPPER_DRIVE_JOINT]) / span
        return int(round(max(0.0, min(1.0, ratio)) * 100))

    def listener_callback(self, msg: JointState):
        """Send arm joint1-6 and F100 drive joint to the real robot."""
        now = time.time()
        if now - self.last_time < 0.05:
            return
        self.last_time = now

        joints = self._joint_map(msg)
        arm_angles = self._arm_angles_deg(joints)
        if arm_angles is None:
            return

        arm_changed = self.last_angles is None or arm_angles != self.last_angles
        if arm_changed:
            self.last_angles = arm_angles
            self.get_logger().debug("joint_angles: {}".format(arm_angles))
            # Position stream: speed is catch-up cap (1-100), _async avoids waiting in-position.
            self.mycobot_450.send_angles(arm_angles, self.arm_speed, _async=True)

        gripper_cmd = self._gripper_angle_cmd(joints)
        if gripper_cmd is None:
            return
        gripper_changed = (
            self.last_gripper_cmd is None
            or abs(gripper_cmd - self.last_gripper_cmd) >= 2
        )
        if not gripper_changed:
            return
        if now - self.last_gripper_time < GRIPPER_MIN_INTERVAL_SEC:
            return

        self.last_gripper_cmd = gripper_cmd
        self.last_gripper_time = now
        self.get_logger().info("gripper_angle: {}".format(gripper_cmd))
        self.mycobot_450.set_pro_gripper_angle(gripper_cmd, self.gripper_id)


def main(args=None):
    """Main entry point for the Slider_Subscriber node.

    Args:
        args (list, optional): Command-line arguments for ROS2. Defaults to None.

    Returns:
        None
    """
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()

    rclpy.spin(slider_subscriber)

    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

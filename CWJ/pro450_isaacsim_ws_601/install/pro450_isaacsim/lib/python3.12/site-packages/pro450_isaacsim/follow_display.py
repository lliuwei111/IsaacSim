import math
import time
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
        'Current version is {}. Please upgrade the library version.'.format(
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
# Keep in sync with slider_control.py: 0 rad = closed, -1.012 rad = open.
GRIPPER_CLOSED_RAD = 0.0
GRIPPER_OPEN_RAD = -1.012


class Talker(Node):
    """Drag-teach the real Pro450 and stream pose into Isaac Sim."""

    def __init__(self):
        """Initialize the Talker node and connect to MyCobotPro450."""
        super().__init__("follow_display")
        self.declare_parameter('ip', '192.168.0.232')
        self.declare_parameter('port', 4500)
        self.declare_parameter('sync_isaac', True)
        self.declare_parameter('command_topic', 'isaac_joint_commands')
        self.declare_parameter('gripper_id', 14)
        self.declare_parameter('publish_rate', 30.0)

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        self.sync_isaac = self.get_parameter("sync_isaac").get_parameter_value().bool_value
        self.command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        self.gripper_id = self.get_parameter("gripper_id").get_parameter_value().integer_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.get_logger().info(
            "ip:%s, port:%d, topic:%s, gripper_id:%d"
            % (ip, port, self.command_topic, self.gripper_id)
        )
        self.mycobot_450 = Pro450Client(ip, port)
        if self.mycobot_450.is_power_on != 1:
            self.mycobot_450.power_on()
        time.sleep(0.05)
        self.mycobot_450.set_free_move_mode(1)
        time.sleep(0.05)

        self.command_pub = None
        if self.sync_isaac:
            self.command_pub = self.create_publisher(JointState, self.command_topic, 10)

        period = 1.0 / max(publish_rate, 1.0)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "Please press the button at the end of the machine to drag the joint.\n"
            "请按下机器末端按钮进行关节拖拽运动"
        )

    def _gripper_cmd_to_rad(self, gripper_cmd):
        """Map F100 SDK angle 0(close)~100(open) to joint2_left_joint radians."""
        ratio = max(0.0, min(1.0, float(gripper_cmd) / 100.0))
        return GRIPPER_CLOSED_RAD + ratio * (GRIPPER_OPEN_RAD - GRIPPER_CLOSED_RAD)

    def _on_timer(self):
        if self.command_pub is None:
            return

        angles = self.mycobot_450.get_angles()
        if not (isinstance(angles, list) and len(angles) >= 6):
            self.get_logger().warn("Failed to get valid angles: {}".format(angles))
            return

        names = list(ARM_JOINT_NAMES)
        positions = [math.radians(value) for value in angles[:6]]

        gripper_cmd = self.mycobot_450.get_pro_gripper_angle(self.gripper_id)
        if isinstance(gripper_cmd, (int, float)) and 0 <= gripper_cmd <= 100:
            names.append(GRIPPER_DRIVE_JOINT)
            positions.append(self._gripper_cmd_to_rad(gripper_cmd))

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = names
        command.position = positions
        self.command_pub.publish(command)


def main(args=None):
    """Main function to run the Talker node.

    Args:
        args (list, optional): Command-line arguments for ROS2. Defaults to None.
    """
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

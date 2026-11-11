"""Forward /joint_states (e.g. RViz sliders) to the real JetCobot (myCobot 280)."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot

# Joint names in jetcobot.urdf, in the order send_angles() expects (J1..J6).
JOINT_NAMES = ['1_Joint', '2_Joint', '3_Joint', '4_Joint', '5_Joint', '6_Joint']


class JointBridge(Node):
    """Send joint angles from /joint_states to the robot over serial."""

    def __init__(self):
        super().__init__('joint_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('speed', 25)          # 1-100
        self.declare_parameter('deadband_deg', 0.5)  # skip sends smaller than this
        # Per-joint calibration: robot_deg = sign * urdf_deg + offset
        self.declare_parameter('signs', [1.0] * 6)
        self.declare_parameter('offsets_deg', [0.0] * 6)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.speed = int(self.get_parameter('speed').value)
        self.deadband = float(self.get_parameter('deadband_deg').value)
        self.signs = list(self.get_parameter('signs').value)
        self.offsets = list(self.get_parameter('offsets_deg').value)
        if len(self.signs) != 6 or len(self.offsets) != 6:
            raise ValueError('signs and offsets_deg must have 6 values')

        self.mc = MyCobot(port, baud)
        # Fresh mode: always go to the newest target instead of queueing them.
        if self.mc.get_fresh_mode() != 1:
            self.mc.set_fresh_mode(1)

        self.last_sent = None
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.get_logger().info(
            f'Connected to {port} @ {baud}, speed {self.speed}. Waiting for joint_states.')

    def on_joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in JOINT_NAMES):
            return  # message from another robot / incomplete

        angles = [
            round(s * math.degrees(positions[name]) + o, 2)
            for name, s, o in zip(JOINT_NAMES, self.signs, self.offsets)
        ]

        if self.last_sent is not None and all(
                abs(a - b) < self.deadband for a, b in zip(angles, self.last_sent)):
            return

        self.mc.send_angles(angles, self.speed)
        self.last_sent = angles
        self.get_logger().info(f'Sent {angles}')


def main(args=None):
    rclpy.init(args=args)
    node = JointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""
Mock inference publisher — simulates what joint_base_node publishes when a
real ML policy is running.

Publishes YamCmd to /YAM_INFER_L and /YAM_INFER_R at 50 Hz.
Joint positions are a slowly-sweeping sine wave centered near 0.3 rad,
so they are visually distinct from mock_master_pub output.
"""

import math
import rclpy
from rclpy.node import Node
from yam_arm_msg.msg import YamCmd


class MockInferencePub(Node):
    def __init__(self):
        super().__init__('mock_inference_pub')
        self._pub_l = self.create_publisher(YamCmd, '/YAM_INFER_L', 1)
        self._pub_r = self.create_publisher(YamCmd, '/YAM_INFER_R', 1)
        self._t = 0.0
        self.create_timer(0.02, self._timer_cb)  # 50 Hz
        self.get_logger().info(
            '[mock_inference_pub] publishing YamCmd to /YAM_INFER_L and /YAM_INFER_R'
        )

    def _timer_cb(self):
        self._t += 0.02
        # Slow sine sweep, amplitude 0.2 rad, centered at +0.3 rad
        val = 0.3 + 0.2 * math.sin(self._t)

        for pub, side in [(self._pub_l, 'L'), (self._pub_r, 'R')]:
            cmd = YamCmd()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.mode = 5
            # Each joint gets a slightly offset sine so we can tell them apart
            cmd.joint_pos = [val + i * 0.01 for i in range(6)]
            cmd.gripper = val
            pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MockInferencePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

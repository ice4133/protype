"""
Mock master arm publisher — simulates what master_l / master_r YamController
nodes publish when the operator is hand-guiding the master arm.

Publishes YamStatus to /master_l_status and /master_r_status at 50 Hz.
Joint positions are a slowly-sweeping sine wave centered near -0.3 rad,
so they are visually distinct from mock_inference_pub output.
"""

import math
import rclpy
from rclpy.node import Node
from yam_arm_msg.msg import YamStatus
from rclpy.qos import qos_profile_sensor_data


class MockMasterPub(Node):
    def __init__(self):
        super().__init__('mock_master_pub')
        self._pub_l = self.create_publisher(YamStatus, '/master_l_status', qos_profile_sensor_data)
        self._pub_r = self.create_publisher(YamStatus, '/master_r_status', qos_profile_sensor_data)
        self._t = 0.0
        self.create_timer(0.02, self._timer_cb)  # 50 Hz
        self.get_logger().info(
            '[mock_master_pub] publishing YamStatus to /master_l_status and /master_r_status'
        )

    def _timer_cb(self):
        self._t += 0.02
        # Slow sine sweep, amplitude 0.2 rad, centered at -0.3 rad
        # Clearly different from inference publisher values
        val = -0.3 + 0.2 * math.sin(self._t)

        for pub in [self._pub_l, self._pub_r]:
            status = YamStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            # joint_pos has 7 elements: [j1..j6, gripper]
            status.joint_pos = [val + i * 0.01 for i in range(7)]
            status.joint_vel = [0.0] * 7
            status.joint_cur = [0.0] * 7
            pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = MockMasterPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

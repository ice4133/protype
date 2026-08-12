import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import zmq
import pickle
import json

hostname = "192.168.0.4"
remotename = "192.168.0.5"
# hostname = "127.0.0.1"
# remotename = "127.0.0.1"
cam_left_port = 6003
cam_right_port = 6004

class Processor(Node):
    def __init__(self):
        super().__init__('inference_camera_hub')
        self.pub_rate = 50
        self.right_cv_image = None
        self.left_cv_image = None
        self.top_cv_image = None
        self.socket_cam_right, self.cam_right_context = self.create_zmq_connection(socket_type=zmq.PUB,
                                                                                   port=cam_left_port,
                                                                                   bind=True,
                                                                                   hwm=2)

        self.socket_cam_left, self.cam_left_context = self.create_zmq_connection(socket_type=zmq.PUB,
                                                                                 port=cam_right_port,
                                                                                 bind=True,
                                                                                 hwm=2)

        self.socket_cam_top, self.cam_top_context = self.create_zmq_connection(socket_type=zmq.PUB,
                                                                               port=6005,
                                                                               bind=True,
                                                                               hwm=2)

        self.bridge = CvBridge()
        self.create_subscription(Image, '/cam_high/color/image_rect_raw', self.cam_top_callback, 1)
        self.create_subscription(Image, '/cam_left_wrist/color/image_rect_raw', self.cam_left_callback, 1)
        self.create_subscription(Image, '/cam_right_wrist/color/image_rect_raw', self.cam_right_callback, 1)


    def create_zmq_connection(self, socket_type, port, bind, hwm=2):
        # Create new context if none provided
        context = zmq.Context()

        socket = context.socket(socket_type)

        # Set socket options based on socket type
        if socket_type == zmq.PUB:
            socket.setsockopt(zmq.SNDHWM, hwm)

            address = f"tcp://{hostname}:{port}"
            socket.bind(address)

        elif socket_type == zmq.SUB:
            address = f"tcp://{remotename}:{port}"
            socket.connect(address)

            socket.setsockopt(zmq.RCVTIMEO, 10000)
            socket.setsockopt(zmq.RCVHWM, hwm)
            socket.setsockopt(zmq.SUBSCRIBE, b'')  # Subscribe to all messages

        return socket, context


    def cam_right_callback(self, msg):
        # print(msg)
        self.right_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def cam_left_callback(self, msg):
        # print(msg)
        self.left_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def cam_top_callback(self, msg):
        # print(msg)
        self.top_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def serialize_send(self, image, pub_socket):

        metadata = {
            "shape": image.shape,
            "dtype": image.dtype.str
        }
        data = image.tobytes()

        pub_socket.send_multipart([
            json.dumps(metadata).encode('utf-8'),
            data
        ])

    def process_callback(self):
        self.serialize_send(self.right_cv_image, self.socket_cam_right)
        self.get_logger().info("!!!right send!!!")

        self.serialize_send(self.left_cv_image, self.socket_cam_left)
        self.get_logger().info("!!!left send!!!")

        self.serialize_send(self.top_cv_image, self.socket_cam_top)
        self.get_logger().info("!!!top send!!!")


def main(args=None):
    rclpy.init(args=args)
    processor = Processor()
    time.sleep(3)
    processor.create_timer(1.0 / processor.pub_rate, processor.process_callback)

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

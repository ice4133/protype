import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from arx5_arm_msg.msg import RobotCmd, RobotStatus
from yam_arm_msg.msg import YamCmd, YamStatus
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import numpy as np
import zmq
import pickle
import threading
from rclpy.qos import qos_profile_sensor_data

hostname = "192.168.0.4"
remotename = "192.168.0.5"
# hostname = "127.0.0.1"
# remotename = "127.0.0.1"

arm_status_port = 6001
base_status_port = 6002
arm_cmd_port = 6006
base_cmd_port = 6007

class Processor(Node):
    def __init__(self):
        super().__init__('joint_base_node')

        # Declare the 'arm_type' parameter
        self.declare_parameter('arm_type', 'yam')
        arm_type = self.get_parameter('arm_type').get_parameter_value().string_value
        self.get_logger().info(f"Starting node with arm type: {arm_type}")

        # Set message types and topic names based on the arm type
        if arm_type == 'arx':
            self.CmdMsg = RobotCmd
            self.StatusMsg = RobotStatus
            self.left_status_topic = '/arm_l_status'
            self.right_status_topic = '/arm_r_status'
            self.left_cmd_topic = '/ARX_INFER_L'
            self.right_cmd_topic = '/ARX_INFER_R'
        elif arm_type == 'yam':
            self.CmdMsg = YamCmd
            self.StatusMsg = YamStatus
            self.left_status_topic = '/arm_l_status'
            self.right_status_topic = '/arm_r_status'
            self.left_cmd_topic = '/YAM_VR_L'
            self.right_cmd_topic = '/YAM_VR_R'
        else:
            self.get_logger().fatal(f"Invalid arm_type: {arm_type}. Must be 'yam'.")
            return

        # ZMQ connections for publishing status
        self.socket_arm_pub, _ = self.create_zmq_connection(socket_type=zmq.PUB, port=arm_status_port, hwm=2)
        self.socket_base_pub, _ = self.create_zmq_connection(socket_type=zmq.PUB, port=base_status_port, hwm=2)
        
        # ZMQ connections for subscribing to commands
        self.socket_arm_sub, _ = self.create_zmq_connection(socket_type=zmq.SUB, port=arm_cmd_port, hwm=2)
        self.socket_base_sub, _ = self.create_zmq_connection(socket_type=zmq.SUB, port=base_cmd_port, hwm=2)

        self.current_left_status = None
        self.current_right_status = None
        self.current_base_status = None
        self.pub_rate = 50

        # ROS 2 Subscribers for robot status
        self.create_subscription(self.StatusMsg, self.left_status_topic, self.left_status_callback, qos_profile_sensor_data)
        self.create_subscription(self.StatusMsg, self.right_status_topic, self.right_status_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.base_status_callback, qos_profile_sensor_data)

        # ROS 2 Publishers for robot commands
        self.right_cmd_pub = self.create_publisher(self.CmdMsg, self.right_cmd_topic, 1)
        self.left_cmd_pub = self.create_publisher(self.CmdMsg, self.left_cmd_topic, 1)
        self.base_cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)

        # State variable for takeover
        self.is_taken_over = False

        # Create service servers
        self.takeover_srv = self.create_service(Trigger, '/arm_takeover', self.arm_takeover_callback)
        self.release_srv = self.create_service(Trigger, '/arm_release', self.arm_release_callback)
        self.get_logger().info('✅ ROS2 service servers for /arm_takeover and /arm_release created.')

    def create_zmq_connection(self, socket_type, port, hwm=2):
        context = zmq.Context()
        socket = context.socket(socket_type)

        if socket_type == zmq.PUB:
            # PUB sockets use bind(), which is more reliable and doesn't usually need a reconnect loop.
            # It waits for clients to connect.
            socket.setsockopt(zmq.SNDHWM, hwm)
            address = f"tcp://{hostname}:{port}"
            try:
                socket.bind(address)
                self.get_logger().info(f"PUB socket bound to {address}")
            except zmq.error.ZMQError as e:
                self.get_logger().error(f"Failed to bind PUB socket: {e}")
                raise
            return socket, context

        elif socket_type == zmq.SUB:
            address = f"tcp://{remotename}:{port}"
            socket.connect(address)

            socket.setsockopt(zmq.RCVTIMEO, 10000)
            socket.setsockopt(zmq.RCVHWM, hwm)
            socket.setsockopt(zmq.SUBSCRIBE, b'')  # Subscribe to all messages

            return socket, context

    def left_status_callback(self, msg):
        self.current_left_status = msg.joint_pos

    def right_status_callback(self, msg):
        self.current_right_status = msg.joint_pos

    def base_status_callback(self, msg):
        twist = msg.twist.twist
        self.current_base_status = np.array([twist.angular.x,
                                             twist.angular.y,
                                             twist.angular.z,
                                             twist.linear.x,
                                             twist.linear.y,
                                             twist.linear.z,
                                             ])
    
    def arm_takeover_callback(self, request, response):
        self.is_taken_over = True
        self.get_logger().info('✅ Arm takeover service called. Robot command publishing suspended.')
        response.success = True
        response.message = "Arm takeover successful. Commands will no longer be published."
        return response

    def arm_release_callback(self, request, response):
        self.is_taken_over = False
        self.get_logger().info('✅ Arm release service called. Robot command publishing resumed.')
        response.success = True
        response.message = "Arm released. Command publishing resumed."
        return response

    def joint_msg(self, cur_joint):
        msg = self.CmdMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = 5
        msg.gripper = cur_joint[-1]
        msg.joint_pos = cur_joint[:-1]
        return msg

    def send_left_joint(self, joints):
        left_msg = self.joint_msg(joints[0:7])
        self.left_cmd_pub.publish(left_msg)

    def send_right_joint(self, joints):
        right_msg = self.joint_msg(joints[7:14])
        self.right_cmd_pub.publish(right_msg)

    def base_msg(self, cur_base):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        (angular_x, angular_y, angular_z, linear_x, linear_y, linear_z) = cur_base
        msg.twist.angular.x = angular_x
        msg.twist.angular.y = angular_y
        msg.twist.angular.z = angular_z
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.linear.z = linear_z
        return msg

    def send_base(self, base):
        base_msg = self.base_msg(base)
        self.base_cmd_pub.publish(base_msg)

    def process_callback(self):
        if self.current_left_status is not None and self.current_right_status is not None:
            concat_arm = np.concatenate((self.current_left_status, self.current_right_status))
            self.socket_arm_pub.send(pickle.dumps(concat_arm))
        
        if self.current_base_status is not None:
            self.socket_base_pub.send(pickle.dumps(self.current_base_status))

        # Only publish commands if the arm is NOT taken over
        if not self.is_taken_over:
            try:
                arm_message = self.socket_arm_sub.recv(flags=zmq.NOBLOCK)
                joints = pickle.loads(arm_message)
                self.send_left_joint(joints)
                self.send_right_joint(joints)
            except zmq.Again:
                pass

            try:
                base_message = self.socket_base_sub.recv(flags=zmq.NOBLOCK)
                base = pickle.loads(base_message)
                self.send_base(base)
            except zmq.Again:
                pass

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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from quest2_button_msg.msg import Quest2Button
from arx5_arm_msg.msg import RobotCmd, RobotStatus

import numpy as np
from tf_transformations import quaternion_from_matrix, quaternion_multiply, quaternion_from_euler, euler_from_quaternion, quaternion_matrix, euler_from_matrix
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
import copy

import time


class PoseProcessor(Node):
    def __init__(self):
        super().__init__('arx_cmd_pub')

        # Subscribers
        self.right_trigger = None
        self.left_trigger = None
        self.create_subscription(Quest2Button, '/vr_button', self.button_callback, 1)
        self.create_subscription(PoseStamped, '/vr_pose_left', self.left_pose_callback, 1)
        self.create_subscription(PoseStamped, '/vr_pose_right', self.right_pose_callback, 1)
        self.create_subscription(PoseStamped, '/vr_pose_head', self.head_pose_callback, 1)
        self.create_subscription(RobotStatus, '/arm_l_status', self.left_status_callback, 1)
        self.create_subscription(RobotStatus, '/arm_r_status', self.right_status_callback, 1)

        # Add TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.left_cmd_pub = self.create_publisher(RobotCmd, '/ARX_VR_L', 1)
        self.right_cmd_pub = self.create_publisher(RobotCmd, '/ARX_VR_R', 1)
        self.base_cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.height_cmd_pub = self.create_publisher(Joy, '/vr_height_speed', 1)

        # State variables
        self.status = "standby"
        self.last_button_a = False
        self.stored_left_pose = None
        self.stored_head_pose = None
        self.stored_right_pose = None
        self.current_left_pose = None
        self.current_right_pose = None
        self.current_head_pose = None
        self.left_joint = None
        self.right_joint = None

        # base control param
        self.speed = 0.3
        self.turn = 0.6

        self.pub_rate = 50

        # Homing control
        self.left_home = np.array([0.0] * 6)
        self.right_home = np.array([0.0] * 6)
        self.ramp_step = np.deg2rad(90) / self.pub_rate  # °/s in rad/step

        # Processing timer (50Hz)
        self.process_timer = self.create_timer(1.0/self.pub_rate, self.process_callback)

        self.initialize_transformations()

    def initialize_transformations(self):
        """Initialize static transformation matrices"""
        # Create rotation matrices
        # theta_x = np.deg2rad(-150)  # quest2
        theta_x = np.deg2rad(-90)  # quest3s
        theta_z = np.deg2rad(90)

        # X-axis rotation
        self.Rx = np.eye(4)
        self.Rx[1:3, 1:3] = np.array([
            [np.cos(theta_x), -np.sin(theta_x)],
            [np.sin(theta_x), np.cos(theta_x)]
        ])

        # Z-axis rotation
        self.Rz = np.eye(4)
        self.Rz[0:2, 0:2] = np.array([
            [np.cos(theta_z), -np.sin(theta_z)],
            [np.sin(theta_z), np.cos(theta_z)]
        ])

        # Combined transformation
        self.robot2handle = np.linalg.inv(self.Rx @ self.Rz)

        self.robot_init = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def pose_to_matrix(self, pose):
        """
        Convert PoseStamped to 4x4 transformation matrix
        Returns: numpy array of shape (4, 4)
        """
        # Extract components
        p = pose.pose.position
        q = pose.pose.orientation

        # Convert quaternion to rotation matrix
        rotation_matrix = quaternion_matrix([q.x, q.y, q.z, q.w])

        # Add translation components
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
        transformation_matrix[:3, 3] = [p.x, p.y, p.z]

        return transformation_matrix

    def process_pose(self, stored_pose, current_pose):
        stored_T = self.pose_to_matrix(stored_pose)
        current_T = self.pose_to_matrix(current_pose)

        self.handle2world = np.linalg.inv(stored_T)
        trans_handle = self.handle2world @ current_T
        trans_robot = self.robot2handle @ trans_handle @ np.linalg.inv(self.robot2handle) @ self.robot_init

        msg = RobotCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        [msg.end_pos[0], msg.end_pos[1], msg.end_pos[2]] = trans_robot[:3, 3]   # x y z
        [msg.end_pos[3], msg.end_pos[4], msg.end_pos[5]] = euler_from_matrix(trans_robot[:3, :3], axes='sxyz')  # r p y

        msg.mode = 4 # end_pose mode

        return msg

    def publish_tf(self, pose_msg, child_frame):
        transform = TransformStamped()

        # Populate header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = child_frame

        # Copy position
        transform.transform.translation.x = pose_msg.pose.position.x
        transform.transform.translation.y = pose_msg.pose.position.y
        transform.transform.translation.z = pose_msg.pose.position.z

        # Copy orientation
        transform.transform.rotation = pose_msg.pose.orientation

        # Send transformation
        self.tf_broadcaster.sendTransform(transform)

    def process_base(self, msg):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = -msg.left_joystick[1] * self.speed
        twist_msg.twist.linear.y = -msg.left_joystick[0] * self.speed
        twist_msg.twist.angular.z = -msg.right_joystick[0] * self.turn

        return twist_msg
    
    def process_height(self, msg):
        height_msg = Joy()
        height_msg.header.stamp = self.get_clock().now().to_msg()
        height_msg.axes = [0.]
        height_msg.axes[0] = -msg.right_joystick[1]
        return height_msg

    def init_joint(self):
        left_joint = copy.deepcopy(self.left_joint.joint_pos[:6])
        right_joint = copy.deepcopy(self.right_joint.joint_pos[:6])

        step = self.ramp_step

        def home_msg(cur_joint):
            msg = RobotCmd()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.mode = 5    # joint control
            msg.gripper = 5.0   # use this to control gripper
            msg.joint_pos = cur_joint   # be careful, the RobotCmd only control 6 joint position, no gripper

            return msg

        def ramp(goal, current, ramp_k):
            delta = goal - current
            # Element-wise check: set to goal if within ramp_k, otherwise step towards goal
            return np.where(
                np.abs(delta) <= ramp_k,  # Condition per joint
                goal,  # Set to goal if condition met
                current + np.sign(delta) * ramp_k  # Step towards goal otherwise
            )

        print(f"Left joint: {left_joint}, Right joint: {right_joint}")

        if np.allclose(left_joint, self.left_home, atol=step*2.0) and np.allclose(right_joint, self.right_home, atol=step*2.0):
            # close the gripper
            msg_left = home_msg(self.left_home)
            msg_left.gripper = 0.0
            msg_right = home_msg(self.right_home)
            msg_right.gripper = 0.0

            self.left_cmd_pub.publish(msg_left)
            self.right_cmd_pub.publish(msg_right)
            return True
        else:
            left_joint = ramp(self.left_home, left_joint, step)
            right_joint = ramp(self.right_home, right_joint, step)

            msg_left = home_msg(left_joint)
            msg_right = home_msg(right_joint)

            self.left_cmd_pub.publish(msg_left)
            self.right_cmd_pub.publish(msg_right)
            return False


    def button_callback(self, msg):
        if msg.rg:
            height_speed= self.process_height(msg)
            self.height_cmd_pub.publish(height_speed)
        
        if msg.lg:
            twist_msg = self.process_base(msg)
            self.base_cmd_pub.publish(twist_msg)
        else:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.base_cmd_pub.publish(twist_msg)

        if msg.x:
            srv_name = 'start_collect'
            cli = self.create_client(Trigger, srv_name)

            if not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Service unavailable')
                return

            request = Trigger.Request()
            future = cli.call_async(request)

            time.sleep(0.2)

        if msg.y:
            srv_name = 'stop_collect'
            cli = self.create_client(Trigger, srv_name)
            if not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Service unavailable')
                return

            request = Trigger.Request()
            future = cli.call_async(request)

            time.sleep(0.2)

        # Detect button A press (rising edge)
        if msg.a and not self.last_button_a:

            if self.status == "standby":
                # Store current poses when activated
                if self.current_left_pose and self.current_right_pose:
                    self.stored_left_pose = self.current_left_pose
                    self.stored_right_pose = self.current_right_pose
                    self.stored_head_pose = self.current_head_pose
                    self.get_logger().info("Started processing with stored poses")
                    self.status = "teleop"
                else:
                    self.get_logger().warn("No poses available to store!")

            elif self.status == "teleop":
                self.status = "init"

        if self.status == "init":
            self.get_logger().info("reset robot...")
            flag = self.init_joint()
            if flag is True:
                self.status = 'standby'

        self.last_button_a = msg.a
        self.left_trigger = msg.left_trigger
        self.right_trigger = msg.right_trigger

    def left_status_callback(self, msg):
        self.left_joint = msg

    def right_status_callback(self, msg):
        self.right_joint = msg

    def left_pose_callback(self, msg):
        self.current_left_pose = msg

    def right_pose_callback(self, msg):
        self.current_right_pose = msg

    def head_pose_callback(self, msg):
        self.current_head_pose = msg

    def arx_msg_to_pose(self, msg):
        tf_msg = PoseStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()  # ROS2 time
        tf_msg.header.frame_id = "map"
        tf_msg.pose.position.x = msg.end_pos[0]
        tf_msg.pose.position.y = msg.end_pos[1]
        tf_msg.pose.position.z = msg.end_pos[2]

        x, y, z, w = quaternion_from_euler(msg.end_pos[3], msg.end_pos[4], msg.end_pos[5])

        tf_msg.pose.orientation.x = x
        tf_msg.pose.orientation.y = y
        tf_msg.pose.orientation.z = z
        tf_msg.pose.orientation.w = w

        return tf_msg

    def process_grip(self, grip):
        return 5 * (1 - grip)

    def process_callback(self):
        if self.status == "teleop":
            # Process poses (example: offset calculation)
            left_xyzrpy = self.process_pose(self.stored_left_pose, self.current_left_pose)
            right_xyzrpy = self.process_pose(self.stored_right_pose, self.current_right_pose)
            head_xyzrpy = self.process_pose(self.stored_head_pose, self.current_head_pose)

            left_xyzrpy.gripper = self.process_grip(self.left_trigger)
            right_xyzrpy.gripper = self.process_grip(self.right_trigger)

            # Publish processed poses
            self.left_cmd_pub.publish(left_xyzrpy)
            left_pose = self.arx_msg_to_pose(left_xyzrpy)
            self.publish_tf(left_pose, 'arx_left_pose')

            self.right_cmd_pub.publish(right_xyzrpy)
            right_pose = self.arx_msg_to_pose(right_xyzrpy)
            self.publish_tf(right_pose, 'arx_right_pose')

            head_pose = self.arx_msg_to_pose(head_xyzrpy)
            self.publish_tf(head_pose, 'arx_head_pose')




def main(args=None):
    rclpy.init(args=args)
    processor = PoseProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

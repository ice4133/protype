import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from quest2_button_msg.msg import Quest2Button
from yam_arm_msg.msg import YamCmd, YamStatus
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import copy
import time
from yam_cmd_pub.arm_processor import ArmProcessor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from enum import Enum, auto

class State(Enum):
    TELEOP = auto()
    PAUSED = auto()
    HOMING = auto()

class PoseProcessor(Node):
    def __init__(self):
        super().__init__('yam_vr_teleop_node')
        
        self.declare_parameter('use_left_arm', False)
        self.declare_parameter('use_right_arm', False)
        self.declare_parameter('use_take_over', False)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('turn', 0.6)
        self.declare_parameter('pub_rate', 50.0)
        
        self.use_left_arm = self.get_parameter('use_left_arm').get_parameter_value().bool_value
        self.use_right_arm = self.get_parameter('use_right_arm').get_parameter_value().bool_value
        self.use_take_over = self.get_parameter('use_take_over').get_parameter_value().bool_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.turn = self.get_parameter('turn').get_parameter_value().double_value
        self.pub_rate = self.get_parameter('pub_rate').get_parameter_value().double_value

        self.get_logger().info(f"use_take_over: {self.use_take_over}")

        self.status = State.PAUSED
        self.last_button_a = False
        self.last_button_b = False
        self.arm_processors = {}

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.base_cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.height_cmd_pub = self.create_publisher(Joy, '/vr_height_speed', 1)
        
        self.create_subscription(Quest2Button, '/vr_button', self.button_callback, 1)
        
        if self.use_left_arm:
            self.left_cmd_pub = self.create_publisher(YamCmd, '/YAM_VR_L', 1)
            self.arm_processors['left'] = ArmProcessor('left', self)
            self.create_subscription(PoseStamped, '/vr_pose_left', self.left_pose_callback, qos_profile)
            self.create_subscription(YamStatus, '/arm_l_status', self.left_status_callback, qos_profile)
        
        if self.use_right_arm:
            self.right_cmd_pub = self.create_publisher(YamCmd, '/YAM_VR_R', 1)
            self.arm_processors['right'] = ArmProcessor('right', self)
            self.create_subscription(PoseStamped, '/vr_pose_right', self.right_pose_callback, qos_profile)
            self.create_subscription(YamStatus, '/arm_r_status', self.right_status_callback, qos_profile)
        
        self.create_timer(1.0/self.pub_rate, self.process_callback)

    def left_pose_callback(self, msg): self.arm_processors['left'].current_vr_pose = msg
    def right_pose_callback(self, msg): self.arm_processors['right'].current_vr_pose = msg
    def left_status_callback(self, msg): self.arm_processors['left'].joint_status = msg
    def right_status_callback(self, msg): self.arm_processors['right'].joint_status = msg

    def button_callback(self, msg):
        self.handle_base_control(msg)
        self.handle_recording_services(msg)
        if self.use_take_over:
            self.handle_arm_services(msg)
        else:
            self.handle_state_change(msg)
        self.handle_gripper_control(msg)

        self.last_button_a = msg.a
        self.last_button_b = msg.b

    def handle_base_control(self, msg):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        if msg.lg:
            twist_msg.twist.linear.x = -msg.left_joystick[1] * self.speed
            twist_msg.twist.linear.y = -msg.left_joystick[0] * self.speed
            twist_msg.twist.angular.z = -msg.right_joystick[0] * self.turn
        self.base_cmd_pub.publish(twist_msg)
        height_msg = Joy()
        height_msg.header.stamp = self.get_clock().now().to_msg()
        if msg.rg:
            height_msg.axes = [-msg.right_joystick[1]]
        self.height_cmd_pub.publish(height_msg)

    def call_service(self, service_name):
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Service '{service_name}' not available.")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda future: self.get_logger().info(f"Service call '{service_name}' successful."))

    def handle_recording_services(self, msg):
        if msg.x: self.call_service('start_collect')
        if msg.y: self.call_service('stop_collect')

    def handle_arm_services(self, msg):
        if msg.a and not self.last_button_a:
            if self.status == State.TELEOP:
                self.get_logger().info("Arm is released. Transitioning to paused state.")
                self.call_service('/arm_release')
                self.status = State.PAUSED
            elif self.status == State.PAUSED:
                if all(ap.calculate_initial_vr_se3_norm() for ap in self.arm_processors.values()):
                    self.get_logger().info("Attempting arm takeover. Transitioning to 'teleop' state.")
                    self.call_service('/arm_takeover')
                    self.status = State.TELEOP
                else:
                    self.get_logger().warn("Missing data! Cannot start teleop. Stay in paused state.")
        
        if msg.b and not self.last_button_b and self.status != State.HOMING:
            self.status = State.HOMING
            self.get_logger().info("Initiating homing sequence.")

    def handle_state_change(self, msg):
        if msg.a and not self.last_button_a:
            if self.status == State.TELEOP:
                self.status = State.PAUSED
                self.get_logger().info("Teleoperation paused.")
            elif self.status == State.PAUSED:
                if all(ap.calculate_initial_vr_se3_norm() for ap in self.arm_processors.values()):
                    self.status = State.TELEOP
                    self.get_logger().info("Teleoperation resumed with re-calibration.")
                else:
                    self.get_logger().warn("Missing data! Cannot resume teleop. Stay in paused state.")

        if msg.b and not self.last_button_b and self.status != State.HOMING:
            self.status = State.HOMING
            self.get_logger().info("Initiating homing sequence.")
    
    def handle_gripper_control(self, msg):
        if 'left' in self.arm_processors: self.arm_processors['left'].gripper_trigger = msg.left_trigger
        if 'right' in self.arm_processors: self.arm_processors['right'].gripper_trigger = msg.right_trigger

    def process_callback(self):
        if self.status == State.TELEOP:
            for arm_side, ap in self.arm_processors.items():
                cmd_msg = ap.process_pose(ap.current_vr_pose)
                if cmd_msg:
                    cmd_msg.gripper = 5 * (1 - ap.gripper_trigger)
                    if arm_side == 'left': self.left_cmd_pub.publish(cmd_msg)
                    else: self.right_cmd_pub.publish(cmd_msg)
                    ap.publish_tf(cmd_msg.end_pose, f'eef_{arm_side}')
        elif self.status == State.PAUSED:
            pass
        elif self.status == State.HOMING:
            all_homed = True
            for arm_side, ap in self.arm_processors.items():
                cmd_msg = ap.process_homing()
                if cmd_msg:
                    if arm_side == 'left': self.left_cmd_pub.publish(cmd_msg)
                    else: self.right_cmd_pub.publish(cmd_msg)
                    if cmd_msg.mode == 5 and cmd_msg.gripper == 0.0:
                        pass
                    else:
                        all_homed = False
            if all_homed:
                self.get_logger().info("All active arms have been homed. Transitioning to paused.")
                self.status = State.PAUSED

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

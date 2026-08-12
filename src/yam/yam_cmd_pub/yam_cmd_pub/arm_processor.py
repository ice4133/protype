import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from quest2_button_msg.msg import Quest2Button
from yam_arm_msg.msg import YamCmd, YamStatus
import numpy as np
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
import os
import copy
import time

# Constants
GRIPPER_OPEN_VALUE = 5.0
GRIPPER_CLOSED_VALUE = 0.0
MODE_IK = 4
MODE_JOINT = 5
EEF_FRAME_NAME = "link_6"

class ArmProcessor:
    def __init__(self, arm_side, parent_node):
        self.arm_side = arm_side
        self.parent_node = parent_node
        self.vr_SE3_norm = None
        self.current_vr_pose = None
        self.joint_status = None
        self.gripper_trigger = 0.0
        self.home_joint = np.array([0.0] * 6)
        self.ramp_step = np.deg2rad(360) / 50.0

        self.model, self.data, self.base_SE3_norm, self.norm_SE3_handle = self.initialize_pinocchio()
        self.tf_broadcaster = TransformBroadcaster(self.parent_node)

    def initialize_pinocchio(self):
        try:
            package_share_directory = get_package_share_directory('yam_description')
            urdf_filename = os.path.join(package_share_directory, 'urdf', 'yam.urdf')
            model = pin.buildModelFromUrdf(urdf_filename)
            data = pin.Data(model)
            if not model.existFrame(EEF_FRAME_NAME):
                raise ValueError(f"Frame '{EEF_FRAME_NAME}' not found in URDF model.")
            end_effector_id = model.getFrameId(EEF_FRAME_NAME)
            q_init = pin.neutral(model)
            pin.forwardKinematics(model, data, q_init)
            pin.updateFramePlacement(model, data, end_effector_id)
            eef_SE3_init = data.oMf[end_effector_id]
            norm_SE3_base = pin.SE3(np.eye(3), eef_SE3_init.translation)
            base_SE3_norm = norm_SE3_base.inverse()
            theta_x, theta_z = np.deg2rad(-90), np.deg2rad(90)
            Rx = np.array([[1, 0, 0], [0, np.cos(theta_x), -np.sin(theta_x)], [0, np.sin(theta_x), np.cos(theta_x)]])
            Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0], [np.sin(theta_z), np.cos(theta_z), 0], [0, 0, 1]])
            norm_SE3_handle = pin.SE3(Rotation.from_matrix(Rx @ Rz).as_matrix(), np.zeros(3))
            self.parent_node.get_logger().info("✅ Pinocchio model loaded successfully.")
            return model, data, base_SE3_norm, norm_SE3_handle
        except Exception as e:
            self.parent_node.get_logger().error(f"Failed to initialize Pinocchio: {e}")
            raise e

    def pose_msg_to_SE3(self, pose):
        p = pose.pose.position
        q = pose.pose.orientation
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        return pin.SE3(r.as_matrix(), np.array([p.x, p.y, p.z]))

    def process_pose(self, vr_msg):
        if self.vr_SE3_norm is None or vr_msg is None: return None
        handle_SE3_vr = self.pose_msg_to_SE3(vr_msg)
        eef_SE3_norm = self.vr_SE3_norm * handle_SE3_vr * self.norm_SE3_handle
        rotation = eef_SE3_norm.rotation
        translation = eef_SE3_norm.translation
        cmd_msg = YamCmd()
        cmd_msg.header.stamp = self.parent_node.get_clock().now().to_msg()
        cmd_msg.end_pose[:3] = translation
        r = Rotation.from_matrix(rotation)
        trans_quat = r.as_quat()
        # Reorder from (x, y, z, w) to (w, x, y, z) for the message
        cmd_msg.end_pose[3] = trans_quat[3]  # w
        cmd_msg.end_pose[4] = trans_quat[0]  # x
        cmd_msg.end_pose[5] = trans_quat[1]  # y
        cmd_msg.end_pose[6] = trans_quat[2]  # z
        cmd_msg.mode = MODE_IK
        return cmd_msg

    def publish_tf(self, pose_data, child_frame):
        transform = TransformStamped()
        transform.header.stamp = self.parent_node.get_clock().now().to_msg()
        transform.header.frame_id = 'norm_frame'
        transform.child_frame_id = child_frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = pose_data[0:3]
        w, x, y, z = pose_data[3:7]
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = x, y, z, w
        self.tf_broadcaster.sendTransform(transform)

    def process_homing(self):
        if self.joint_status is None:
            self.parent_node.get_logger().warn(f"{self.arm_side.capitalize()} arm status not available for homing.")
            return False
        current_joint = copy.deepcopy(self.joint_status.joint_pos[:6])
        delta = self.home_joint - current_joint
        
        cmd_msg = YamCmd()
        cmd_msg.header.stamp = self.parent_node.get_clock().now().to_msg()
        cmd_msg.mode = MODE_JOINT
        
        if np.allclose(current_joint, self.home_joint, atol=self.ramp_step):
            cmd_msg.gripper = GRIPPER_CLOSED_VALUE
            cmd_msg.joint_pos = self.home_joint.tolist()
            return cmd_msg
        else:
            ramped_joint = np.where(np.abs(delta) <= self.ramp_step, self.home_joint, current_joint + np.sign(delta) * self.ramp_step)
            cmd_msg.gripper = GRIPPER_OPEN_VALUE
            cmd_msg.joint_pos = ramped_joint.tolist()
            return cmd_msg

    def calculate_initial_vr_se3_norm(self):
        if self.joint_status is None or self.current_vr_pose is None:
            self.parent_node.get_logger().warn(f"Cannot calculate initial pose for {self.arm_side} arm. Missing data.")
            return False
        
        eef_arm_msg = self.joint_status.end_pose
        
        # Extract position and quaternion components
        # x y z, w x y z
        position = np.array(eef_arm_msg[0:3])
        quaternion = np.array(eef_arm_msg[3:7])
        quaternion = quaternion / np.linalg.norm(quaternion)
        # Create SE3 from position and quaternion
        # scipy quat x y z w
        r = Rotation.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        eef_SE3_base = pin.SE3(r.as_matrix(), position)
        handle_SE3_vr = self.pose_msg_to_SE3(self.current_vr_pose)
        vr_SE3_eef = self.norm_SE3_handle.inverse() * handle_SE3_vr.inverse()
        # return self.base_SE3_norm * eef_SE3_base * vr_SE3_eef
        # The end_pose from arm is in Norm frame
        self.vr_SE3_norm = eef_SE3_base * vr_SE3_eef
        self.parent_node.get_logger().info(f"Initial pose for {self.arm_side} arm stored.")
        return True

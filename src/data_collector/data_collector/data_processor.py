from typing import Dict, Any, List
from cv_bridge import CvBridge
import numpy as np
import cv2
from arx5_arm_msg.msg import RobotCmd
from arx5_arm_msg.msg import RobotStatus
from yam_arm_msg.msg import YamCmd
from yam_arm_msg.msg import YamStatus
from geometry_msgs.msg import Twist

class DataProcessor:
    def __init__(self, config: Dict[str, Any]):
        self.config = config

    def process(self, msg) -> Any:
        raise NotImplementedError

    def get_dataset_config(self) -> Dict[str, Any]:
        return {
            'shape': tuple(self.config['output_shape']),
            'dtype': self.config['dtype'],
            'description': self.config['description'],
        }

    def get_message(self) -> str:
        return ''


class ImageProcessor(DataProcessor):
    def process(self, msg):
        """Process RGB images"""
        img = CvBridge().imgmsg_to_cv2(msg, desired_encoding='rgb8')
        if self.config.get('resize'):
            img = cv2.resize(img, tuple(self.config['resize'][:2]))
        return img


class DepthProcessor(DataProcessor):
    def process(self, msg):
        """Convert depth image message to numpy array"""
        # Convert to 16UC1 format (uint16) for depth preservation
        depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='16UC1')

        if self.config.get('resize'):
            depth_image = cv2.resize(depth_image,
                                     tuple(self.config['resize'][:2]))
        return depth_image

class PointcloudProcessor(DataProcessor):

    def process(self, msg):
        data = msg.data
        points = np.frombuffer(data, dtype=np.float32).reshape((-1, 4))[:, :3]

        not_nan_mask = ~np.isnan(points).any(axis=1)
        valid_points = points[not_nan_mask]

        num_points = self.config["output_shape"][0]
        if len(valid_points) < num_points:
            num_pad = num_points - len(valid_points)
            pad_points = np.zeros((num_pad, 3))
            sampled_points = np.concatenate([valid_points, pad_points], axis=0)
        else:
            sampled_indices = np.random.choice(valid_points.shape[0], size=num_points, replace=False)
            sampled_points = valid_points[sampled_indices]

        return sampled_points.astype(np.float16)

class EulerPoseProcessor(DataProcessor):
    def process(self, msg):
        x, y, z = msg.end_pos[:3]
        roll, pitch, yaw = msg.end_pos[3:]
        grip = msg.gripper

        xyz_rpy_grip = [x, y, z, roll, pitch, yaw, grip]

        return np.array(xyz_rpy_grip)


class JointPositionProcessor(DataProcessor):
    def process(self, msg):
        return np.array(msg.joint_pos)


class JointVelocityProcessor(DataProcessor):
    def process(self, msg):
        return np.array(msg.joint_vel)


class JointCurrentProcessor(DataProcessor):
    def process(self, msg):
        return np.array(msg.joint_cur)


class TwistProcessor(DataProcessor):
    def process(self, msg):

        angular_linear = [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z, msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

        return np.array(angular_linear)


class OdomProcessor(DataProcessor):
    def process(self, msg):

        twist = msg.twist.twist

        angular_linear = [twist.angular.x, twist.angular.y, twist.angular.z,twist.linear.x, twist.linear.y, twist.linear.z]

        return np.array(angular_linear)


class PedalProcessor(DataProcessor):
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.last_data = 0

    def process(self, msg):
        data = msg.buttons
        if data[0] == 1:
            value = 10
        else:
            value = 0

        self.last_data = value
        return value

    def get_message(self):
        if self.last_data != 0 :
            return 'pedal stepped!!!'
        else:
            return ''
        
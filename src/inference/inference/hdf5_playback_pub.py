"""
HDF5 Playback Publisher — replays a previously collected HDF5 episode as if
it were live inference output.

Reads joint position data from an HDF5 file and publishes YamCmd messages to
/YAM_INFER_L and /YAM_INFER_R at the original collection frequency, allowing
the intervention relay node to be tested with real recorded motions instead of
a trained ML policy.

Parameters
----------
hdf5_path   : str   (required) Absolute path to the .hdf5 file.
left_key    : str   (default '/state/joint_position/left')
                    HDF5 dataset key for the left arm, shape (T, 7).
right_key   : str   (default '/state/joint_position/right')
                    HDF5 dataset key for the right arm, shape (T, 7).
                    If the key does not exist in the file, the left arm data
                    is used as a fallback for the right arm as well.
loop        : bool  (default False) Replay in a loop; False = stop at end.
speed       : float (default 1.0)   Playback speed multiplier (0.5 = half speed).

Usage
-----
  # Minimal — loop the file at original speed
  pixi run ros2 run inference hdf5_playback_pub \
      --ros-args -p hdf5_path:=/home/user/data/collection_1234.hdf5

  # One-shot at half speed
  pixi run ros2 run inference hdf5_playback_pub \
      --ros-args -p hdf5_path:=/path/to/file.hdf5 -p loop:=false -p speed:=0.5
"""

import h5py
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from yam_arm_msg.msg import YamCmd


class HDF5PlaybackPub(Node):
    def __init__(self):
        super().__init__('hdf5_playback_pub')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('hdf5_path', '')
        self.declare_parameter('left_key', '/state/joint_position/left')
        self.declare_parameter('right_key', '/state/joint_position/right')
        self.declare_parameter('loop', False)
        self.declare_parameter('speed', 1.0)

        hdf5_path = self.get_parameter('hdf5_path').get_parameter_value().string_value
        left_key  = self.get_parameter('left_key').get_parameter_value().string_value
        right_key = self.get_parameter('right_key').get_parameter_value().string_value
        self._loop  = self.get_parameter('loop').get_parameter_value().bool_value
        speed = self.get_parameter('speed').get_parameter_value().double_value

        if not hdf5_path:
            self.get_logger().fatal('Parameter hdf5_path is required but not set.')
            raise RuntimeError('hdf5_path not set')

        # ── load data ─────────────────────────────────────────────────────
        self._left_data, self._right_data, freq = self._load(
            hdf5_path, left_key, right_key
        )
        self._n_frames = len(self._left_data)
        self._idx = 0
        self._paused = False  # True while intervention is active

        interval = 1.0 / (freq * speed)
        self.get_logger().info(
            f'Loaded {self._n_frames} frames from {hdf5_path}\n'
            f'  freq={freq:.1f} Hz  speed={speed}x  effective={freq*speed:.1f} Hz\n'
            f'  loop={self._loop}  duration={self._n_frames/freq:.1f}s per episode\n'
            f'  left  key : {left_key}\n'
            f'  right key : {right_key}'
        )

        # ── publishers ────────────────────────────────────────────────────
        self._pub_l = self.create_publisher(YamCmd, '/YAM_INFER_L', 1)
        self._pub_r = self.create_publisher(YamCmd, '/YAM_INFER_R', 1)

        # Freeze frame index while intervention is active so playback resumes
        # from the same position after the operator releases the master arm.
        self.create_subscription(Bool, '/intervention_mode', self._intervention_cb, 1)

        self.create_timer(interval, self._timer_cb)

    # ── data loading ──────────────────────────────────────────────────────

    def _load(self, path: str, left_key: str, right_key: str):
        with h5py.File(path, 'r') as f:
            if left_key not in f:
                raise KeyError(
                    f"Key '{left_key}' not found in {path}.\n"
                    f"Available keys: {list(f.keys())}"
                )
            left_data = np.array(f[left_key], dtype=np.float64)   # (T, 7)

            if right_key in f:
                right_data = np.array(f[right_key], dtype=np.float64)
                self.get_logger().info(f"Right arm data loaded from '{right_key}'")
            else:
                right_data = left_data.copy()
                self.get_logger().warn(
                    f"Key '{right_key}' not found — using left arm data for right arm."
                )

            freq = float(f['collection_frequency'][()]) if 'collection_frequency' in f else 50.0

        if left_data.ndim != 2 or left_data.shape[1] != 7:
            raise ValueError(
                f"Expected shape (T, 7) for '{left_key}', got {left_data.shape}"
            )

        return left_data, right_data, freq

    # ── intervention mode listener ────────────────────────────────────────

    def _intervention_cb(self, msg: Bool):
        if msg.data and not self._paused:
            self._paused = True
            self.get_logger().info(
                f'Playback paused at frame {self._idx} (intervention active)'
            )
        elif not msg.data and self._paused:
            self._paused = False
            self.get_logger().info(
                f'Playback resumed from frame {self._idx} (inference restored)'
            )

    # ── publish loop ──────────────────────────────────────────────────────

    def _timer_cb(self):
        if self._paused:
            return

        if self._idx >= self._n_frames:
            if self._loop:
                self._idx = 0
                self.get_logger().info('Episode ended — looping back to frame 0')
            else:
                self.get_logger().info('Episode ended — playback complete', once=True)
                return

        left_joints  = self._left_data[self._idx]   # shape (7,)
        right_joints = self._right_data[self._idx]  # shape (7,)

        self._pub_l.publish(self._make_cmd(left_joints))
        self._pub_r.publish(self._make_cmd(right_joints))

        self._idx += 1

    def _make_cmd(self, joints: np.ndarray) -> YamCmd:
        """Convert a (7,) joint array [j1..j6, gripper] to YamCmd."""
        cmd = YamCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.mode = 5
        cmd.joint_pos = joints[:6].tolist()
        cmd.gripper = float(joints[6])
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = HDF5PlaybackPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

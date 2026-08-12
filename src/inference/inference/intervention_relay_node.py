import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from threading import Thread, Lock
from pynput import keyboard

from yam_arm_msg.msg import YamCmd, YamStatus
from rclpy.qos import qos_profile_sensor_data


class InterventionRelayNode(Node):
    def __init__(self):
        super().__init__('intervention_relay_node')

        # transition_duration: seconds to smoothly interpolate from master arm
        # position back to inference position when pressing D.
        # Set to 0.0 to disable (instant switch, may cause jumps).
        self.declare_parameter('transition_duration', 1.0)
        transition_duration = self.get_parameter('transition_duration').get_parameter_value().double_value
        self._trans_total_steps = max(1, int(transition_duration * 50))  # 50 Hz timer

        self._lock = Lock()
        self._intervention_mode = False

        self._latest_master_l: YamStatus = None
        self._latest_master_r: YamStatus = None
        self._latest_infer_l: YamCmd = None
        self._latest_infer_r: YamCmd = None

        # Smooth transition state (intervention → inference)
        self._transitioning = False
        self._trans_steps_left = 0
        self._trans_start_l: np.ndarray = None   # master arm position at D press
        self._trans_start_r: np.ndarray = None
        self._trans_target_l: np.ndarray = None  # infer position at D press
        self._trans_target_r: np.ndarray = None

        # Subscriptions
        self.create_subscription(YamStatus, '/master_l_status', self._master_l_cb, qos_profile_sensor_data)
        self.create_subscription(YamStatus, '/master_r_status', self._master_r_cb, qos_profile_sensor_data)
        self.create_subscription(YamCmd, '/YAM_INFER_L', self._infer_l_cb, 1)
        self.create_subscription(YamCmd, '/YAM_INFER_R', self._infer_r_cb, 1)

        # Publishers
        self._pub_l = self.create_publisher(YamCmd, '/YAM_VR_L', 1)
        self._pub_r = self.create_publisher(YamCmd, '/YAM_VR_R', 1)
        self._pub_mode = self.create_publisher(Bool, '/intervention_mode', 1)

        # Services
        self.create_service(Trigger, '/start_intervention', self._start_intervention_srv)
        self.create_service(Trigger, '/stop_intervention', self._stop_intervention_srv)

        # 50 Hz relay timer
        self.create_timer(0.02, self._relay_timer)

        # Keyboard listener thread
        self._kb_thread = Thread(target=self._keyboard_listener, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            f"intervention_relay_node ready — press 'S' to take over, 'D' to resume inference\n"
            f"  transition_duration = {transition_duration:.1f}s ({self._trans_total_steps} steps)"
        )

    # ── callbacks ──────────────────────────────────────────────────────────

    def _master_l_cb(self, msg: YamStatus):
        with self._lock:
            self._latest_master_l = msg

    def _master_r_cb(self, msg: YamStatus):
        with self._lock:
            self._latest_master_r = msg

    def _infer_l_cb(self, msg: YamCmd):
        with self._lock:
            self._latest_infer_l = msg

    def _infer_r_cb(self, msg: YamCmd):
        with self._lock:
            self._latest_infer_r = msg

    # ── relay timer ────────────────────────────────────────────────────────

    def _relay_timer(self):
        with self._lock:
            intervention  = self._intervention_mode
            transitioning = self._transitioning
            master_l = self._latest_master_l
            master_r = self._latest_master_r
            infer_l  = self._latest_infer_l
            infer_r  = self._latest_infer_r

        if intervention:
            if master_l is not None:
                self._pub_l.publish(self._status_to_cmd(master_l))
            if master_r is not None:
                self._pub_r.publish(self._status_to_cmd(master_r))

        elif transitioning:
            with self._lock:
                steps_left   = self._trans_steps_left
                total_steps  = self._trans_total_steps
                start_l = self._trans_start_l
                start_r = self._trans_start_r
                target_l = self._trans_target_l
                target_r = self._trans_target_r

            # alpha goes from 0 (master position) to 1 (infer position)
            alpha = 1.0 - steps_left / total_steps
            interp_l = (1.0 - alpha) * start_l + alpha * target_l
            interp_r = (1.0 - alpha) * start_r + alpha * target_r
            self._pub_l.publish(self._joints_to_cmd(interp_l))
            self._pub_r.publish(self._joints_to_cmd(interp_r))

            with self._lock:
                self._trans_steps_left -= 1
                if self._trans_steps_left <= 0:
                    self._transitioning = False
                    self.get_logger().info('Transition complete — now following inference')

        else:
            if infer_l is not None:
                self._pub_l.publish(infer_l)
            if infer_r is not None:
                self._pub_r.publish(infer_r)

    # ── message helpers ────────────────────────────────────────────────────

    def _status_to_cmd(self, status: YamStatus) -> YamCmd:
        cmd = YamCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.mode = 5
        cmd.joint_pos = list(status.joint_pos[:6])
        cmd.gripper = float(status.joint_pos[6])
        return cmd

    def _joints_to_cmd(self, joints: np.ndarray) -> YamCmd:
        """Convert a (7,) array [j1..j6, gripper] to YamCmd."""
        cmd = YamCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.mode = 5
        cmd.joint_pos = joints[:6].tolist()
        cmd.gripper = float(joints[6])
        return cmd

    @staticmethod
    def _status_to_joints(status: YamStatus) -> np.ndarray:
        return np.array(list(status.joint_pos[:6]) + [status.joint_pos[6]])

    @staticmethod
    def _cmd_to_joints(cmd: YamCmd) -> np.ndarray:
        return np.array(list(cmd.joint_pos[:6]) + [cmd.gripper])

    # ── mode switching ─────────────────────────────────────────────────────

    def _set_intervention_mode(self, value: bool):
        with self._lock:
            was_intervention = self._intervention_mode
            self._intervention_mode = value

            # When leaving intervention mode, start a smooth transition back to
            # the inference position to avoid sudden arm jumps.
            if was_intervention and not value:
                master_l = self._latest_master_l
                master_r = self._latest_master_r
                infer_l  = self._latest_infer_l
                infer_r  = self._latest_infer_r

                if master_l is not None and infer_l is not None:
                    self._trans_start_l  = self._status_to_joints(master_l)
                    self._trans_start_r  = self._status_to_joints(master_r) if master_r is not None else self._trans_start_l.copy()
                    self._trans_target_l = self._cmd_to_joints(infer_l)
                    self._trans_target_r = self._cmd_to_joints(infer_r) if infer_r is not None else self._trans_target_l.copy()
                    self._trans_steps_left = self._trans_total_steps
                    self._transitioning = True
                    self.get_logger().info(
                        f'Starting {self._trans_total_steps}-step transition back to inference'
                    )

        self._pub_mode.publish(Bool(data=value))

    # ── services ───────────────────────────────────────────────────────────

    def _start_intervention_srv(self, request, response):
        self._set_intervention_mode(True)
        self.get_logger().info('Intervention started (master arm controls slave)')
        response.success = True
        response.message = 'Intervention mode ON'
        return response

    def _stop_intervention_srv(self, request, response):
        self._set_intervention_mode(False)
        self.get_logger().info('Inference resumed — transitioning smoothly')
        response.success = True
        response.message = 'Inference mode ON (transitioning)'
        return response

    # ── keyboard ───────────────────────────────────────────────────────────

    def _keyboard_listener(self):
        def on_press(key):
            try:
                if not hasattr(key, 'char') or key.char is None:
                    return
                ch = key.char.lower()
                if ch == 's':
                    self._set_intervention_mode(True)
                    self.get_logger().info("Key 'S': intervention ON (master → slave)")
                elif ch == 'd':
                    self._set_intervention_mode(False)
                    self.get_logger().info("Key 'D': inference ON (transitioning)")
            except Exception as e:
                self.get_logger().error(f'Keyboard handler error: {e}')

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()


def main(args=None):
    rclpy.init(args=args)
    node = InterventionRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from threading import Thread, Lock
from pynput import keyboard


class KeyboardCollectTrigger(Node):
    def __init__(self):
        super().__init__('keyboard_collect_trigger')

        self.state_lock = Lock()
        self.is_recording = False

        self.start_client = self.create_client(Trigger, 'start_collect')
        self.stop_client = self.create_client(Trigger, 'stop_collect')

        self.monitor_thread = Thread(target=self._keyboard_listener, daemon=True)
        self.monitor_thread.start()

        self.get_logger().info("Keyboard trigger ready: press 'S' to start, 'D' to stop")

    def _keyboard_listener(self):
        def on_press(key):
            try:
                if hasattr(key, 'char') and key.char is not None:
                    ch = key.char.lower()
                    if ch == 's':
                        self._start_collect()
                    elif ch == 'd':
                        self._stop_collect()
            except Exception as e:
                self.get_logger().error(f"Key handler error: {e}")

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def _start_collect(self):
        with self.state_lock:
            if self.is_recording:
                self.get_logger().warn("Already recording, ignoring 'S'")
                return
            self.is_recording = True

        self.get_logger().info(">>> START recording")
        future = self.start_client.call_async(Trigger.Request())
        future.add_done_callback(self._start_done)

    def _stop_collect(self):
        with self.state_lock:
            if not self.is_recording:
                self.get_logger().warn("Not recording, ignoring 'D'")
                return
            self.is_recording = False

        self.get_logger().info(">>> STOP recording, saving...")
        future = self.stop_client.call_async(Trigger.Request())
        future.add_done_callback(self._stop_done)

    def _start_done(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f"Start: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Start service failed: {e}")
            with self.state_lock:
                self.is_recording = False

    def _stop_done(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f"Stop: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Stop service failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCollectTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

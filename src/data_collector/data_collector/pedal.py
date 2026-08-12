import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from threading import Thread, Event, Lock
from pynput import keyboard


class PedalPub(Node):
    def __init__(self):
        super().__init__('PedalPub')

        self.declare_parameter('publish_rate', 15)
        self.publish_rate = self.get_parameter('publish_rate').value

        self.state_lock = Lock()
        self.current_state = 0
        self.shutdown_event = Event()

        self.publisher = self.create_publisher(Joy, 'pedal_status', 5)

        self.monitor_thread = Thread(target=self._responsive_monitor, daemon=True)
        self.monitor_thread.start()

        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_callback)
        self.get_logger().info(f"Pedal publisher ready (Rate: {self.publish_rate}Hz)")

    def _responsive_monitor(self):
        def on_press(key):
            try:
                if hasattr(key, 'char') and key.char is not None:
                    if key.char.lower() == 'c':
                        with self.state_lock:
                            self.current_state = 1
            except Exception as e:
                self.get_logger().error(f"Error in on_press: {str(e)}")

        try:
            with keyboard.Listener(on_press=on_press) as listener:
                listener.join()
        except Exception as e:
            self.get_logger().error(f"Monitor thread failed: {str(e)}")
            self.shutdown_event.set()

    def _publish_callback(self):
        try:
            with self.state_lock:
                msg = Joy()
                msg.buttons = [self.current_state]
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(msg)
                self.current_state = 0
        except Exception as e:
            self.get_logger().error(f"Publish failed: {str(e)}")

    def destroy_node(self):
        self.shutdown_event.set()
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=0.1)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PedalPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

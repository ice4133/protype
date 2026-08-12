import serial
from rclpy.node import Node
import rclpy
from std_msgs.msg import Int32  # 用于接收目标高度
import time
from sensor_msgs.msg import Joy
import numpy as np

class LiftController(Node):
    def __init__(self):
        super().__init__('lift_node')

        # 串口配置
        self.port = "/dev/ttyCH341USB0"  # 根据实际情况修改
        self.baud = 115200       
        self.current_speed = 0  # 如果没有按键按下，速度归零
        self.past_height = 0
        self.target_speed = 0

        # 初始化串口
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02
            )
            self.get_logger().info(f"Opened {self.port} at {self.baud} baud, 8N1.")
            time.sleep(2)
            self.ser.write("enable\n".encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"串口打开失败: {e}")
            return

        # 订阅目标高度的主题
        self.subscription = self.create_subscription(Joy, '/vr_height_speed', self.vr_height_speed_callback, 1)
        # 发布当前位置的主题
        self.position_publisher = self.create_publisher(Int32, '/current_height', 1)
        self.speed_publisher = self.create_publisher(Int32, '/current_speed', 1)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("LiftController节点已启动，等待速度消息...")

    def timer_callback(self):
        command = f"speed {int(self.target_speed)}\n"
        receive = f"INFO\n"
        try:
            # 发送速度指令
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent: {command.strip()}")

            # 请求数据
            self.ser.write(receive.encode('utf-8'))
            
            # 读取两行数据（位置 + 速度）
            lines = []
            while len(lines) < 2:  # 确保读取两行
                line = self.ser.readline().decode('utf-8').strip()
                if line:  # 忽略空行
                    lines.append(line)
            
            # 解析位置（第一行）
            if len(lines) > 0 and lines[0].startswith("P="):
                try:
                    current_height = int(float(lines[0][2:]))
                    self.publish_current_height(current_height)
                except ValueError:
                    self.get_logger().warn(f"位置数据无效: {lines[0]}")
            
            # 解析速度（第二行）
            if len(lines) > 1 and lines[1].startswith("V="):
                try:
                    current_speed = int(lines[1][2:])
                    self.publish_current_speed(current_speed)
                except ValueError:
                    self.get_logger().warn(f"速度数据无效: {lines[1]}")

        except serial.SerialException as e:
            self.get_logger().error(f"串口通信失败: {e}")
            
            
    def vr_height_speed_callback(self, msg):
        self.target_speed = np.int32(msg.axes[0] * 100)
        

    def publish_current_height(self, height):
        # 发布当前位置到
        msg = Int32()
        msg.data = height
        self.position_publisher.publish(msg)
        self.get_logger().info(f"Published current height: {height}")

    def publish_current_speed(self, speed):
        # 发布当前速度
        msg = Int32()
        msg.data = speed
        self.speed_publisher.publish(msg)
        self.get_logger().info(f"Published current speed: {speed}")


    def destroy_node(self):
        # 程序结束前关闭串口
        self.ser.write("disable\n".encode('utf-8'))
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，程序结束")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

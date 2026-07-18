#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")
        self.publisher_ = self.create_publisher(String, "/control_mode", 10)
        self.get_logger().info("Başladi.")

        while rclpy.ok():
            key = input("Mode (n=nav, j=joy): ").lower()

            msg = String()

            if key == 'n':
                msg.data = 'NAV'
                self.publisher_.publish(msg)
                self.get_logger().info("NAV")

            elif key == 'j':
                msg.data = 'JOY'
                self.publisher_.publish(msg)
                self.get_logger().info("Joy")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
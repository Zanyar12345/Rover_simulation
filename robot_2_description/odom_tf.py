#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTfBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Başladi")

    def odom_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp 
        t.header.frame_id = 'odom'
        
        t.child_frame_id = 'base_footprint' 
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdomTfBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
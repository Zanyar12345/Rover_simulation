#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray


class Publisher(Node):

    def __init__(self):
        super().__init__('Publisher')

        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )

        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steer_position_controller/commands',
            10
        )
        self.slowness = -1/2
        self.vel = [0.0, 0.0, 0.0, 0.0]
        self.ang = [0.0, 0.0, 0.0, 0.0]

        self.create_subscription(Int32, 'wheel_front_left_speed',  self.fl_speed_cb, 10)
        self.create_subscription(Int32, 'wheel_front_right_speed', self.fr_speed_cb, 10)
        self.create_subscription(Int32, 'wheel_rear_left_speed',   self.rl_speed_cb, 10)
        self.create_subscription(Int32, 'wheel_rear_right_speed',  self.rr_speed_cb, 10)

        self.create_subscription(Int32, 'wheel_front_left_angle',  self.fl_angle_cb, 10)
        self.create_subscription(Int32, 'wheel_front_right_angle', self.fr_angle_cb, 10)
        self.create_subscription(Int32, 'wheel_rear_left_angle',   self.rl_angle_cb, 10)
        self.create_subscription(Int32, 'wheel_rear_right_angle',  self.rr_angle_cb, 10)

        self.create_timer(0.02, self.publish_commands)   

    def fl_speed_cb(self, msg): 
        self.vel[2] = float(msg.data)*(self.slowness)
    def fr_speed_cb(self, msg): 
        self.vel[3] = float(msg.data)*(self.slowness)
    def rr_speed_cb(self, msg): 
        self.vel[1] = float(msg.data)*(self.slowness)
    def rl_speed_cb(self, msg): 
        self.vel[0] = float(msg.data)*(self.slowness)

    def fl_angle_cb(self, msg): 
        self.ang[2] = (180-msg.data)*1.57/90
    def fr_angle_cb(self, msg): 
        self.ang[3] = (180-msg.data)*1.57/90
    def rr_angle_cb(self, msg): 
        self.ang[0] = (180-msg.data)*1.57/90
    def rl_angle_cb(self, msg): 
        self.ang[1] = (180-msg.data)*1.57/90

    def publish_commands(self):

        vel_msg = Float64MultiArray()
        ang_msg = Float64MultiArray()

        vel_msg.data = self.vel
        ang_msg.data = self.ang

        self.wheel_pub.publish(vel_msg)
        self.steer_pub.publish(ang_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

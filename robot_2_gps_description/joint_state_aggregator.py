#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import time
import math
class JointStateAggregator(Node):
    def __init__(self):
        super().__init__('joint_state_aggregator')
        
        # Publish to /joint_states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Store latest speeds (default 0.0)
        self.speeds = {
            'sol_on_drive_to_wheel': 0.0,
            'sag_on_drive_to_wheel': 0.0,
            'sol_arka_drive_to_wheel': 0.0,
            'sag_arka_drive_to_wheel': 0.0
        }
        
        # Store latest angles (default 0.0 rad)
        self.angles = {
            'sol_on_gearbox_to_mil': 0.0,
            'sag_on_gearbox_to_mil': 0.0,
            'sol_arka_gearbox_to_mil': 0.0,
            'sag_arka_gearbox_to_mil': 0.0
        }
        
        # Pico'dan gelen Int32 degeri RPM (Dakikadaki devir) oldugu icin,
        # RPM'i gercek radyan/saniye (rad/s) birimine ceviren kesin fiziksel carpan: (2 * Pi / 60)
        self.speed_multiplier = 0.104719755
        
        # Subscriptions for the 4 Pico bridge nodes
        self.create_subscription(
            Int32, 
            '/wheel_front_left/status/current_speed', 
            self.fl_callback, 10)
            
        self.create_subscription(
            Int32, 
            '/wheel_front_right/status/current_speed', 
            self.fr_callback, 10)
            
        self.create_subscription(
            Int32, 
            '/wheel_rear_left/status/current_speed', 
            self.rl_callback, 10)
            
        self.create_subscription(
            Int32, 
            '/wheel_rear_right/status/current_speed', 
            self.rr_callback, 10)
            
        # Angle Subscriptions
        self.create_subscription(Int32, '/wheel_front_left/status/current_angle', self.fl_angle_callback, 10)
        self.create_subscription(Int32, '/wheel_front_right/status/current_angle', self.fr_angle_callback, 10)
        self.create_subscription(Int32, '/wheel_rear_left/status/current_angle', self.rl_angle_callback, 10)
        self.create_subscription(Int32, '/wheel_rear_right/status/current_angle', self.rr_angle_callback, 10)
            
        import math
        self.math = math

        # Timer to publish at 50Hz (0.02 seconds)
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        self.get_logger().info("Joint State Aggregator Started! Bridging Pico to Odom...")

    # NOTE: The Int32 data from Pico is assumed to be directly compatible with rad/s here.
    # If it is scaled (e.g. by 1000) or in RPM, you can multiply/divide it here.
    def fl_callback(self, msg):
        self.speeds['sol_on_drive_to_wheel'] = float(msg.data) * self.speed_multiplier
        
    def fr_callback(self, msg):
        self.speeds['sag_on_drive_to_wheel'] = float(msg.data) * self.speed_multiplier
        
    def rl_callback(self, msg):
        self.speeds['sol_arka_drive_to_wheel'] = float(msg.data) * self.speed_multiplier
        
    def rr_callback(self, msg):
        self.speeds['sag_arka_drive_to_wheel'] = float(msg.data) * self.speed_multiplier

    def fl_angle_callback(self, msg):
        self.angles['sol_on_gearbox_to_mil'] = -(float(msg.data) - 180.0) * self.math.pi / 180.0
        
    def fr_angle_callback(self, msg):
        self.angles['sag_on_gearbox_to_mil'] = -(float(msg.data) - 180.0) * self.math.pi / 180.0
        
    def rl_angle_callback(self, msg):
        self.angles['sol_arka_gearbox_to_mil'] = -(float(msg.data) - 180.0) * self.math.pi / 180.0
        
    def rr_angle_callback(self, msg):
        self.angles['sag_arka_gearbox_to_mil'] = -(float(msg.data) - 180.0) * self.math.pi / 180.0

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Extract names and velocities for wheels
        for joint_name, speed in self.speeds.items():
            msg.name.append(joint_name)
            msg.velocity.append(speed)
            msg.position.append(0.0) 
            msg.effort.append(0.0)
            
        # Extract names and positions for steering
        for joint_name, angle in self.angles.items():
            msg.name.append(joint_name)
            msg.velocity.append(0.0)
            msg.position.append(angle)
            msg.effort.append(0.0)
            
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

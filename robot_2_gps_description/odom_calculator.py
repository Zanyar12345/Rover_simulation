#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
import time

class OdomCalculator(Node):
    def __init__(self):
        super().__init__('odom_calculator')
        
        # Odom Yayinci
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # JointState Dinleyici
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # Fiziksel Olculer (Metre)
        self.wheel_radius = 0.09  
        self.rover_length = 0.95 # On ve arka tekerler arasi mesafe
        self.rover_width = 0.82  # Sag ve sol teker arasi mesafe
        
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0
        
        self.last_time = time.time()

        self.get_logger().info("Odometry Calculator Basladi!")

    def joint_cb(self, msg: JointState):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        try:
            # Tekerlek hiz indexleri
            idx_sol_on = msg.name.index("sol_on_drive_to_wheel")
            idx_sag_on = msg.name.index("sag_on_drive_to_wheel")
            idx_sol_arka = msg.name.index("sol_arka_drive_to_wheel")
            idx_sag_arka = msg.name.index("sag_arka_drive_to_wheel")
            
            # Tekerlek direksiyon acisi indexleri
            idx_sol_on_steer = msg.name.index("sol_on_gearbox_to_mil")
            idx_sag_on_steer = msg.name.index("sag_on_gearbox_to_mil")
            idx_sol_arka_steer = msg.name.index("sol_arka_gearbox_to_mil")
            idx_sag_arka_steer = msg.name.index("sag_arka_gearbox_to_mil")
            
            # Tekerlek hizlarini al (rad/s)
            v_sol_on = msg.velocity[idx_sol_on]
            v_sag_on = msg.velocity[idx_sag_on]
            v_sol_arka = msg.velocity[idx_sol_arka]
            v_sag_arka = msg.velocity[idx_sag_arka]

            # Tekerlek acilarini al (rad)
            a_sol_on = msg.position[idx_sol_on_steer]
            a_sag_on = msg.position[idx_sag_on_steer]
            a_sol_arka = msg.position[idx_sol_arka_steer]
            a_sag_arka = msg.position[idx_sag_arka_steer]

            # 1. Her tekerlegin yerel X ve Y hizlarini hesapla
            vx_fl = v_sol_on * self.wheel_radius * math.cos(a_sol_on)
            vy_fl = v_sol_on * self.wheel_radius * math.sin(a_sol_on)
            
            vx_fr = v_sag_on * self.wheel_radius * math.cos(a_sag_on)
            vy_fr = v_sag_on * self.wheel_radius * math.sin(a_sag_on)
            
            vx_rl = v_sol_arka * self.wheel_radius * math.cos(a_sol_arka)
            vy_rl = v_sol_arka * self.wheel_radius * math.sin(a_sol_arka)
            
            vx_rr = v_sag_arka * self.wheel_radius * math.cos(a_sag_arka)
            vy_rr = v_sag_arka * self.wheel_radius * math.sin(a_sag_arka)

            # 2. Robotun merkezinin cizgisel hizini (V_x, V_y) hesapla
            linear_x = (vx_fl + vx_fr + vx_rl + vx_rr) / 4.0
            linear_y = (vy_fl + vy_fr + vy_rl + vy_rr) / 4.0

            # 3. Robotun donme hizini (Yaw Rate, Omega_z) hesapla (Katı Cisim Kinematiği Çapraz Çarpım Yöntemi)
            L = self.rover_length
            W = self.rover_width
            
            # (x_i, y_i)
            # FL = (L/2, W/2)
            # FR = (L/2, -W/2)
            # RL = (-L/2, W/2)
            # RR = (-L/2, -W/2)
            
            omega_fl = ((L/2.0) * vy_fl - (W/2.0) * vx_fl)
            omega_fr = ((L/2.0) * vy_fr - (-W/2.0) * vx_fr)
            omega_rl = ((-L/2.0) * vy_rl - (W/2.0) * vx_rl)
            omega_rr = ((-L/2.0) * vy_rr - (-W/2.0) * vx_rr)
            
            r_squared_sum = (L**2 + W**2)  # 4 * ((L/2)^2 + (W/2)^2) = L^2 + W^2
            angular_z = (omega_fl + omega_fr + omega_rl + omega_rr) / r_squared_sum

            # 4. Global X, Y ve Aciyi (Theta) guncelle
            self.theta += angular_z * dt
            self.x_pos += (linear_x * math.cos(self.theta) - linear_y * math.sin(self.theta)) * dt
            self.y_pos += (linear_x * math.sin(self.theta) + linear_y * math.cos(self.theta)) * dt

            # 5. Odometry mesaji hazirla ve yayinla
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            
            # Pozisyon
            odom.pose.pose.position.x = self.x_pos
            odom.pose.pose.position.y = self.y_pos
            odom.pose.pose.position.z = 0.0
            
            # Euler to Quaternion (Yaw -> Quaternion)
            odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
            odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            
            # Hiz
            odom.twist.twist.linear.x = linear_x
            odom.twist.twist.linear.y = linear_y
            odom.twist.twist.angular.z = angular_z
            
            self.odom_pub.publish(odom)

        except ValueError:
            # joint_states icinde henuz tekerlekler yayinlanmamis olabilir
            pass

def main():
    rclpy.init()
    node = OdomCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

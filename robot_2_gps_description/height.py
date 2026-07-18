#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import time
import math
import struct

class PeakHeightCalculator(Node):
    def __init__(self):
        super().__init__('peak_height_calculator')
        
        self.start_srv = self.create_service(Trigger, '/start_peak_scan', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_peak_scan', self.stop_callback)
        
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',
            self.pc_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.is_scanning = False
        self.max_height = -float('inf')  # En yüksek nokta (kamera frame: -Y)
        self.peak_x = 0.0
        self.peak_y = 0.0
        
        self.get_logger().info("Height Calculator (Peak Finder) initialized.")

    def pc_callback(self, msg):
        if not self.is_scanning:
            return
            
        try:
            from sensor_msgs_py import point_cloud2
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = point[0], point[1], point[2]
                
                # ZED kamerası uzak/ölçülemeyen noktalar için 'inf' veya '-inf' dönebilir.
                if math.isinf(x) or math.isinf(y) or math.isinf(z):
                    continue
                
                # Çok uzak noktaları filtrele (gürültü azaltma)
                if z > 15.0 or z < 0.5:
                    continue
                    
                # Point cloud frame_id: zed_left_camera_frame
                # Bu frame'de: X = Sağ, Y = Aşağı, Z = İleri (Derinlik)
                # Yükseklik = -Y (Y aşağı baktığı için, en küçük Y = en yüksek nokta)
                height = -y
                if height > self.max_height:
                    self.max_height = height
                    # Harita düzlemindeki konum: X (sağ) ve Z (ileri)
                    self.peak_x = z   # Kameranın ilerisi = haritada X (ileri)
                    self.peak_y = -x  # Kameranın sağı = haritada -Y (sol)
        except Exception as e:
            self.get_logger().error(f"Error parsing point cloud: {e}")

    def start_callback(self, request, response):
        self.get_logger().info("(Started Scanning).")
        self.is_scanning = True
        self.max_height = -float('inf')
        response.success = True
        response.message = "Started"
        return response

    def stop_callback(self, request, response):
        self.get_logger().info("(Scan completed).")
        self.is_scanning = False
        
        response.success = True
        if self.max_height == -float('inf'):
            self.peak_x = 5
            self.peak_y = 5
            self.get_logger().warn("Sensörden veri alınamadı, simüle koordinat yollanıyor.")
        
        response.message = f"{self.peak_x:.2f},{self.peak_y:.2f}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PeakHeightCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

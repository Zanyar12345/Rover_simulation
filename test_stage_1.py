#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class TestStage1(Node):
    def __init__(self):
        super().__init__('test_stage_1_node')
        self.publisher_ = self.create_publisher(String, '/rscp/received_task', 10)
        
        self.get_logger().info("Baglanti bekleniyor...")
        
        # ROS 2 Publisher'ın arka planda aboneyi bulması bazen 2-3 saniye sürer.
        # Abone sayısını kontrol ederek bağlanana kadar bekleyelim:
        while self.publisher_.get_subscription_count() == 0:
            time.sleep(0.5)
            
        self.get_logger().info("Baglanti Kuruldu! Sinyaller gonderiliyor...")
        time.sleep(0.5) # Emin olmak için ufak bir pay
        
        # 1. Aşama 1'i ayarla (Anten Görevi)
        msg = String()
        msg.data = json.dumps({"type": "set_stage", "stage": 1})
        self.publisher_.publish(msg)
        self.get_logger().info("Gonderildi: Stage 1")
        time.sleep(0.5)
        
        # 2. Otonom Sistemi Arm Et
        msg.data = json.dumps({"type": "arm_disarm", "arm": True})
        self.publisher_.publish(msg)
        self.get_logger().info("Gonderildi: Arm")
        time.sleep(0.5)
        
        # 3. Arama Alanı Sinyali Gönder (0.0 veriyoruz ki YAML dosyasını kullansın)
        msg.data = json.dumps({
            "type": "search_area", 
            "latitude": 0.0, 
            "longitude": 0.0, 
            "radius": 10.0
        })
        self.publisher_.publish(msg)
        self.get_logger().info("Gonderildi: Search Area")
        
        # Kapanmadan önce mesajların iletilmesi için bekle
        time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = TestStage1()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

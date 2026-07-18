#!/usr/bin/env python3
"""
ARC'26 Otonom Görev Test Aracı
Kullanım: python3 test_mission.py <stage_numarası>

Stage 1: Anten Görevi (Zirve bul + koordinat gönder)
Stage 2: Shackleton Krateri (İlmenit kaya bul + koordinat gönder)  
Stage 3: Lav Tüpü (Giriş + ölçüm + çıkış)
Stage 4: Airlock'a Dönüş (ArUco ile giriş)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys

class TestMission(Node):
    def __init__(self, stage):
        super().__init__('test_mission_node')
        self.publisher_ = self.create_publisher(String, '/rscp/received_task', 10)
        
        self.get_logger().info("Baglanti bekleniyor...")
        while self.publisher_.get_subscription_count() == 0:
            time.sleep(0.5)
        self.get_logger().info("Baglanti kuruldu!")
        time.sleep(0.5)

        if stage == 1:
            self.test_stage_1()
        elif stage == 2:
            self.test_stage_2()
        elif stage == 3:
            self.test_stage_3()
        elif stage == 4:
            self.test_stage_4()
        else:
            self.get_logger().error(f"Gecersiz stage: {stage}. 1-4 arasi giriniz.")
        
        time.sleep(1.0)

    def send(self, data):
        msg = String()
        msg.data = json.dumps(data)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Gonderildi: {data['type']}")
        time.sleep(0.5)

    def test_stage_1(self):
        """Anten Görevi: Airlock çıkış → Anten bölgesi → Zirve bul → Anten bırak"""
        self.get_logger().info("=== STAGE 1: ANTEN GOREVI ===")
        self.send({"type": "set_stage", "stage": 1})
        self.send({"type": "arm_disarm", "arm": True})
        self.send({"type": "search_area", "latitude": 0.0, "longitude": 0.0, "radius": 10.0})

    def test_stage_2(self):
        """Shackleton: Kratere git → 360° tara → İlmenit bul → Koordinat gönder"""
        self.get_logger().info("=== STAGE 2: SHACKLETON KRATERI ===")
        self.send({"type": "set_stage", "stage": 2})
        self.send({"type": "arm_disarm", "arm": True})
        self.send({"type": "search_area", "latitude": 0.0, "longitude": 0.0, "radius": 10.0})

    def test_stage_3(self):
        """Lav Tüpü: Girişe git → (sunucu StartExploration gönderecek) → Tünele gir"""
        self.get_logger().info("=== STAGE 3: LAV TUPU ===")
        self.get_logger().info("Not: Once navigate_to_gps ile girişe gidecek.")
        self.get_logger().info("      Varildiginda WAITING_FOR_SERVER'a donecek.")
        self.get_logger().info("      Sonra tekrar bu script'i 'start_exploration' ile calistirin.")
        self.send({"type": "set_stage", "stage": 3})
        self.send({"type": "arm_disarm", "arm": True})
        self.send({"type": "navigate_to_gps", "latitude": 0.0, "longitude": 0.0})

    def test_stage_3_explore(self):
        """Lav Tüpü girişe vardıktan sonra keşfi başlat"""
        self.get_logger().info("=== STAGE 3: KESIF BASLAT ===")
        self.send({"type": "start_exploration"})

    def test_stage_4(self):
        """Airlock'a Dönüş: GPS ile yaklaş → ArUco ara → Hizala → İçeri gir"""
        self.get_logger().info("=== STAGE 4: AIRLOCK DONUS ===")
        self.send({"type": "set_stage", "stage": 4})
        self.send({"type": "arm_disarm", "arm": True})
        self.send({"type": "navigate_to_gps", "latitude": 0.0, "longitude": 0.0})

def main():
    if len(sys.argv) < 2:
        print("Kullanim: python3 test_mission.py <stage>")
        print("  Stage 1: Anten Gorevi")
        print("  Stage 2: Shackleton Krateri")
        print("  Stage 3: Lav Tupu (girişe git)")
        print("  Stage 3e: Lav Tupu (kesfi baslat)")
        print("  Stage 4: Airlock Donus")
        return

    rclpy.init()
    
    arg = sys.argv[1]
    if arg == "3e":
        node = TestMission.__new__(TestMission)
        Node.__init__(node, 'test_mission_node')
        node.publisher_ = node.create_publisher(String, '/rscp/received_task', 10)
        node.get_logger().info("Baglanti bekleniyor...")
        while node.publisher_.get_subscription_count() == 0:
            time.sleep(0.5)
        time.sleep(0.5)
        node.test_stage_3_explore()
        time.sleep(1.0)
    else:
        stage = int(arg)
        node = TestMission(stage)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

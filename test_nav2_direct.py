#!/usr/bin/env python3
"""
ARC'26 — GPS'siz Test (Direkt Nav2 metre hedefleri)
Kullanım: python3 test_nav2_direct.py <stage>

Rover'ın önüne kısa mesafeler koyarak her görevi test eder.
GPS modülüne gerek yok — Nav2 harita üzerinde çalışır.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import time
import sys
import math

class DirectNavTest(Node):
    def __init__(self):
        super().__init__('direct_nav_test')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rscp_pub = self.create_publisher(String, '/rscp/received_task', 10)
        
        self.get_logger().info("Nav2 server bekleniyor...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 hazır!")

    def go_to(self, x, y, description=""):
        """Nav2'ye metre cinsinden hedef gönder ve varmasını bekle."""
        self.get_logger().info(f">>> {description} → Hedef: ({x}, {y}) metre")
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Hedef reddedildi!")
            return False
        
        self.get_logger().info("Hedef kabul edildi, gidiyor...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        self.get_logger().info("Hedefe varıldı! ✓")
        return True

    def spin_360(self, duration=15):
        """Kendi etrafında dön (saniye cinsinden)."""
        self.get_logger().info(f">>> 360° dönüş başlıyor ({duration} sn)...")
        twist = Twist()
        twist.angular.z = 0.4
        
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Dönüş tamamlandı! ✓")

    def drive_forward(self, duration=5, speed=0.3):
        """Düz ileri sür."""
        self.get_logger().info(f">>> Düz ileri ({duration} sn, {speed} m/s)...")
        twist = Twist()
        twist.linear.x = speed
        
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Durdu! ✓")

    def send_rscp(self, data):
        """RSCP komutu simüle et."""
        msg = String()
        msg.data = json.dumps(data)
        self.rscp_pub.publish(msg)

    # ==================================================================

    def test_stage_1(self):
        """Anten Görevi — GPS yerine direkt metre hedefleri"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("STAGE 1: ANTEN GÖREVİ (Nav2 direkt)")
        self.get_logger().info("=" * 50)
        
        input("ENTER'a bas → Airlock'tan çıkış (1m ileri)")
        self.go_to(1.0, 0.0, "Airlock'tan çıkış")
        
        input("ENTER'a bas → Anten bölgesine git (3m ileri)")
        self.go_to(3.0, 0.0, "Anten bölgesine gidiş")
        
        input("ENTER'a bas → 360° dönüş (zirve tarama)")
        self.spin_360(duration=18)
        
        input("ENTER'a bas → Zirve noktasına git (1m ileri)")
        self.go_to(4.0, 0.0, "Zirve noktasına gidiş")
        
        self.get_logger().info("Anten bırakılıyor (2 sn bekleme)...")
        time.sleep(2)
        self.get_logger().info("✅ STAGE 1 TAMAMLANDI!")

    def test_stage_2(self):
        """Kaya Tespiti — Kratere git + 360° tara"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("STAGE 2: KAYA TESPİTİ (Nav2 direkt)")
        self.get_logger().info("=" * 50)
        
        input("ENTER'a bas → Kratere git (2m ileri)")
        self.go_to(2.0, 0.0, "Kratere gidiş")
        
        input("ENTER'a bas → 360° dönüş (kaya tarama)")
        self.get_logger().info("vision_perception.py çalışıyorsa kamera taramaya başlayacak...")
        self.spin_360(duration=20)
        
        self.get_logger().info("✅ STAGE 2 TAMAMLANDI!")

    def test_stage_3(self):
        """Lav Tüpü — Girişe git + gir + ölç"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("STAGE 3: LAV TÜPÜ (Nav2 direkt)")
        self.get_logger().info("=" * 50)
        
        input("ENTER'a bas → Lav tüpü girişine git (2m ileri)")
        self.go_to(2.0, 0.0, "Lav tüpü girişi")
        
        input("ENTER'a bas → 360° dönüş (ArUco ara)")
        self.spin_360(duration=18)
        
        input("ENTER'a bas → Tünele gir (5 sn düz ileri)")
        self.drive_forward(duration=5, speed=0.25)
        
        self.get_logger().info("✅ STAGE 3 TAMAMLANDI!")

    def test_stage_4(self):
        """Airlock'a Dönüş"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("STAGE 4: AIRLOCK DÖNÜŞ (Nav2 direkt)")
        self.get_logger().info("=" * 50)
        
        input("ENTER'a bas → Airlock'a dön (başlangıç noktasına)")
        self.go_to(0.0, 0.0, "Airlock'a dönüş")
        
        input("ENTER'a bas → 360° dönüş (ArUco ara)")
        self.spin_360(duration=18)
        
        input("ENTER'a bas → Airlock'a gir (5 sn düz ileri)")
        self.drive_forward(duration=5, speed=0.3)
        
        self.get_logger().info("✅ STAGE 4 TAMAMLANDI! GÖREV BİTTİ! 🏁")

def main():
    if len(sys.argv) < 2:
        print("Kullanım: python3 test_nav2_direct.py <stage>")
        print("  1 → Anten Görevi")
        print("  2 → Kaya Tespiti")
        print("  3 → Lav Tüpü")
        print("  4 → Airlock Dönüş")
        print("  all → Hepsini sırayla")
        return

    rclpy.init()
    node = DirectNavTest()
    
    stage = sys.argv[1]
    
    if stage == '1':
        node.test_stage_1()
    elif stage == '2':
        node.test_stage_2()
    elif stage == '3':
        node.test_stage_3()
    elif stage == '4':
        node.test_stage_4()
    elif stage == 'all':
        node.test_stage_1()
        node.test_stage_2()
        node.test_stage_3()
        node.test_stage_4()
    else:
        print(f"Geçersiz stage: {stage}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

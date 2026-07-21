#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
import time
import sys

class RscpMockServer(Node):
    def __init__(self):
        super().__init__('rscp_mock_server')
        
        # Publisher to rover
        self.task_pub = self.create_publisher(String, '/rscp/received_task', 10)
        
        # Subscribers from rover
        self.coord_sub = self.create_subscription(String, '/rscp/send_coordinates', self.coord_callback, 10)
        self.dist_sub = self.create_subscription(String, '/rscp/send_distance', self.dist_callback, 10)
        self.ack_sub = self.create_subscription(String, '/rscp/send_ack', self.ack_callback, 10)
        self.comp_sub = self.create_subscription(String, '/rscp/send_task_complete', self.comp_callback, 10)
        
        self.get_logger().info("RSCP Mock Server başlatıldı.")
        
    def coord_callback(self, msg):
        print(f"\n[ROVER'DAN GELDİ] Koordinat: {msg.data}\nSeçiminiz: ", end="", flush=True)
        
    def dist_callback(self, msg):
        print(f"\n[ROVER'DAN GELDİ] Mesafe: {msg.data} metre\nSeçiminiz: ", end="", flush=True)
        
    def ack_callback(self, msg):
        print(f"\n[ROVER'DAN GELDİ] ACK Alındı.\nSeçiminiz: ", end="", flush=True)
        
    def comp_callback(self, msg):
        print(f"\n[ROVER'DAN GELDİ] GÖREV TAMAMLANDI (TaskFinished)!\nSeçiminiz: ", end="", flush=True)

    def publish_json(self, data_dict):
        msg = String()
        msg.data = json.dumps(data_dict)
        self.task_pub.publish(msg)
        print(f"Gönderildi: {msg.data}")

def print_menu():
    print("\n" + "="*40)
    print("--- ARC'26 RSCP SUNUCU SİMÜLATÖRÜ ---")
    print("1. Stage 1: Anten Görevi (Koordinat Sorar + Otomatik ARM Yapar)")
    print("2. Stage 2: Krater Görevi (Koordinat Sorar)")
    print("3. Stage 3: Tünel Görevine Git (Koordinat Sorar)")
    print("4. Stage 4: Airlock Dönüş (Koordinat Sorar)")
    print("5. Özel: Robotu Manuel ARM Et")
    print("6. Özel: Robotu Manuel DISARM Et (Acil Durdurma)")
    print("7. Özel: Keşif Başlat (Stage 3 Tünel İçi İçin)")
    print("0. Çıkış")
    print("="*40)

def cli_thread(node):
    time.sleep(1) # Node un başlaması için bekle
    while rclpy.ok():
        print_menu()
        try:
            choice = input("Seçiminiz: ")
        except EOFError:
            break
            
        if choice == '1':
            print("Örnek Koordinatlar -> Stage 1 Anten: 39.000050, 32.000050")
            lat = input("Enlem (Latitude) [Boş = 0.0]: ")
            lon = input("Boylam (Longitude) [Boş = 0.0]: ")
            r = input("Yarıçap (Radius) [Boş = 10.0]: ")
            try:
                lat_f = float(lat) if lat else 0.0
                lon_f = float(lon) if lon else 0.0
                r_f = float(r) if r else 10.0
                node.publish_json({"type": "set_stage", "stage": 1})
                time.sleep(0.5)
                node.publish_json({"type": "arm_disarm", "arm": True})
                time.sleep(0.5)
                node.publish_json({"type": "search_area", "latitude": lat_f, "longitude": lon_f, "radius": r_f})
            except ValueError:
                print("Hatalı sayı girdiniz!")
                
        elif choice == '2':
            print("Örnek Koordinatlar -> Stage 2 Krater: 39.000100, 32.000100")
            lat = input("Enlem (Latitude) [Boş = 0.0]: ")
            lon = input("Boylam (Longitude) [Boş = 0.0]: ")
            r = input("Yarıçap (Radius) [Boş = 10.0]: ")
            try:
                lat_f = float(lat) if lat else 0.0
                lon_f = float(lon) if lon else 0.0
                r_f = float(r) if r else 10.0
                node.publish_json({"type": "set_stage", "stage": 2})
                time.sleep(0.5)
                node.publish_json({"type": "search_area", "latitude": lat_f, "longitude": lon_f, "radius": r_f})
            except ValueError:
                print("Hatalı sayı girdiniz!")
                
        elif choice == '3':
            print("Örnek Koordinatlar -> Tünel Girişi: 39.000150, 32.000150")
            lat = input("Enlem (Latitude) [Boş = 0.0]: ")
            lon = input("Boylam (Longitude) [Boş = 0.0]: ")
            try:
                lat_f = float(lat) if lat else 0.0
                lon_f = float(lon) if lon else 0.0
                node.publish_json({"type": "set_stage", "stage": 3})
                time.sleep(0.5)
                node.publish_json({"type": "navigate_to_gps", "latitude": lat_f, "longitude": lon_f})
            except ValueError:
                print("Hatalı sayı girdiniz!")
                
        elif choice == '4':
            print("Örnek Koordinatlar -> Hava Kilidi Dönüş: 39.000000, 32.000000")
            lat = input("Enlem (Latitude) [Boş = 0.0]: ")
            lon = input("Boylam (Longitude) [Boş = 0.0]: ")
            try:
                lat_f = float(lat) if lat else 0.0
                lon_f = float(lon) if lon else 0.0
                node.publish_json({"type": "set_stage", "stage": 4})
                time.sleep(0.5)
                node.publish_json({"type": "navigate_to_gps", "latitude": lat_f, "longitude": lon_f})
            except ValueError:
                print("Hatalı sayı girdiniz!")
                
        elif choice == '5':
            node.publish_json({"type": "arm_disarm", "arm": True})
        elif choice == '6':
            node.publish_json({"type": "arm_disarm", "arm": False})
        elif choice == '7':
            node.publish_json({"type": "start_exploration"})
        elif choice == '0':
            print("Çıkılıyor...")
            rclpy.shutdown()
            sys.exit(0)
        else:
            print("Geçersiz seçim!")

def main(args=None):
    rclpy.init(args=args)
    node = RscpMockServer()
    
    # Run CLI in a background thread
    t = threading.Thread(target=cli_thread, args=(node,))
    t.daemon = True
    t.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
ARC'26 Uçtan Uca Seri Port Simülatörü (Hardware-in-the-loop Mock)
Bu script ROS gerektirmez! Doğrudan Python üzerinden sanal bir seri port (PTY) açar,
komutları Protobuf + COBS ile şifreleyerek basar ve Rover'dan gelen şifreli yanıtları çözer.
"""

import os
import pty
import tty
import threading
import time
import sys

# rscp kütüphaneleri (Downloads/rscp-main altından veya pip ile yüklü olmalı)
try:
    import cobs.cobs
    import rscp_protobuf
except ImportError:
    print("HATA: 'cobs' veya 'rscp_protobuf' modülleri bulunamadı.")
    print("Lütfen kurulu olduklarından emin olun.")
    sys.exit(1)

def send_frame(master_fd, req: rscp_protobuf.RequestEnvelope):
    data = req.SerializeToString()
    cobs_encoded = cobs.cobs.encode(data)
    frame = cobs_encoded + b"\x00"
    os.write(master_fd, frame)
    # print(f"\n[SUNUCU] Şifreli Frame Gönderildi ({len(frame)} bayt).")

def send_arm(master_fd, arm: bool):
    req = rscp_protobuf.RequestEnvelope()
    req.arm_disarm.value = arm
    send_frame(master_fd, req)

def send_set_stage(master_fd, stage: int):
    req = rscp_protobuf.RequestEnvelope()
    req.set_stage.value = stage
    send_frame(master_fd, req)

def send_search_area(master_fd, lat: float, lon: float, radius: float):
    req = rscp_protobuf.RequestEnvelope()
    req.search_area.center_coordinate.latitude = lat
    req.search_area.center_coordinate.longitude = lon
    req.search_area.center_coordinate.altitude = 0.0
    req.search_area.radius = radius
    send_frame(master_fd, req)

def send_navigate_to_gps(master_fd, lat: float, lon: float):
    req = rscp_protobuf.RequestEnvelope()
    req.navigate_to_gps.coordinate.latitude = lat
    req.navigate_to_gps.coordinate.longitude = lon
    req.navigate_to_gps.coordinate.altitude = 0.0
    send_frame(master_fd, req)

def send_start_exploration(master_fd):
    req = rscp_protobuf.RequestEnvelope()
    req.start_exploration.CopyFrom(rscp_protobuf.StartExploration())
    send_frame(master_fd, req)

def listen_rover(master_fd):
    """Rover'dan sanal kablo üzerinden gelen COBS paketlerini dinler ve çözer."""
    buffer = b""
    while True:
        try:
            # 1 bayt oku (Daha büyük de okunabilir ama mantık aynı)
            data = os.read(master_fd, 1024)
            if not data:
                time.sleep(0.1)
                continue
                
            for byte in data:
                b = bytes([byte])
                if b == b"\x00":
                    if buffer:
                        decode_and_print(buffer)
                    buffer = b""
                else:
                    buffer += b
        except OSError:
            break
        except Exception as e:
            print(f"\n[Dinleme Hatası]: {e}")

def decode_and_print(raw_data):
    try:
        decoded = cobs.cobs.decode(raw_data)
        res = rscp_protobuf.ResponseEnvelope()
        res.ParseFromString(decoded)
        
        msg_type = res.WhichOneof('response')
        
        # 1 Hz RoverStatus terminali çok dolduracağı için onu sessize alalım veya tek satır basalım
        if msg_type == 'rover_status':
            # print(".", end="", flush=True) # Kalp atışı niyetine nokta basılabilir
            pass
        else:
            print(f"\n\n[ROVER'DAN GELDİ - Şifre Çözüldü!] Tip: {msg_type}")
            if msg_type == 'acknowledge':
                print(">>> ACK Alındı (Görev Onaylandı).")
            elif msg_type == 'task_finished':
                print(">>> GÖREV TAMAMLANDI (TaskFinished).")
            elif msg_type == 'gps_coordinate':
                print(f">>> HEDEF BULUNDU: Enlem={res.gps_coordinate.latitude:.6f}, Boylam={res.gps_coordinate.longitude:.6f}")
            elif msg_type == 'distance':
                print(f">>> MESAFE ÖLÇÜLDÜ: {res.distance:.2f} metre")
            
            print("\nSeçiminiz: ", end="", flush=True)
            
    except Exception as e:
        print(f"\n[Hata - Şifre Çözülemedi]: {e}")

def print_menu():
    print("\n" + "="*50)
    print("--- ARC'26 DONANIMSAL SERİ PORT SİMÜLATÖRÜ ---")
    print("1. Stage 1: Anten Görevi (SetStage1 + Arm + SearchArea)")
    print("2. Stage 2: Krater Görevi (SetStage2 + SearchArea)")
    print("3. Stage 3: Tünel Girişine Git (NavigateToGPS)")
    print("4. Stage 3: Keşif Başlat (StartExploration)")
    print("5. Özel: Robotu Manuel ARM Et")
    print("6. Özel: Robotu Manuel DISARM Et (Acil Durdurma)")
    print("0. Çıkış")
    print("="*50)

def main():
    # Sanal Seri Port (PTY) oluştur
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    
    print("*"*60)
    print("SANAL SERİ PORT BAŞARIYLA OLUŞTURULDU!")
    print(f"Lütfen diğer terminalde rscp_client'ı şu komutla başlatın:")
    print(f"\n   ros2 run robot_2_gps_description rscp_client --ros-args -p serial_port:={slave_name}\n")
    print("*"*60)
    
    # Dinleyici thread'i başlat
    listener = threading.Thread(target=listen_rover, args=(master,), daemon=True)
    listener.start()
    
    time.sleep(1)
    
    while True:
        print_menu()
        try:
            choice = input("Seçiminiz: ")
        except (EOFError, KeyboardInterrupt):
            print("\nÇıkılıyor...")
            break
            
        if choice == '1':
            print("Örnek: 39.000050, 32.000050")
            lat = input("Enlem [Boş = 0.0]: ")
            lon = input("Boylam [Boş = 0.0]: ")
            rad = input("Yarıçap [Boş = 10.0]: ")
            try:
                lat_f = float(lat) if lat else 0.0
                lon_f = float(lon) if lon else 0.0
                rad_f = float(rad) if rad else 10.0
                send_set_stage(master, 1)
                time.sleep(0.5)
                send_arm(master, True)
                time.sleep(0.5)
                send_search_area(master, lat_f, lon_f, rad_f)
                print("[SUNUCU] Aşama 1 komutları şifrelenip gönderildi.")
            except ValueError:
                print("Hatalı girdi.")
                
        elif choice == '2':
            print("Örnek: 39.000100, 32.000100")
            lat = input("Enlem [Boş = 0.0]: ")
            lon = input("Boylam [Boş = 0.0]: ")
            rad = input("Yarıçap [Boş = 10.0]: ")
            try:
                lat_f = float(lat) if lat else 0.0
                lon_f = float(lon) if lon else 0.0
                rad_f = float(rad) if rad else 10.0
                send_set_stage(master, 2)
                time.sleep(0.5)
                send_search_area(master, lat_f, lon_f, rad_f)
                print("[SUNUCU] Aşama 2 komutları şifrelenip gönderildi.")
            except ValueError:
                print("Hatalı girdi.")
                
        elif choice == '3':
            lat = input("Enlem [Boş = 0.0]: ")
            lon = input("Boylam [Boş = 0.0]: ")
            try:
                lat_f = float(lat) if lat else 0.0
                lon_f = float(lon) if lon else 0.0
                send_set_stage(master, 3)
                time.sleep(0.5)
                send_navigate_to_gps(master, lat_f, lon_f)
                print("[SUNUCU] Aşama 3 (Navigasyon) komutları şifrelenip gönderildi.")
            except ValueError:
                print("Hatalı girdi.")
                
        elif choice == '4':
            send_start_exploration(master)
            print("[SUNUCU] Keşif Başlat şifrelenip gönderildi.")
            
        elif choice == '5':
            send_arm(master, True)
            print("[SUNUCU] ARM komutu şifrelenip gönderildi.")
            
        elif choice == '6':
            send_arm(master, False)
            print("[SUNUCU] DISARM (Acil Dur) şifrelenip gönderildi.")
            
        elif choice == '0':
            print("Çıkılıyor...")
            break
            
if __name__ == '__main__':
    main()

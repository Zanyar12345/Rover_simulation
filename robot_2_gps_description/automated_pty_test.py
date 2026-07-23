#!/usr/bin/env python3
import os
import pty
import time
import subprocess
import threading
import sys

try:
    import cobs.cobs
    import rscp_protobuf
except ImportError:
    print("HATA: 'cobs' veya 'rscp_protobuf' modülleri bulunamadı.")
    sys.exit(1)

def listen_rover(master_fd):
    buffer = b""
    while True:
        try:
            data = os.read(master_fd, 1024)
            if not data:
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

def decode_and_print(raw_data):
    try:
        decoded = cobs.cobs.decode(raw_data)
        res = rscp_protobuf.ResponseEnvelope()
        res.ParseFromString(decoded)
        msg_type = res.WhichOneof('response')
        if msg_type != 'rover_status':
            print(f"    [--> ROVER YANIT VERDİ] Tip: {msg_type.upper()}")
    except Exception as e:
        pass

def main():
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    
    print(f"[*] Sanal port oluşturuldu: {slave_name}")
    
    t = threading.Thread(target=listen_rover, args=(master,), daemon=True)
    t.start()
    
    print("[*] ROS düğümleri arka planda başlatılıyor...")
    
    # RSCP Client'ı başlat
    cmd_client = f"source ~/ros2_ws/install/setup.bash && ros2 run robot_2_gps_description rscp_client.py --ros-args -p serial_port:={slave_name}"
    p_client = subprocess.Popen(cmd_client, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # Otonom Kontrolcüyü başlat
    cmd_controller = f"source ~/ros2_ws/install/setup.bash && ros2 run robot_2_gps_description autonomous_mission_controller.py"
    p_controller = subprocess.Popen(cmd_controller, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    print("[*] Düğümlerin ayağa kalkması bekleniyor (4 saniye)...\n")
    time.sleep(4)
    
    def send_frame(req):
        data = req.SerializeToString()
        cobs_encoded = cobs.cobs.encode(data)
        frame = cobs_encoded + b"\x00"
        os.write(master, frame)

    print("--- TEST SENARYOSU: AŞAMA 1 (ANTEN) ---")
    
    req = rscp_protobuf.RequestEnvelope()
    req.set_stage.value = 1
    send_frame(req)
    print("[<-- SUNUCU GÖNDERDİ] SetStage(1)")
    time.sleep(1)
    
    req = rscp_protobuf.RequestEnvelope()
    req.arm_disarm.value = True
    send_frame(req)
    print("[<-- SUNUCU GÖNDERDİ] Arm(True)")
    time.sleep(1)
    
    req = rscp_protobuf.RequestEnvelope()
    req.search_area.center_coordinate.latitude = 39.000050
    req.search_area.center_coordinate.longitude = 32.000050
    req.search_area.center_coordinate.altitude = 0.0
    req.search_area.radius = 10.0
    send_frame(req)
    print("[<-- SUNUCU GÖNDERDİ] SearchArea(39.00005, 32.00005, R=10)")
    
    print("\n[*] Rover'ın tepkileri dinleniyor (5 saniye)...\n")
    time.sleep(5)
    
    print("\n--- TEST BAŞARIYLA BİTTİ ---")
    p_client.terminate()
    p_controller.terminate()

if __name__ == '__main__':
    main()

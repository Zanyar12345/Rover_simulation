#!/usr/bin/env python3
"""
RSCP Client — ARC'26 Resmi Protokol Uygulaması

Protokol: Google Protobuf + COBS encoding + RS-232 seri port
Referans: https://github.com/anatolianroverchallenge/rscp

Gönderme (Rover → CM): ResponseEnvelope → Protobuf serialize → COBS encode → + 0x00 → serial write
Alma (CM → Rover):      serial read → 0x00 delimiter → COBS decode → RequestEnvelope parse
"""

import rclpy
from rclpy.node import Node
import serial
import threading
import time
import json

import cobs.cobs
import rscp_protobuf

from std_msgs.msg import String

class RSCPClient(Node):
    def __init__(self):
        super().__init__('rscp_client')
        
        # Seri port parametreleri (PDF: RS-232, 115200 baud, 1 stop bit, no parity)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        
        # Publisher: Sunucudan gelen görevler (parsed JSON olarak)
        self.task_pub = self.create_publisher(String, '/rscp/received_task', 10)
        
        # Subscribers: Farklı mesaj tipleri için
        self.coord_sub = self.create_subscription(
            String, '/rscp/send_coordinates', self.send_gps_callback, 10
        )
        self.distance_sub = self.create_subscription(
            String, '/rscp/send_distance', self.send_distance_callback, 10
        )
        self.ack_sub = self.create_subscription(
            String, '/rscp/send_ack', self.send_ack_callback, 10
        )
        self.task_complete_sub = self.create_subscription(
            String, '/rscp/send_task_complete', self.send_task_complete_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/rscp/send_status', self.send_status_callback, 10
        )
        
        # Seri port bağlantısı
        self.ser = None
        self.connected = False
        self.serial_lock = threading.Lock()  # Seri port erişimini koruma kilidi
        self.connect_to_serial()
        
        # Dinleme thread'i
        self.listen_thread = threading.Thread(target=self.listen_for_commands, daemon=True)
        self.listen_thread.start()
        
        # Yeniden bağlanma timer'ı (5 saniyede bir)
        self.reconnect_timer = self.create_timer(5.0, self.check_connection)
        
        self.get_logger().info(
            f"RSCP Client başlatıldı (Protobuf + COBS). Port: {self.serial_port}"
        )

    # =========================================================================
    # Seri Port Yönetimi
    # =========================================================================
    
    def connect_to_serial(self):
        """RS-232 seri port bağlantısını aç."""
        with self.serial_lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout
                )
                self.connected = True
                self.get_logger().info(f"Seri port bağlantısı başarılı: {self.serial_port}")
            except serial.SerialException as e:
                self.connected = False
                self.get_logger().error(
                    f"Seri port açılamadı ({self.serial_port}): {e} "
                    f"— Port bağlı mı kontrol edin."
                )
            except Exception as e:
                self.connected = False
                self.get_logger().error(f"Beklenmeyen hata: {e}")

    def check_connection(self):
        """Bağlantı kopmuşsa tekrar bağlanmayı dene."""
        if not self.connected or self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Seri port bağlantısı yok, yeniden deneniyor...")
            self.connect_to_serial()

    # =========================================================================
    # Gönderme (Rover → ARC Sunucu)
    # =========================================================================

    def _send_response(self, response: rscp_protobuf.ResponseEnvelope):
        """ResponseEnvelope'u Protobuf + COBS ile seri porta gönderir."""
        response_bytes = response.SerializeToString()
        cobs_encoded = cobs.cobs.encode(response_bytes)
        frame = cobs_encoded + b"\x00"
        
        with self.serial_lock:
            if self.connected and self.ser and self.ser.is_open:
                try:
                    self.ser.write(frame)
                    self.ser.flush()
                    self.get_logger().info(f"RSCP Gönderildi: {response}")
                except serial.SerialException as e:
                    self.get_logger().error(f"Gönderme hatası: {e}")
                    self.connected = False
            else:
                self.get_logger().warn(
                    f"RSCP [SİMÜLE - port yok]: {response}"
                )

    def send_ack_callback(self, msg):
        """Acknowledge mesajı gönderir (her komut alındığında çağrılmalı)."""
        response = rscp_protobuf.ResponseEnvelope()
        response.acknowledge.CopyFrom(rscp_protobuf.Acknowledge())
        self._send_response(response)
        self.get_logger().info("ACK gönderildi.")

    def send_gps_callback(self, msg):
        """GPS koordinatı gönderir (zirve, kaya koordinatı vb.)
        Beklenen format: 'lat,lon' veya 'lat,lon,alt'
        """
        try:
            parts = msg.data.split(',')
            lat = float(parts[0].strip())
            lon = float(parts[1].strip())
            alt = float(parts[2].strip()) if len(parts) > 2 else 0.0
            
            response = rscp_protobuf.ResponseEnvelope()
            coord = rscp_protobuf.GPSCoordinate()
            coord.latitude = lat
            coord.longitude = lon
            coord.altitude = alt
            response.gps_coordinate.CopyFrom(coord)
            self._send_response(response)
            self.get_logger().info(f"GPS Koordinatı gönderildi: lat={lat}, lon={lon}, alt={alt}")
        except Exception as e:
            self.get_logger().error(f"GPS gönderme hatası: {e} (veri: {msg.data})")

    def send_distance_callback(self, msg):
        """Mesafe/uzunluk gönderir (tünel uzunluğu vb.)
        Beklenen format: '12.50' (metre)
        """
        try:
            distance = float(msg.data.strip())
            response = rscp_protobuf.ResponseEnvelope()
            response.distance = distance
            self._send_response(response)
            self.get_logger().info(f"Mesafe gönderildi: {distance} metre")
        except Exception as e:
            self.get_logger().error(f"Mesafe gönderme hatası: {e} (veri: {msg.data})")

    def send_task_complete_callback(self, msg):
        """TaskFinished mesajı gönderir (bir adım tamamlandığında)."""
        response = rscp_protobuf.ResponseEnvelope()
        response.task_finished.CopyFrom(rscp_protobuf.TaskFinished())
        self._send_response(response)
        self.get_logger().info("TaskFinished gönderildi.")

    def send_status_callback(self, msg):
        """RoverStatus mesajı gönderir (1Hz ile batarya, konum, durum)."""
        try:
            status_data = json.loads(msg.data)
            response = rscp_protobuf.ResponseEnvelope()
            
            rover_status = rscp_protobuf.RoverStatus()
            rover_status.state = status_data.get('state', 0)
            
            coord = status_data.get('coordinate', {})
            rover_status.coordinate.latitude = coord.get('latitude', 0.0)
            rover_status.coordinate.longitude = coord.get('longitude', 0.0)
            rover_status.coordinate.altitude = coord.get('altitude', 0.0)
            
            rover_status.heading = status_data.get('heading', 0.0)
            
            batt = status_data.get('battery_state', {})
            rover_status.battery_state.voltage = batt.get('voltage', 0.0)
            rover_status.battery_state.current = batt.get('current', 0.0)
            rover_status.battery_state.state_of_charge = batt.get('state_of_charge', 0.0)
            
            response.rover_status.CopyFrom(rover_status)
            
            self._send_response(response)
            # Loglamayı kapalı tutalım, 1Hz de terminali spam yapmasın.
            # self.get_logger().debug("RoverStatus gönderildi.")
        except Exception as e:
            self.get_logger().error(f"Status gönderme hatası: {e} (veri: {msg.data})")

    # =========================================================================
    # Alma (ARC Sunucu → Rover)
    # =========================================================================

    def listen_for_commands(self):
        """
        Seri port üzerinden ARC sunucusundan gelen komutları sürekli dinler.
        Mesajlar: SetStage, ArmDisarm, NavigateToGPS, SearchArea, StartExploration
        """
        buffer = b""
        
        while rclpy.ok():
            if not self.connected or self.ser is None or not self.ser.is_open:
                time.sleep(1.0)
                continue
            
            try:
                with self.serial_lock:
                    if not self.connected or self.ser is None or not self.ser.is_open:
                        continue
                    data = self.ser.read(1)
                if not data:
                    continue
                    
                if data == b"\x00":
                    # Mesaj tamamlandı — çöz ve işle
                    if buffer:
                        self._process_incoming(buffer)
                    buffer = b""
                else:
                    buffer += data
                    
            except serial.SerialException as e:
                self.get_logger().error(f"Okuma hatası: {e}")
                self.connected = False
                buffer = b""
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f"Dinleme hatası: {e}")
                buffer = b""
                time.sleep(0.5)

    def _process_incoming(self, raw_data: bytes):
        """COBS decode → Protobuf parse → ROS topic'e yayınla."""
        try:
            cobs_decoded = cobs.cobs.decode(raw_data)
            request = rscp_protobuf.RequestEnvelope()
            request.ParseFromString(cobs_decoded)
            
            msg_type = request.WhichOneof('request')
            self.get_logger().info(f"RSCP Alındı — Tip: {msg_type}")
            
            # JSON formatında ROS topic'e yayınla
            task_data = {'type': msg_type}
            
            if msg_type == 'set_stage':
                task_data['stage'] = request.set_stage.value
                self.get_logger().info(f"  Stage: {request.set_stage.value}")
                
            elif msg_type == 'arm_disarm':
                task_data['arm'] = request.arm_disarm.value
                self.get_logger().info(f"  Arm: {request.arm_disarm.value}")
                
            elif msg_type == 'navigate_to_gps':
                coord = request.navigate_to_gps.coordinate
                task_data['latitude'] = coord.latitude
                task_data['longitude'] = coord.longitude
                task_data['altitude'] = coord.altitude
                self.get_logger().info(
                    f"  GPS Hedefi: lat={coord.latitude}, lon={coord.longitude}"
                )
                
            elif msg_type == 'search_area':
                center = request.search_area.center_coordinate
                task_data['latitude'] = center.latitude
                task_data['longitude'] = center.longitude
                task_data['radius'] = request.search_area.radius
                self.get_logger().info(
                    f"  Arama Alanı: lat={center.latitude}, lon={center.longitude}, "
                    f"r={request.search_area.radius}m"
                )
                
            elif msg_type == 'start_exploration':
                self.get_logger().info("  Keşif başlatılıyor!")
            
            # ROS topic'e JSON olarak yayınla
            task_msg = String()
            task_msg.data = json.dumps(task_data)
            self.task_pub.publish(task_msg)
            
            # Her alınan komuta otomatik ACK gönder
            self.send_ack_callback(None)
            
        except cobs.cobs.DecodeError as e:
            self.get_logger().error(f"COBS decode hatası: {e}")
        except Exception as e:
            self.get_logger().error(f"Mesaj işleme hatası: {e}")

    # =========================================================================
    # Cleanup
    # =========================================================================

    def destroy_node(self):
        """Node kapatılırken seri portu temiz kapat."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Seri port kapatıldı.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RSCPClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

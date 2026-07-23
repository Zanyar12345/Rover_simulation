#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math

from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import String, Float32, Float32MultiArray
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Image, CameraInfo

class AutonomousMissionTest(Node):
    def __init__(self):
        super().__init__('autonomous_mission_test')
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Service clients for Peak Finder
        self.start_scan_client = self.create_client(Trigger, '/start_peak_scan')
        self.stop_scan_client = self.create_client(Trigger, '/stop_peak_scan')
        self.scan_future = None
        
        # Publishers
        self.rscp_gps_pub = self.create_publisher(String, '/rscp/send_coordinates', 10)
        self.rscp_distance_pub = self.create_publisher(String, '/rscp/send_distance', 10)
        self.rscp_ack_pub = self.create_publisher(String, '/rscp/send_ack', 10)
        self.rscp_task_complete_pub = self.create_publisher(String, '/rscp/send_task_complete', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber: ARC sunucusundan gelen görevler (RSCP client JSON olarak yayınlar)
        self.rscp_task_sub = self.create_subscription(String, '/rscp/received_task', self.rscp_task_callback, 10)
        
        # Subscribers (Vision)
        self.aruco_sub = self.create_subscription(Point, '/vision/aruco_position', self.aruco_callback, 10)
        self.rock_sub = self.create_subscription(Float32MultiArray, '/vision/ilmenite_rock_position', self.rock_callback, 10)
        
        # Height / Odom calculator subscriber
        self.roof_length_sub = self.create_subscription(Float32, '/measured_roof_length', self.roof_callback, 10)
        
        # Odometry subscriber to track rotation
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered_map', self.odom_callback, 10)
        
        # GPS Subscriber for geographical navigation
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        
        # Depth Subscriber for Tunnel Navigation
        self.depth_sub = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, 10)
        
        # Camera Info Subscriber for dynamic resolution adaptation
        self.camera_info_sub = self.create_subscription(CameraInfo, '/zed/zed_node/rgb/camera_info', self.camera_info_callback, 10)
        self.camera_c_x = 640.0
        self.camera_f_x = 896.0
        self.image_center_x = 640.0
        self.tunnel_angular_z = 0.0
        self.tunnel_linear_x = 0.25
        
        self.origin_lat = None
        self.origin_lon = None
        self.current_lat = None
        self.current_lon = None
        
        self.current_state = 'WAITING_FOR_SERVER'
        self.mission_stage = 0
        self.target_nav_lat = 0.0
        self.target_nav_lon = 0.0
        self.send_task_complete_after_nav = False
        # GPS koordinatları artık doğrudan RSCP üzerinden dinamik olarak gelmektedir.
        
        self.aruco_detected = False
        self.rock_detected = False
        self.detected_rocks = []
        self.measured_length = 0.0
        self.wait_counter = 0
        
        # Odom tracking for tunnel
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.tube_start_x = None
        self.tube_start_y = None
        
        # Yaw tracking
        self.current_yaw = 0.0
        self.accumulated_yaw = 0.0
        self.last_yaw = None
        
        # Nav retry tracking
        self.nav_retry_count = 0
        self.current_nav_x = 0.0
        self.current_nav_y = 0.0
        self.start_wait_count = 0
        
        self.get_logger().info("Autonomous Mission Controller initialized.")
        
        # Start mission loop in a timer to not block
        self.timer = self.create_timer(1.0, self.mission_loop)

    def aruco_callback(self, msg):
        self.aruco_detected = True
        self.aruco_x = msg.x  # ArUco merkez X pikseli (Visual Servoing için)

    def rock_callback(self, msg):
        if self.current_state == 'LOCATE_ROCK':
            self.rock_detected = True
            
            cx = msg.data[0]
            cy = msg.data[1]
            depth = msg.data[2]
            conf = msg.data[3]
            
            if hasattr(self, 'current_lat') and hasattr(self, 'current_lon') and self.current_lat is not None:
                d = depth if depth > 0.0 else 1.5
                c_x = self.camera_c_x
                f_x = self.camera_f_x
                x_cam = (cx - c_x) * d / f_x
                x_rover = d
                y_rover = -x_cam
                yaw = self.current_yaw
                delta_x_map = x_rover * math.cos(yaw) - y_rover * math.sin(yaw)
                delta_y_map = x_rover * math.sin(yaw) + y_rover * math.cos(yaw)
                delta_lat = delta_y_map / 111139.0
                delta_lon = delta_x_map / (111139.0 * math.cos(math.radians(self.current_lat)))
                
                rock_lat = self.current_lat + delta_lat
                rock_lon = self.current_lon + delta_lon
                
                self.detected_rocks.append({
                    'lat': rock_lat,
                    'lon': rock_lon,
                    'conf': conf
                })
        
    def roof_callback(self, msg):
        self.measured_length = msg.data

    def camera_info_callback(self, msg):
        self.camera_c_x = msg.k[2]
        self.camera_f_x = msg.k[0]
        self.image_center_x = msg.width / 2.0

    def odom_callback(self, msg):
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if self.current_state in ['SPINNING', 'LOCATE_ROCK', 'SEARCH_AIRLOCK_ARUCO', 'ENTER_LAVA_TUBE']:
            if self.last_yaw is not None:
                delta_yaw = yaw - self.last_yaw
                delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw)) # normalize [-pi, pi]
                self.accumulated_yaw += abs(delta_yaw)
            self.last_yaw = yaw
            
        self.current_yaw = yaw

    def gps_callback(self, msg):
        if math.isnan(msg.latitude) or msg.latitude == 0.0:
            return
            
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # İlk geçerli veriyi orijin (0,0 noktası) kabul et
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"GPS Orijini ayarlandı: Enlem {self.origin_lat}, Boylam {self.origin_lon}")

    def depth_callback(self, msg):
        """Tünel içinde (Lava Tube) merkezde kalmak için derinlik kamerasını kullanır."""
        if self.current_state not in ['ENTER_LAVA_TUBE', 'MEASURE_ROOF']:
            return
            
        try:
            if not hasattr(self, 'bridge'):
                from cv_bridge import CvBridge
                self.bridge = CvBridge()
                import numpy as np
                self.np = np
                
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # ZED 2 depth image can contain NaNs or Infs.
            h, w = depth_image.shape
            
            # Orta kısımdaki yatay şeridi al (kamera açısına göre yer/gökyüzü parazitini önlemek için)
            strip = depth_image[h//2 - 20 : h//2 + 20, :]
            
            # TAVAN (ÇATI) KONTROLÜ: Görüntünün üstte kalan 1/3'lük dilimine bak. 
            # Yandaki yüksek duvarları tavan sanmaması için bölgeyi daha da inceltip TAM ORTA (%20'lik kısım) alıyoruz.
            roof_strip = depth_image[0 : h//3, w//2 - w//10 : w//2 + w//10]
            # Gökyüzü değilse (NaN, Inf değil) ve üstümüzdeki tavan 3.5 metreden yakınsa çatı say!
            roof_valid = roof_strip[(roof_strip > 0.0) & (roof_strip < 3.5) & (~self.np.isnan(roof_strip)) & (~self.np.isinf(roof_strip))]
            # Üstteki devasa bölgenin en az %2'sini kaplıyorsa çatının altındayız
            self.under_roof = len(roof_valid) > (roof_strip.size * 0.02)
            
            # Sol, Orta ve Sağ bölgeleri daraltıyoruz (Dar tünel için)
            left_part = strip[:, :w//3]       # Sol %33
            right_part = strip[:, 2*w//3:]    # Sağ %33
            center_part = strip[:, w//2 - w//8 : w//2 + w//8] # Sadece tam önümüz (orta %25) - Duvarları engel sanmasın
            
            left_valid = left_part[(left_part > 0) & (~self.np.isnan(left_part)) & (~self.np.isinf(left_part))]
            right_valid = right_part[(right_part > 0) & (~self.np.isnan(right_part)) & (~self.np.isinf(right_part))]
            center_valid = center_part[(center_part > 0) & (~self.np.isnan(center_part)) & (~self.np.isinf(center_part))]
            
            # Ortalama mesafeleri hesapla (Duvar çok uzaksa 5.0 metre varsay)
            left_dist = float(self.np.nanmean(left_valid)) if len(left_valid) > 0 else 5.0
            right_dist = float(self.np.nanmean(right_valid)) if len(right_valid) > 0 else 5.0
            center_dist = float(self.np.nanmean(center_valid)) if len(center_valid) > 0 else 5.0
            
            # Merkezleme hatası (Sol uzaksa sola dön (+), sağ uzaksa sağa dön (-))
            error = left_dist - right_dist
            
            # P-Controller (Oransal Kontrol)
            Kp = 0.5
            self.tunnel_angular_z = float(self.np.clip(Kp * error, -0.4, 0.4))
            
            # Çarpışma önleyici (Eğimli ve dar yollar için mesafe 0.45'e düşürüldü)
            if center_dist < 0.45:
                self.tunnel_linear_x = 0.0 # Çok yakında engel var, dur
            else:
                self.tunnel_linear_x = 0.25 # Normal hız
                
        except Exception as e:
            self.get_logger().error(f"Tünel navigasyonu depth hatası: {e}")
            self.tunnel_angular_z = 0.0
            self.tunnel_linear_x = 0.0

    def gps_to_local(self, target_lat, target_lon):
        # TEST MODU: Gelen değerler GPS değil, direkt Nav2 X, Y metre hedefleridir.
        return target_lat, target_lon

    def local_to_gps(self, east, north):
        # TEST MODU: Geri dönüşüm iptal. Direk metre gönderilir.
        return east, north

    def send_gps_goal(self, target_lat, target_lon):
        x, y = self.gps_to_local(target_lat, target_lon)
        self.get_logger().info(f"GPS Hedefi {target_lat}, {target_lon} -> Metreye çevrildi: X={x:.2f}, Y={y:.2f}")
        self.send_nav_goal(x, y)

    def send_nav_goal(self, x, y):
        self.current_nav_x = x
        self.current_nav_y = y
        self.get_logger().info(f"Sending Nav2 goal: ({x}, {y})")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 # No rotation
        
        self.nav_client.wait_for_server()
        self.nav_future = self.nav_client.send_goal_async(goal_msg)

    def send_rscp_gps(self, lat, lon, alt=0.0):
        """RSCP üzerinden GPS koordinatı gönderir (Protobuf GPSCoordinate)."""
        msg = String()
        msg.data = f"{lat:.6f},{lon:.6f},{alt:.2f}"
        self.rscp_gps_pub.publish(msg)
        self.get_logger().info(f"RSCP GPS gönderildi: lat={lat:.6f}, lon={lon:.6f}")

    def send_rscp_distance(self, distance_m):
        """RSCP üzerinden mesafe gönderir (Protobuf distance)."""
        msg = String()
        msg.data = f"{distance_m:.2f}"
        self.rscp_distance_pub.publish(msg)
        self.get_logger().info(f"RSCP Mesafe gönderildi: {distance_m:.2f} m")

    def send_rscp_ack(self):
        """RSCP Acknowledge gönderir."""
        msg = String()
        msg.data = "ACK"
        self.rscp_ack_pub.publish(msg)

    def send_rscp_task_complete(self):
        """RSCP TaskFinished gönderir (bir stage tamamlandığında)."""
        msg = String()
        msg.data = "COMPLETE"
        self.rscp_task_complete_pub.publish(msg)
        self.get_logger().info("RSCP TaskFinished gönderildi.")

    def rscp_task_callback(self, msg):
        """ARC sunucusundan gelen komutları işler."""
        import json
        try:
            task = json.loads(msg.data)
            msg_type = task.get('type', '')
            self.get_logger().info(f"RSCP Komut alındı: {msg_type} → {task}")
            
            if msg_type == 'set_stage':
                stage = task.get('stage', 0)
                self.mission_stage = stage
                self.get_logger().info(f"Stage ayarlandı: {stage}")
                
            elif msg_type == 'arm_disarm':
                armed = task.get('arm', False)
                if armed:
                    self.get_logger().info("Rover ARM edildi — Otonom mod aktif!")
                else:
                    self.get_logger().info("Rover DISARM edildi — Acil Durdurma!")
                    # Motorları durdur
                    stop_twist = Twist()
                    self.cmd_vel_pub.publish(stop_twist)
                    # Dönme modunu kapat
                    mode_msg = String()
                    mode_msg.data = "NAV"
                    self.mode_pub.publish(mode_msg)
                    # Aktif Nav2 hedefini iptal et
                    if hasattr(self, 'nav_future') and self.nav_future is not None and self.nav_future.done():
                        try:
                            self.nav_future.result().cancel_goal_async()
                            self.get_logger().info("Nav2 hedefi iptal edildi.")
                        except Exception as e:
                            self.get_logger().warn(f"Nav2 iptal hatası: {e}")
                    self.current_state = 'DONE'
                    
            elif msg_type == 'navigate_to_gps':
                lat = task.get('latitude', 0.0)
                lon = task.get('longitude', 0.0)
                
                self.target_nav_lat = lat
                self.target_nav_lon = lon
                self.get_logger().info(f"Sunucudan GPS hedefi alındı: {lat}, {lon}")
                
                if self.mission_stage == 3:
                    self.current_state = 'REACH_LAVA_TUBE'
                elif self.mission_stage == 4:
                    self.current_state = 'RETURN_AIRLOCK'
                
            elif msg_type == 'search_area':
                lat = task.get('latitude', 0.0)
                lon = task.get('longitude', 0.0)
                radius = task.get('radius', 10.0)
                
                self.target_nav_lat = lat
                self.target_nav_lon = lon
                self.get_logger().info(f"Arama alanı alındı: ({lat}, {lon}), r={radius}m")
                
                if self.mission_stage == 1:
                    self.current_state = 'REACH_ANTENNA_AREA'
                elif self.mission_stage == 2:
                    self.current_state = 'REACH_CRATER'
                
            elif msg_type == 'start_exploration':
                self.get_logger().info("Keşif başlatma komutu alındı!")
                if self.mission_stage == 3:
                    self.current_state = 'ENTER_LAVA_TUBE'
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"RSCP JSON parse hatası: {e}")

    def publish_spin_cmd(self, activate=True):
        mode_msg = String()
        twist_msg = Twist()
        
        if activate:
            mode_msg.data = "SPIN"
            twist_msg.angular.z = 0.2
        else:
            mode_msg.data = "NAV"
            twist_msg.angular.z = 0.0 
            
        self.mode_pub.publish(mode_msg)
        self.cmd_vel_pub.publish(twist_msg)

    def mission_loop(self):
        if self.current_state == 'WAITING_FOR_SERVER':
            # Sadece sunucudan komut bekliyor
            pass
            
        elif self.current_state == 'WAIT_NAV_ACCEPT':
            if self.nav_future.done():
                goal_handle = self.nav_future.result()
                if not goal_handle.accepted:
                    self.nav_retry_count += 1
                    if self.nav_retry_count <= 5:
                        self.get_logger().error(f'Navigasyon hedefi reddedildi! Nav2 hazır olmayabilir. Tekrar deneniyor... ({self.nav_retry_count}/5)')
                        self.send_nav_goal(self.current_nav_x, self.current_nav_y)
                    else:
                        self.get_logger().error('Navigasyon hedefi defalarca reddedildi! Adım atlanıyor...')
                        self.nav_retry_count = 0
                        self.current_state = getattr(self, 'next_state_after_nav', 'WAITING_FOR_SERVER')
                else:
                    self.nav_retry_count = 0
                    self.get_logger().info('Navigasyon hedefi kabul edildi, yola çıkıldı...')
                    self.nav_result_future = goal_handle.get_result_async()
                    self.current_state = 'WAIT_NAV_RESULT'

        elif self.current_state == 'WAIT_NAV_RESULT':
            # Eğer hava kilidine dönüş aşamasındaysak ve ArUco görürsek navigasyonu kes.
            if self.mission_stage == 4 and self.aruco_detected:
                self.get_logger().info("Airlock ArUco tespit edildi! Navigasyon iptal ediliyor, hava kilidine manuel giriliyor.")
                if hasattr(self, 'nav_future') and self.nav_future.done():
                    try:
                        self.nav_future.result().cancel_goal_async()
                    except Exception as e:
                        self.get_logger().warn(f"Goal iptal hatası: {e}")
                
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.current_state = 'ENTER_AIRLOCK'
                self.wait_counter = 0
                return

            if self.nav_result_future.done():
                self.get_logger().info('Hedefe başarıyla varıldı!')
                if hasattr(self, 'send_task_complete_after_nav') and self.send_task_complete_after_nav:
                    self.send_rscp_task_complete()
                    self.send_task_complete_after_nav = False
                self.current_state = getattr(self, 'next_state_after_nav', 'WAITING_FOR_SERVER')

        elif self.current_state == 'REACH_ANTENNA_AREA':
            self.get_logger().info("Step 3: Reaching Antenna Area...")
            self.send_gps_goal(self.target_nav_lat, self.target_nav_lon)
            self.next_state_after_nav = 'START_PEAK_SCAN'
            self.current_state = 'WAIT_NAV_ACCEPT'
            
        elif self.current_state == 'START_PEAK_SCAN':
            self.get_logger().info("Yükseklik algılayıcıları açılıyor...")
            if self.start_scan_client.wait_for_service(timeout_sec=1.0):
                self.scan_future = self.start_scan_client.call_async(Trigger.Request())
                self.current_state = 'WAIT_START_SCAN'
            else:
                self.get_logger().warn("Height scanner service not available, skipping.")
                self.current_state = 'INSTALL_ANTENNA'
                
        elif self.current_state == 'WAIT_START_SCAN':
            if self.scan_future.done():
                self.get_logger().info("Algılayıcılar açıldı. Kendi etrafında dönme başlatılıyor (360 derece dönüş)...")
                self.publish_spin_cmd(activate=True)
                self.accumulated_yaw = 0.0
                self.last_yaw = self.current_yaw
                self.current_state = 'SPINNING'
                
        elif self.current_state == 'SPINNING':
            
            self.publish_spin_cmd(activate=True)
            
            if self.accumulated_yaw >= 2 * math.pi: # 360 degrees = 2 * pi radians
                self.get_logger().info("360 derece dönüldü. Dönme durduruluyor ve algılayıcı kapatılıyor...")
                self.publish_spin_cmd(activate=False)
                self.scan_future = self.stop_scan_client.call_async(Trigger.Request())
                self.current_state = 'WAIT_STOP_SCAN'
                
        elif self.current_state == 'WAIT_STOP_SCAN':
            if self.scan_future.done():
                try:
                    response = self.scan_future.result()
                    peak_coords = response.message
                    
                    x_str, y_str = peak_coords.split(',')
                    x = float(x_str)
                    y = float(y_str)
                    
                    # Yerel (x,y) metre koordinatını GPS (lat,lon)'a çevir
                    peak_lat, peak_lon = self.local_to_gps(x, y)
                    
                    self.get_logger().info(
                        f"Zirve bulundu: Yerel=({x:.2f}, {y:.2f}) → GPS=({peak_lat:.6f}, {peak_lon:.6f})"
                    )
                    self.send_rscp_gps(peak_lat, peak_lon)
                    
                    self.get_logger().info("Tepe noktasına doğru hareket ediliyor...")
                    self.send_nav_goal(x, y)
                    self.next_state_after_nav = 'INSTALL_ANTENNA'
                    self.current_state = 'WAIT_NAV_ACCEPT'
                    
                except Exception as e:
                    self.get_logger().error(f"Failed to get peak: {e}")
                    self.current_state = 'INSTALL_ANTENNA'
            
        elif self.current_state == 'INSTALL_ANTENNA':
            if not hasattr(self, 'wait_counter'): self.wait_counter = 0
            if self.wait_counter == 0:
                self.get_logger().info("Step 5: Installing Antenna...")
                self.wait_counter = 2 # 2 saniye bekle
            else:
                self.wait_counter -= 1
                if self.wait_counter <= 0:
                    self.send_rscp_task_complete()
                    self.current_state = 'WAITING_FOR_SERVER'
                    self.wait_counter = 0
            
        elif self.current_state == 'REACH_CRATER':
            self.get_logger().info("Step 6: Reaching Shackleton Crater...")
            self.send_gps_goal(self.target_nav_lat, self.target_nav_lon)
            self.next_state_after_nav = 'LOCATE_ROCK'
            self.current_state = 'WAIT_NAV_ACCEPT'
            
        elif self.current_state == 'LOCATE_ROCK':
            if not hasattr(self, 'wait_counter'): self.wait_counter = 0
            if self.wait_counter == 0:
                self.get_logger().info("Step 7: Locating Ilmenite Rock (360 derece tarama)...")
                self.detected_rocks = []
                self.accumulated_yaw = 0.0
                self.last_yaw = self.current_yaw
                self.publish_spin_cmd(activate=True)
                self.wait_counter = 60 # 60 saniye timeout (tam tur dönmesi için güvenli süre)
            else:
                self.wait_counter -= 1
                self.publish_spin_cmd(activate=True)
                
                if self.accumulated_yaw >= 2 * math.pi or self.wait_counter <= 0:
                    self.publish_spin_cmd(activate=False)
                    if self.accumulated_yaw >= 2 * math.pi:
                        self.get_logger().info("360 derece tarama tamamlandı.")
                    else:
                        self.get_logger().warn("Tarama süresi doldu (Timeout).")
                        
                    if len(self.detected_rocks) > 0:
                        # Find the best rock
                        best_rock = max(self.detected_rocks, key=lambda x: x['conf'])
                        self.get_logger().info(f"{len(self.detected_rocks)} adet İlmenit tespit edildi! En iyisi seçildi (Conf: {best_rock['conf']:.2f})")
                        self.send_rscp_gps(best_rock['lat'], best_rock['lon'])
                    else:
                        self.get_logger().warn("Hiç İlmenit bulunamadı! Mevcut koordinat yollanıyor.")
                        if hasattr(self, 'current_lat') and hasattr(self, 'current_lon') and self.current_lat is not None:
                            self.send_rscp_gps(self.current_lat, self.current_lon)
                        else:
                            self.get_logger().error("GPS verisi yok! Koordinat gönderilemedi.")
                            
                    self.send_rscp_task_complete()
                    self.current_state = 'WAITING_FOR_SERVER'
                    self.wait_counter = 0
            
        elif self.current_state == 'REACH_LAVA_TUBE':
            self.get_logger().info("Step 8: Reaching Lava Tube Entrance...")
            self.send_gps_goal(self.target_nav_lat, self.target_nav_lon)
            self.send_task_complete_after_nav = True
            self.next_state_after_nav = 'WAITING_FOR_SERVER'
            self.current_state = 'WAIT_NAV_ACCEPT'
            
        elif self.current_state == 'ENTER_LAVA_TUBE':
            if not hasattr(self, 'wait_counter'): self.wait_counter = 0
            if self.wait_counter == 0:
                self.get_logger().info("Step 9: Entering Lava Tube (Looking for ArUco)...")
                self.wait_counter = 60 # 60 saniye arama / hizalanma süresi
            else:
                self.wait_counter -= 1
                
                if self.aruco_detected:
                    # ArUco'yu dinamik olarak ekranın ortasına hizala (Çözünürlüğe göre otomatik)
                    error = self.image_center_x - getattr(self, 'aruco_x', self.image_center_x)
                    
                    if abs(error) < 40: # 40 piksel toleransla ortalandı
                        self.get_logger().info("Lava Tube kapısı ortalandı! İçeri giriliyor...")
                        self.publish_spin_cmd(activate=False)
                        
                        # ArUco'yu gördüğümüz (ve hizalandığımız) anı tüpün girişi kabul et
                        self.tube_start_x = self.current_odom_x
                        self.tube_start_y = self.current_odom_y
                        
                        self.current_state = 'MEASURE_ROOF'
                        self.wait_counter = 0
                        self.aruco_detected = False
                    else:
                        # Yavaşça dönerek ortala
                        twist = Twist()
                        twist.angular.z = 0.001 * error
                        twist.angular.z = max(-0.3, min(0.3, twist.angular.z)) # Sınırla
                        self.cmd_vel_pub.publish(twist)
                else:
                    self.publish_spin_cmd(activate=True)
                    if self.wait_counter <= 0:
                        self.get_logger().warn("Lava Tube ArUco bulunamadı! Dümdüz içeri giriliyor.")
                        self.publish_spin_cmd(activate=False)
                        self.tube_start_x = self.current_odom_x
                        self.tube_start_y = self.current_odom_y
                        self.aruco_detected = False
                        self.current_state = 'MEASURE_ROOF'
                        self.wait_counter = 0
            
        elif self.current_state == 'MEASURE_ROOF':
            if not hasattr(self, 'wait_counter'): self.wait_counter = 0
            if self.wait_counter == 0:
                self.get_logger().info("Step 10: Driving and measuring roof length. Waiting for exit ArUco...")
                self.wait_counter = 60 # Tünelde maksimum 60 saniye ilerle (Timeout güvenlik için)
            else:
                self.wait_counter -= 1
                
                # Tünel içinde depth kamerasından gelen verilerle ortalanarak ilerle
                twist = Twist()
                twist.linear.x = getattr(self, 'tunnel_linear_x', 0.25)
                twist.angular.z = getattr(self, 'tunnel_angular_z', 0.0)
                self.cmd_vel_pub.publish(twist)
                
                # Sadece çatı altındayken ilerlemeyi adım adım toplayarak (Integral) ölçüm yap
                if not hasattr(self, 'last_odom_x'):
                    self.last_odom_x = self.current_odom_x
                    self.last_odom_y = self.current_odom_y
                    self.measured_length = 0.0

                dx = self.current_odom_x - self.last_odom_x
                dy = self.current_odom_y - self.last_odom_y
                step_dist = math.sqrt(dx**2 + dy**2)
                
                if getattr(self, 'under_roof', False):
                    self.measured_length += step_dist
                
                # Bir sonraki adım için son konumu kaydet (Böylece çatı olmayan yerden geçerken mesafe sıçramaz)
                self.last_odom_x = self.current_odom_x
                self.last_odom_y = self.current_odom_y
                
                if self.wait_counter % 5 == 0:
                    status = "ÇATI ALTINDA (Ölçülüyor)" if getattr(self, 'under_roof', False) else "AÇIK GÖKYÜZÜ (Ölçülmüyor)"
                    self.get_logger().info(f"Tünel: {status} | Ölçülen Karanlık Kısım: {self.measured_length:.2f} m")
                
                # ÇIKIŞ KONTROLÜ: Eğer kamera çıkıştaki ArUco'yu görürse veya süre biterse çık!
                if self.aruco_detected or self.wait_counter <= 0:
                    if self.aruco_detected:
                        self.get_logger().info("Çıkış ArUco'su görüldü! Tünel Bitti.")
                    else:
                        self.get_logger().warn("Süre doldu (Timeout). Tünelden çıkıldığı varsayılıyor.")
                        
                    # Tünelden çıkış anı (Tekerlekleri durdur)
                    twist.linear.x = 0.0
                    self.cmd_vel_pub.publish(twist)
                    
                    self.current_state = 'EXIT_LAVA_TUBE'
                    self.wait_counter = 0
            
        elif self.current_state == 'EXIT_LAVA_TUBE':
            if not hasattr(self, 'wait_counter'): self.wait_counter = 0
            if self.wait_counter == 0:
                self.get_logger().info(f"Step 11: Exiting Lava Tube. Final Length: {self.measured_length:.2f} meters")
                self.send_rscp_distance(self.measured_length)
                self.wait_counter = 2
            else:
                self.wait_counter -= 1
                if self.wait_counter <= 0:
                    self.send_rscp_task_complete()
                    self.current_state = 'WAITING_FOR_SERVER'
                    self.wait_counter = 0
            
        elif self.current_state == 'RETURN_AIRLOCK':
            self.get_logger().info("Step 12 & 13: Returning to Airlock...")
            self.send_gps_goal(self.target_nav_lat, self.target_nav_lon)
            # RSCP GitHub dokümantasyonuna göre TaskCompleted navigasyon bitince yollanmalı
            self.send_task_complete_after_nav = True
            self.next_state_after_nav = 'SEARCH_AIRLOCK_ARUCO'
            self.aruco_detected = False
            self.current_state = 'WAIT_NAV_ACCEPT'
            
        elif self.current_state == 'SEARCH_AIRLOCK_ARUCO':
            if not hasattr(self, 'wait_counter'): self.wait_counter = 0
            if self.wait_counter == 0:
                self.get_logger().info("Airlock GPS hedefine varıldı. ArUco aranıyor (kendi etrafında dönüş)...")
                self.accumulated_yaw = 0.0
                self.last_yaw = self.current_yaw
                self.publish_spin_cmd(activate=True)
                self.wait_counter = 60 # 60 sn
            else:
                self.wait_counter -= 1
                
                if self.aruco_detected:
                    # ArUco'yu dinamik olarak ekranın ortasına hizala (Çözünürlüğe göre otomatik)
                    error = self.image_center_x - getattr(self, 'aruco_x', self.image_center_x)
                    
                    if abs(error) < 40: # 40 piksel toleransla ortalandı
                        self.get_logger().info("Airlock kapısı ortalandı! İçeri giriliyor...")
                        self.publish_spin_cmd(activate=False)
                        self.current_state = 'ENTER_AIRLOCK'
                        self.wait_counter = 0
                        self.aruco_detected = False
                    else:
                        # Yavaşça dönerek ortala
                        twist = Twist()
                        twist.angular.z = 0.001 * error
                        twist.angular.z = max(-0.3, min(0.3, twist.angular.z)) # Sınırla
                        self.cmd_vel_pub.publish(twist)
                else:
                    self.publish_spin_cmd(activate=True)
                    if self.accumulated_yaw >= 2 * math.pi or self.wait_counter <= 0:
                        self.publish_spin_cmd(activate=False)
                        self.get_logger().warn("Airlock ArUco bulunamadı! Yine de dümdüz içeri girildiği varsayılıyor.")
                        self.current_state = 'ENTER_AIRLOCK'
                        self.wait_counter = 0

        elif self.current_state == 'ENTER_AIRLOCK':
            if not hasattr(self, 'wait_counter'): self.wait_counter = 0
            if self.wait_counter == 0:
                self.get_logger().info("Step 13: Entering Airlock...")
                self.wait_counter = 10 # 10 saniye boyunca ileri sür
            else:
                self.wait_counter -= 1
                
                # İleri sür
                twist = Twist()
                twist.linear.x = 0.3
                self.cmd_vel_pub.publish(twist)
                
                if self.wait_counter <= 0:
                    self.get_logger().info("Airlock'a girildi! Görev tamamlandı, sunucudan DISARM bekleniyor...")
                    twist.linear.x = 0.0
                    self.cmd_vel_pub.publish(twist)
                    self.current_state = 'WAITING_FOR_SERVER'
                    self.wait_counter = 0
            
        elif self.current_state == 'DONE':
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMissionTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class VisionPerceptionNode(Node):
    def __init__(self):
        super().__init__('vision_perception_node')
        
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/color/rect/image',
            self.image_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10
        )
        self.latest_depth_image = None
        
        # Publishers
        self.aruco_pub = self.create_publisher(Point, '/vision/aruco_position', 10)
        self.rock_pub = self.create_publisher(Float32MultiArray, '/vision/ilmenite_rock_position', 10)
        self.annotated_pub = self.create_publisher(Image, '/vision/annotated_image', 10)
        
        # ArUco Dictionary (ARC resmi örneğine göre DICT_ARUCO_ORIGINAL kullanılmalı)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.get_logger().info("Vision Perception Node initialized.")

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        # Draw on a copy of the image so we can publish it
        annotated_image = cv_image.copy()
            
        self.detect_aruco(annotated_image)
        self.detect_ilmenite_rock(annotated_image)
        
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

    def detect_aruco(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            sum_x = 0
            sum_y = 0
            for i in range(len(ids)):
                # Calculate center of the marker
                c = corners[i][0]
                center_x = int((c[0][0] + c[2][0]) / 2)
                center_y = int((c[0][1] + c[2][1]) / 2)
                sum_x += center_x
                sum_y += center_y
                self.get_logger().info(f"Detected ArUco ID: {ids[i][0]} at ({center_x}, {center_y})")
                
            avg_x = sum_x / len(ids)
            avg_y = sum_y / len(ids)
                
            # Publish the position (in pixels for now, or relative angles)
            pt = Point()
            pt.x = float(avg_x)
            pt.y = float(avg_y)
            pt.z = float(len(ids)) # Number of markers detected
            self.aruco_pub.publish(pt)

    def detect_ilmenite_rock(self, cv_image):
        # 1. Görüntüyü HSV formatına çevir
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 2. İlmenit kayaları genelde çok koyu (siyah/gri) tonlardadır.
        # Bu yüzden düşük Doygunluk (Saturation) ve düşük Parlaklık (Value) olan yerleri maskeliyoruz.
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 100, 80]) # Çok açık olmayan ve çok renkli olmayan yerler
        
        mask = cv2.inRange(hsv, lower_black, upper_black)
        
        # 3. Gürültü (ufak lekeler) temizliği
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # 4. Konturları bul (Koyu bölgelerin sınırlarını çiz)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Her bir potansiyel koyu bölgeyi analiz et
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Boyut filtresi: Çok küçük toz parçalarını veya koca dağ gölgelerini atla
            if area < 500 or area > 50000:
                continue
                
            # Koyu bölgenin çevresine dikdörtgen Bounding Box çiz
            x, y, w, h = cv2.boundingRect(cnt)
            x1, y1, x2, y2 = x, y, x + w, y + h
            rock_roi = cv_image[y1:y2, x1:x2]
            
            if rock_roi.size == 0:
                continue
                
            # --- FİZİKSEL DOĞRULAMA (Eski yazdığınız mantık) ---
            hsv_roi = cv2.cvtColor(rock_roi, cv2.COLOR_BGR2HSV)
            mean_val = np.mean(hsv_roi[:, :, 2])
            mean_sat = np.mean(hsv_roi[:, :, 1])
            max_val = np.max(hsv_roi[:, :, 2]) # Güneş yansıması
            
            gray_roi = cv2.cvtColor(rock_roi, cv2.COLOR_BGR2GRAY)
            laplacian_var = cv2.Laplacian(gray_roi, cv2.CV_64F).var()
            
            is_ilmenite = True
            
            # Mat, dümdüz siyah gölgeleri elemek için en parlak nokta kuralı (İlmenitte kristal yapı parlar)
            if max_val < 100:  # Önceden 150 idi, OpenCV ile maske biraz farklı kırpabilir, güvenli aralık 100
                is_ilmenite = False 
                
            # Doku testi (Gölgeler genelde çok pürüzsüzdür, kayalar pürüzlü)
            if laplacian_var < 50 or laplacian_var > 1000:
                is_ilmenite = False 
                
            if is_ilmenite:
                # Yeşil kutu - Onaylı İlmenit!
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                cx = x + w // 2
                cy = y + h // 2
                
                # Derinlik Hesabı
                depth_val = 0.0
                if self.latest_depth_image is not None:
                    try:
                        val = self.latest_depth_image[cy, cx]
                        if np.isnan(val) or val == 0:
                            window = self.latest_depth_image[max(0, cy-5):cy+5, max(0, cx-5):cx+5]
                            valid_depths = window[(~np.isnan(window)) & (window > 0)]
                            if len(valid_depths) > 0:
                                val = np.nanmean(valid_depths)
                            else:
                                val = 0.0
                        
                        if self.latest_depth_image.dtype == np.uint16:
                            depth_val = float(val) / 1000.0
                        else:
                            depth_val = float(val)
                    except IndexError:
                        pass
                
                msg_arr = Float32MultiArray()
                msg_arr.data = [float(cx), float(cy), float(depth_val), 1.0]
                self.rock_pub.publish(msg_arr)
                
                cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                label = f"ILM-CV V:{laplacian_var:.0f} M:{mean_val:.0f}"
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                label = f"REJ-CV V:{laplacian_var:.0f} M:{mean_val:.0f}"
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    node = VisionPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

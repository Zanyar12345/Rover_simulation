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
from ultralytics import YOLO

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
        
        # ArUco Dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # YOLOv8 Model (Yeni eğitilen best.pt modeli)
        try:
            package_share_directory = get_package_share_directory('robot_2_gps_description')
            model_path = os.path.join(package_share_directory, 'best.pt')
            self.yolo_model = YOLO(model_path)
            self.get_logger().info(f"YOLOv8 model loaded successfully from {model_path}.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.yolo_model = None
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
        if self.yolo_model is None:
            return
            
        # Run YOLO inference
        results = self.yolo_model(cv_image, verbose=False)
        
        # Parse results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                class_name = self.yolo_model.names[cls]
                
                if conf > 0.65:
                    # Kayanın sadece bulunduğu bölgeyi (Bounding Box) kes
                    rock_roi = cv_image[y1:y2, x1:x2]
                    
                    if rock_roi.size == 0:
                        continue
                        
                    # 1. Renk Analizi (Siyah/Gri kontrolü)
                    hsv = cv2.cvtColor(rock_roi, cv2.COLOR_BGR2HSV)
                    mean_val = np.mean(hsv[:, :, 2]) # Ortalama parlaklık (Value)
                    mean_sat = np.mean(hsv[:, :, 1]) # Ortalama doygunluk (Saturation)
                    
                    # 2. Parlaklık Analizi (Mat değil, parlak noktaları var)
                    max_val = np.max(hsv[:, :, 2]) # Kutu içindeki en parlak nokta
                    
                    # 3. Doku Analizi (Tanecikli yapı -> Laplacian varyansı)
                    gray = cv2.cvtColor(rock_roi, cv2.COLOR_BGR2GRAY)
                    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
                    
                    is_ilmenite = True
                    
                    # İlmenit kapkaradır: Ortalama parlaklık (Value) 80'den, Doygunluk 80'den küçük olmalı. (Çantalar genelde daha açık renklidir)
                    if mean_val > 80 or mean_sat > 80:
                        is_ilmenite = False 
                    
                    # Mat kutuları elemek için en parlak nokta (güneş yansıması) 150'den büyük olmalı
                    if max_val < 150:
                        is_ilmenite = False 
                        
                    if laplacian_var < 100:
                        is_ilmenite = False 
                        
                    if laplacian_var > 800:
                        is_ilmenite = False
                        
                    if class_name not in ['Igneous_Basalt', 'Sedimentary_coal']:
                        is_ilmenite = False
                        
                    # Eğer tüm bu fiziksel testleri geçerse, evet bu bir ilmenittir!
                    if is_ilmenite:
                        # Draw bounding box (Yeşil - Onaylandı)
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Center point
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)
                        
                        # Calculate depth
                        depth_val = 0.0
                        if self.latest_depth_image is not None:
                            try:
                                val = self.latest_depth_image[cy, cx]
                                if np.isnan(val) or val == 0:
                                    # Fallback to small window
                                    window = self.latest_depth_image[max(0, cy-5):cy+5, max(0, cx-5):cx+5]
                                    valid_depths = window[(~np.isnan(window)) & (window > 0)]
                                    if len(valid_depths) > 0:
                                        val = np.nanmean(valid_depths)
                                    else:
                                        val = 0.0
                                
                                # Convert to meters if uint16 (mm)
                                if self.latest_depth_image.dtype == np.uint16:
                                    depth_val = float(val) / 1000.0
                                else:
                                    depth_val = float(val)
                            except IndexError:
                                pass
                        
                        # X, Y koordinatları, derinlik ve güven skorunu birlikte gönderiyoruz
                        msg_arr = Float32MultiArray()
                        msg_arr.data = [float(cx), float(cy), float(depth_val), float(conf)]
                        self.rock_pub.publish(msg_arr)
                        
                        # Cisim merkezine nokta ve yazi koy
                        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                        # HATA AYIKLAMA: Ekrana Sınıf Adını, Varyans (V) ve Parlaklık (M) değerlerini yazdırıyoruz
                        label = f"ILM {class_name[:6]} V:{laplacian_var:.0f} M:{mean_val:.0f}"
                        cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    else:
                        # Kırmızı kutu - Kaya bulundu ama İlmenit değil
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        label = f"REJ {class_name[:6]} V:{laplacian_var:.0f} M:{mean_val:.0f}"
                        cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    node = VisionPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        # Changed topic to /gps/fix to match navsat_transform_node expectations
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        requests.packages.urllib3.disable_warnings(InsecureRequestWarning)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.HOST     = "10.42.0.20"
        self.USER     = "meturover"
        self.PASSWORD = "meturoverchallenger1_"
        self.uere = 5.0
        self.get_logger().info("GPS Publisher (Robust Version) Baslatildi...")

    def timer_callback(self):
        # Güvenlik Kalkanı (Try-Except) eklendi
        try:
            gps = self.get_prism_gps(self.HOST, self.USER, self.PASSWORD)
        except Exception as e:
            # Hata verip çökmek yerine uyarı verir ve bir sonraki turu bekler
            self.get_logger().warn(f"GPS Baglantisi Aranıyor... (Cihaz Kapalı Olabilir)", throttle_duration_sec=2.0)
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'

        dop = float(gps['horizontal_dilution'])
        sigma = dop * self.uere     # e.g. 2.94 × 5 m ≃ 14.7 m
        var   = sigma * sigma

        # Status: status/status_service (set to STATUS_FIX and SERVICE_GPS)
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Fill in the fix
        msg.latitude = float(gps['latitude'])
        msg.longitude = float(gps['longitude'])
        msg.altitude = float(gps['altitude_m'])

        msg.position_covariance = [
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, var
        ]
        
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.pub.publish(msg)
        self.get_logger().info(
            f'Publishing GPS fix: '
            f'lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.1f}'
        )

    def get_prism_gps(self, host, username, password):
        """
        Logs into a Rocket Prism AC, then fetches /status.cgi JSON and returns the 'gps' block.
        """
        login_url   = f"https://{host}/api/auth"
        status_url  = f"https://{host}/status.cgi"
        session = requests.Session()
        session.verify = False  # skip cert check
        session.headers.update({
            "User-Agent": "Mozilla/5.0",
            "Accept":       "application/json",
        })

        # 2) Authenticate and establish session cookie
        r = session.post(
            login_url,
            data={"username": username, "password": password},
            timeout=2.0 # Daha kısa timeout çabuk kurtulması için
        )
        r.raise_for_status()

        # 3) Fetch the JSON status
        r = session.get(status_url, timeout=2.0)
        r.raise_for_status()
        data = r.json()

        # 4) Extract GPS block
        gps = data.get("gps")
        if not gps:
            raise ValueError("No 'gps' section in JSON response")

        return {
            "latitude":           gps["lat"],
            "longitude":          gps["lon"],
            "fix_acquired":       bool(gps["fix"]),
            "satellites":         gps["sats"],
            "horizontal_dilution":gps["dop"],
            "altitude_m":         gps["alt"],
            "last_sync":          gps["last_sync"],
        }

def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

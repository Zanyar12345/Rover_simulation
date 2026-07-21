#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

import requests
import urllib3

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


class GpsPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')

        self.declare_parameter('host', '10.42.0.65')
        self.declare_parameter('username', 'meturover')
        self.declare_parameter('password', 'meturoverchallenger1_')
        self.declare_parameter('uere', 5.0)
        self.declare_parameter('rate_hz', 1.0)

        self.host = self.get_parameter('host').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.uere = float(self.get_parameter('uere').value)

        self.pub = self.create_publisher(NavSatFix, 'fix', 10)

        # One persistent session: keeps the TCP connection + auth cookie alive
        # instead of re-authenticating on every single poll.
        self.session = requests.Session()
        self.session.verify = False
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0',
            'Accept': 'application/json',
        })
        self.authenticated = False

        rate = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    # ------------------------------------------------------------------ #

    def timer_callback(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.status.service = NavSatStatus.SERVICE_GPS

        try:
            gps = self.get_prism_gps()
        except Exception as e:
            # Radio unreachable, auth failed, JSON malformed, etc.
            # Do NOT crash the node -- publish a NO_FIX message instead.
            self.authenticated = False  # force re-login on next attempt
            self.get_logger().warn(
                f'GPS read failed: {e}',
                throttle_duration_sec=5.0,
            )
            self.publish_no_fix(msg)
            return

        # The radio answered, but the receiver itself may have no fix.
        # In that state lat/lon/dop are often missing, None, or garbage.
        if not gps.get('fix_acquired') or gps.get('latitude') is None:
            self.get_logger().warn(
                f"No GPS fix (satellites={gps.get('satellites', '?')})",
                throttle_duration_sec=5.0,
            )
            self.publish_no_fix(msg)
            return

        try:
            lat = float(gps['latitude'])
            lon = float(gps['longitude'])
            alt = float(gps['altitude_m'] or 0.0)
            dop = float(gps['horizontal_dilution'] or 99.9)
        except (TypeError, ValueError) as e:
            self.get_logger().warn(
                f'Bad GPS values from radio: {e}',
                throttle_duration_sec=5.0,
            )
            self.publish_no_fix(msg)
            return

        sigma = dop * self.uere
        var = sigma * sigma

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.position_covariance = [
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, var,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.pub.publish(msg)
        self.get_logger().info(
            f'lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}, '
            f"sats={gps.get('satellites')}, hdop={dop:.2f}",
            throttle_duration_sec=1.0,
        )

    def publish_no_fix(self, msg: NavSatFix):
        """Publish a NavSatFix marked STATUS_NO_FIX so downstream nodes
        (robot_localization, navsat_transform, etc.) know the GPS is out
        instead of the topic silently going dead."""
        msg.status.status = NavSatStatus.STATUS_NO_FIX
        msg.latitude = math.nan
        msg.longitude = math.nan
        msg.altitude = math.nan
        msg.position_covariance = [0.0] * 9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub.publish(msg)

    # ------------------------------------------------------------------ #

    def login(self):
        r = self.session.post(
            f'https://{self.host}/api/auth',
            data={'username': self.username, 'password': self.password},
            timeout=(2.0, 3.0),  # (connect, read) timeouts
        )
        r.raise_for_status()
        self.authenticated = True

    def get_prism_gps(self):
        """Fetch /status.cgi from the Rocket Prism and return its 'gps' block.
        Reuses the existing session/cookie; re-authenticates only when needed."""
        if not self.authenticated:
            self.login()

        r = self.session.get(f'https://{self.host}/status.cgi', timeout=(2.0, 3.0))

        # Session cookie expired -> log in once and retry once.
        if r.status_code in (401, 403):
            self.login()
            r = self.session.get(f'https://{self.host}/status.cgi', timeout=(2.0, 3.0))

        r.raise_for_status()
        data = r.json()

        gps = data.get('gps')
        if not gps:
            raise ValueError("no 'gps' section in status.cgi response")

        return {
            'latitude': gps.get('lat'),
            'longitude': gps.get('lon'),
            'fix_acquired': bool(gps.get('fix')),
            'satellites': gps.get('sats'),
            'horizontal_dilution': gps.get('dop'),
            'altitude_m': gps.get('alt'),
            'last_sync': gps.get('last_sync'),
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
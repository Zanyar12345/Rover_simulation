#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from sensor_msgs.msg import Joy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import math as mt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry


MAX_SPEED = 32

class HotasProcessor(Node):
    def __init__(self):
        super().__init__('hotas_control')
        
        self.declare_parameter('frequency', 50.0)
        self.frequency = self.get_parameter('frequency').value
        
        self.current_mode = 0
        self.mode_names = ["NORMAL", "CRAB", "SPIN"]
        
        self.mode_buttons = {
            'normal': 2,
            'crab': 3,
            'spin': 4
        }
        
        self.light_mode = False
        self.light_button = 5

        self.last_button_states = {
            'normal': False,
            'crab': False,
            'spin': False,
            'light': False
        }

        self.rover_width = 60 # unit in centimeters
        self.rover_length = 96.5 # unit in centimeters

        self.previous_vertical_joy_comp = 0
        self.control_threshold = 2
        self.control_threshold_update_val = 1

        self.publishers_ = {
            'wheel_front_left_angle':  self.create_publisher(Int32, 'wheel_front_left_angle', 10),
            'wheel_front_right_angle': self.create_publisher(Int32, 'wheel_front_right_angle', 10),
            'wheel_rear_left_angle':   self.create_publisher(Int32, 'wheel_rear_left_angle', 10),
            'wheel_rear_right_angle':  self.create_publisher(Int32, 'wheel_rear_right_angle', 10),
            'wheel_front_left_speed':  self.create_publisher(Int32, 'wheel_front_left_speed', 10),
            'wheel_front_right_speed': self.create_publisher(Int32, 'wheel_front_right_speed', 10),
            'wheel_rear_left_speed':   self.create_publisher(Int32, 'wheel_rear_left_speed', 10),
            'wheel_rear_right_speed':  self.create_publisher(Int32, 'wheel_rear_right_speed', 10),
            'wheel_light':  self.create_publisher(Bool,  'wheel_light', 10),
            'sallama':      self.create_publisher(Int32, 'sallama', 10),
            'sallama2':     self.create_publisher(Int32, 'sallama2', 10)
        }
        
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joy_callrear, 10)
        self.cmd_vel_subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callrear, 10)
        self.create_subscription(String, '/control_mode', self.mode_callback, 10)
        # self.path_sub = self.create_subscription(
        #     Path, '/received_global_plan', self.path_callback, 10)
        # self.target_yaw = None
        # self.odom_sub = self.create_subscription(
        #     Odometry, '/odom', self.odom_callback, 10)
        # self.current_yaw = 0.0

        self.msgs = {}
        for name in self.publishers_.keys():
            if name == 'wheel_light':
                self.msgs[name] = Bool()
            else:
                self.msgs[name] = Int32()
        
        period = 1.0 / self.frequency
        self.timer_ = self.create_timer(period, self.timer_callrear)
        
        
        self.joy_msg = Joy()
        self.cmd_vel_msg = Twist()
        self.control_mode = "Joy"
        self.last_cmd_time = None
  
        self.get_logger().info(f'Motor Control Node initialized at {self.frequency} Hz')
        self.get_logger().info(f'Current mode: {self.mode_names[self.current_mode]}')

    def mode_callback(self, msg):
        if msg.data == "NAV":
            self.get_logger().info("NAV")
            self.control_mode = "NAV"
            self.current_mode = 0
        elif msg.data == "SPIN":
            self.get_logger().info("SPIN")
            self.control_mode = "NAV"
            self.current_mode = 2
        elif msg.data == "JOY":
            self.control_mode = "Joy"
            self.get_logger().info("Joy")

    def process_normal_mode(self, msg):
        if self.control_mode == "Joy":
            steering = (-self.joy_msg.axes[0] * 90) 
            vertical_joy_comp = self.joy_msg.axes[1]*64 
        else:
            steering = (-self.cmd_vel_msg.angular.z) * 90   
            vertical_joy_comp = self.cmd_vel_msg.linear.x * 100
        self.msgs['sallama'].data = 18
        self.msgs['sallama2'].data = 81

        if steering > 0 : # rover turning right


            alfa = steering # turning angle at turning side
            theta = mt.degrees(mt.atan((self.rover_length*mt.tan(mt.radians(alfa)))/ #  turning angle at counter turning side
                    (self.rover_length + 2*self.rover_width*mt.tan(mt.radians(alfa)))))
            
            self.msgs['wheel_front_left_angle'].data = int(180 + theta)
            self.msgs['wheel_front_right_angle'].data = int(180 + alfa)
            self.msgs['wheel_rear_left_angle'].data = int(180 - theta)
            self.msgs['wheel_rear_right_angle'].data = int(180 - alfa)

            if vertical_joy_comp > 0:

                speed_counter_turning_side = vertical_joy_comp**2 /64
                speed_turning_side = speed_counter_turning_side*(mt.sin(mt.radians(theta))/
                                                                mt.sin(mt.radians(alfa)))
                

                for wheel in ['front_left',  'rear_left']:
                    self.msgs[f'wheel_{wheel}_speed'].data = int(speed_counter_turning_side)

                for wheel in [ 'front_right', 'rear_right']:
                    self.msgs[f'wheel_{wheel}_speed'].data = int(speed_turning_side)
            
            elif vertical_joy_comp < 0:
                
                speed_counter_turning_side = vertical_joy_comp**2 /64
                speed_turning_side = speed_counter_turning_side*(mt.sin(mt.radians(theta))/
                                                                mt.sin(mt.radians(alfa)))
                

                for wheel in ['front_left',  'rear_left']:
                    self.msgs[f'wheel_{wheel}_speed'].data = -int(speed_counter_turning_side)

                for wheel in [ 'front_right', 'rear_right']:
                    self.msgs[f'wheel_{wheel}_speed'].data = -int(speed_turning_side)
            
            else:

                for wheel in ['front_left',  'rear_left']:
                    self.msgs[f'wheel_{wheel}_speed'].data = 0

                for wheel in [ 'front_right', 'rear_right']:
                    self.msgs[f'wheel_{wheel}_speed'].data = 0

            
        elif steering < 0: # rover turning left


            alfa = -steering
            theta = mt.degrees(mt.atan((self.rover_length*mt.tan(mt.radians(alfa)))/
                    (self.rover_length + 2*self.rover_width*mt.tan(mt.radians(alfa)))))
            
            self.msgs['wheel_front_left_angle'].data = int(180 - alfa)
            self.msgs['wheel_front_right_angle'].data = int(180 -theta) 
            self.msgs['wheel_rear_left_angle'].data = int(180 + alfa)
            self.msgs['wheel_rear_right_angle'].data = int(180 + theta)

            if vertical_joy_comp > 0:

                speed_counter_turning_side = vertical_joy_comp**2 / 64
                speed_turning_side = speed_counter_turning_side*(mt.sin(mt.radians(theta))/
                                                                mt.sin(mt.radians(alfa)))
                
                

                for wheel in ['front_left',  'rear_left']:
                    self.msgs[f'wheel_{wheel}_speed'].data = int(speed_turning_side)

                for wheel in [ 'front_right', 'rear_right']:
                    self.msgs[f'wheel_{wheel}_speed'].data = int(speed_counter_turning_side)
            
            elif vertical_joy_comp < 0:

                speed_counter_turning_side = vertical_joy_comp**2 / 64
                speed_turning_side = speed_counter_turning_side*(mt.sin(mt.radians(theta))/
                                                        mt.sin(mt.radians(alfa)))
                

                for wheel in ['front_left',  'rear_left']:
                    self.msgs[f'wheel_{wheel}_speed'].data = -int(speed_turning_side)

                for wheel in [ 'front_right', 'rear_right']:
                    self.msgs[f'wheel_{wheel}_speed'].data = -int(speed_counter_turning_side)
            
            else:
                for wheel in ['front_left',  'rear_left']:
                    self.msgs[f'wheel_{wheel}_speed'].data = 0

                for wheel in [ 'front_right', 'rear_right']:
                    self.msgs[f'wheel_{wheel}_speed'].data = 0


        else: # rover going straight


            self.msgs['wheel_front_left_angle'].data = 180 
            self.msgs['wheel_front_right_angle'].data = 180
            self.msgs['wheel_rear_left_angle'].data = 180 
            self.msgs['wheel_rear_right_angle'].data = 180 

            if vertical_joy_comp > 0:

                speed = vertical_joy_comp**2 /64

            elif vertical_joy_comp < 0:

                speed = -vertical_joy_comp**2 /64

            else:
                speed = 0


            for wheel in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                self.msgs[f'wheel_{wheel}_speed'].data = int(speed)



    def process_crab_mode(self, msg):

        self.msgs['sallama'].data = 18
        self.msgs['sallama2'].data = 81


        steering = int(-self.joy_msg.axes[0] * 90 + 180)
        vertical_joy_comp = self.joy_msg.axes[1] * 64


        for wheel in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            self.msgs[f'wheel_{wheel}_angle'].data = int(steering)

        if vertical_joy_comp > 0:
            speed = vertical_joy_comp ** 2 / 100
        elif vertical_joy_comp < 0:
            speed = -(vertical_joy_comp ** 2 / 100)
        else:
            speed = 0

        for wheel in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            self.msgs[f'wheel_{wheel}_speed'].data = int(speed)

    def process_spin_mode(self, msg):
        self.msgs['sallama'].data = 18
        self.msgs['sallama2'].data = 81

        zero_turning_degree_offset = mt.degrees(mt.atan(self.rover_length / self.rover_width))

        self.msgs['wheel_front_left_angle'].data  = int(180 + zero_turning_degree_offset)
        self.msgs['wheel_front_right_angle'].data = int(180 - zero_turning_degree_offset)
        self.msgs['wheel_rear_left_angle'].data   = int(180 - zero_turning_degree_offset)
        self.msgs['wheel_rear_right_angle'].data  = int(180 + zero_turning_degree_offset)

  
        if self.control_mode == "Joy":
            horizontal_joy_comp = self.joy_msg.axes[0] * 64
        else:
            horizontal_joy_comp = self.cmd_vel_msg.angular.z * 64


        speed = int((horizontal_joy_comp ** 2) / 64)
        if horizontal_joy_comp >= 0:
            self.msgs['wheel_front_right_speed'].data =  speed
            self.msgs['wheel_rear_right_speed'].data  =  speed
            self.msgs['wheel_front_left_speed'].data  = -speed
            self.msgs['wheel_rear_left_speed'].data   = -speed
        else:
            self.msgs['wheel_front_right_speed'].data = -speed
            self.msgs['wheel_rear_right_speed'].data  = -speed
            self.msgs['wheel_front_left_speed'].data  =  speed
            self.msgs['wheel_rear_left_speed'].data   =  speed

    def check_mode_changes(self, msg):
        for mode, button_idx in self.mode_buttons.items():
            current_state = msg.buttons[button_idx]
            if current_state and not self.last_button_states[mode]:
                new_mode = list(self.mode_buttons.keys()).index(mode)
                if new_mode != self.current_mode:
                    self.current_mode = new_mode
                    self.get_logger().info(f'Mode changed to: {self.mode_names[self.current_mode]}')
            self.last_button_states[mode] = current_state
        
        current_light_state = msg.buttons[self.light_button]
        if current_light_state and not self.last_button_states['light']:
            self.light_mode = not self.light_mode
        self.msgs['wheel_light'].data = self.light_mode
        self.last_button_states['light'] = current_light_state

    def joy_callrear(self, msg):
        self.joy_msg = msg
        self.check_mode_changes(msg)
        
        # Eğer Manuel (Joy) moddaysak motorlara joystick verilerini yolla
        if self.control_mode == "Joy":
            if self.current_mode == 0:
                self.process_normal_mode(None)
            elif self.current_mode == 2:
                self.process_spin_mode(None)

        return


    def timer_callrear(self):
        for name, publisher in self.publishers_.items():
            publisher.publish(self.msgs[name])

    def cmd_vel_callrear(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.cmd_vel_msg = msg
        
        # Eğer Otonom (NAV) moddaysak Nav2'nin motor komutlarını işle
        if self.control_mode == "NAV":
            if self.current_mode == 0:
                self.process_normal_mode(self.cmd_vel_msg)
            elif self.current_mode == 2:
                self.process_spin_mode(self.cmd_vel_msg)
    
    # def path_callback(self, msg):
    #     if len(msg.poses) > 2:
    #         p0 = msg.poses[0].pose.position
    #         p1 = msg.poses[1].pose.position
    #         self.target_yaw = mt.atan2(p1.y - p0.y, p1.x - p0.x)

    # def odom_callback(self, msg):
    #     q = msg.pose.pose.orientation
    #     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    #     self.current_yaw = mt.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = HotasProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
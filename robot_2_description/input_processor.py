#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Joy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import math as mt



MAX_SPEED = 32

class HotasProcessor(Node):
    """
    A ROS2 node for processing HOTAS (Hands On Throttle-And-Stick) controls.

    This node processes joystick inputs and converts them into wheel angle and speed
    commands for a four-wheeled robot. It supports three different driving modes:
    normal, crab, and spin.

    Publishers:
        * wheel_front_left_angle (Int32): Front left wheel steering angle (-180-180 degrees)
        * wheel_front_right_angle (Int32): Front right wheel steering angle (-180-180 degrees)
        * wheel_rear_left_angle (Int32): rear left wheel steering angle (-180-180 degrees)
        * wheel_rear_right_angle (Int32): rear right wheel steering angle (-180-180 degrees)
        * wheel_front_left_speed (Int32): Front left wheel speed (-MAX_SPEED to MAX_SPEED)
        * wheel_front_right_speed (Int32): Front right wheel speed (-MAX_SPEED to MAX_SPEED)
        * wheel_rear_left_speed (Int32): rear left wheel speed (-MAX_SPEED to MAX_SPEED)
        * wheel_rear_right_speed (Int32): rear right wheel speed (-MAX_SPEED to MAX_SPEED)

    Subscribers:
        * joy (sensor_msgs/Joy): Joystick input data

    Parameters:
        * frequency (float): Update frequency in Hz (default: MAX_SPEED.0)
    """

    def __init__(self):
        """Initialize the HotasProcessor node with publishers, subscribers, and parameters."""
        super().__init__('hotas_control')
        
        # Declare frequency parameter
        self.declare_parameter('frequency', 50.0)
        self.frequency = self.get_parameter('frequency').value
        
        # Initialize mode state
        self.current_mode = 0  # 0: Normal, 1: Crab, 2: Spin
        self.mode_names = ["NORMAL", "CRAB", "SPIN"]
        
        # Define mode buttons
        self.mode_buttons = {
            'normal': 2,  # Button 2 for normal mode
            'crab': 3,    # Button 3 for crab mode
            'spin': 4     # Button 4 for spin mode
        }
        
        self.light_mode = False
        self.light_button = 5

        self.last_button_states = {
            'normal': False,
            'crab': False,
            'spin': False,
            'light': False
        }



        self.rover_width = 82 # unit in centimeters
        self.rover_length = 95 # unit in centimeters

        self.previous_vertical_joy_comp = 0
        self.control_threshold = 2
        self.control_threshold_update_val = 1


        # Create publishers #dictionary
        self.publishers_ = {
            'wheel_front_left_angle': self.create_publisher(Int32, 'wheel_front_left_angle', 10),
            'wheel_front_right_angle': self.create_publisher(Int32, 'wheel_front_right_angle', 10),
            'wheel_rear_left_angle': self.create_publisher(Int32, 'wheel_rear_left_angle', 10),
            'wheel_rear_right_angle': self.create_publisher(Int32, 'wheel_rear_right_angle', 10),
            'wheel_front_left_speed': self.create_publisher(Int32, 'wheel_front_left_speed', 10),
            'wheel_front_right_speed': self.create_publisher(Int32, 'wheel_front_right_speed', 10),
            'wheel_rear_left_speed': self.create_publisher(Int32, 'wheel_rear_left_speed', 10),
            'wheel_rear_right_speed': self.create_publisher(Int32, 'wheel_rear_right_speed', 10),
            'wheel_light': self.create_publisher(Bool , 'wheel_light', 10),
            'sallama': self.create_publisher(Int32, 'sallama', 10),
            'sallama2': self.create_publisher(Int32, 'sallama2', 10)
        }
        
        # Subscribe to joystick
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callrear,
            10
        )
        
        # Initialize messages
        self.msgs = {}
        for name in self.publishers_.keys():
            if name == 'wheel_light':
                self.msgs[name] = Bool()
            else:
                self.msgs[name] = Int32()
        
        # Create timer
        period = 1.0 / self.frequency
        self.timer_ = self.create_timer(period, self.timer_callrear)
        
        self.get_logger().info(f'Motor Control Node initialized at {self.frequency} Hz')
        self.get_logger().info(f'Current mode: {self.mode_names[self.current_mode]}')

#---------------------------CONTROL PROCESSING & SMOOTHING START---------------------------------#
       
        

    def process_normal_mode(self, msg):
        """
        Process joystick input for normal driving mode.
        
        In normal mode, front wheels steer together, and rear wheels steer together
        in the opposite direction for enhanced turning capability.

        Args:
            msg (sensor_msgs.msg.Joy): The joystick input message
        """
       
        # let turning side wheel angle be alfa and opposite of turning side wheel angle be theta
        # in formula : l is length of rover & w is width of rover
        # we will give turning side angle (alfa) and calculate theta with respect to that angle by the formula ;
        # Formula : arctan[(l*tan(alfa))/(l+2w*tan(alfa))] = theta
       
        #--------------STEERING & VELOCITY PROCESSING---------------#

        steering = (-msg.axes[0] * 90) # Joystick horizontal axis value mapping from [0,1] to [0,90]
        vertical_joy_comp = msg.axes[1]*64 # Joystick vertical axis value mapping from [0,1] to [0,100]
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
        """
        Process joystick input for crab driving mode.
        
        In crab mode, all wheels steer in the same direction, allowing the robot
        to move sideways while maintaining its orientation.

        Args:
            msg (sensor_msgs.msg.Joy): The joystick input message
        """
        self.msgs['sallama'].data = 18
        self.msgs['sallama2'].data = 81

        
        steering = int(-msg.axes[0] * 90 + 180) # Joystick horizontal axis value mapping from [0,1] to [0,90]

        vertical_joy_comp=msg.axes[1]*64

        for wheel in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            self.msgs[f'wheel_{wheel}_angle'].data = steering
        if vertical_joy_comp > 0:

            speed = vertical_joy_comp**2 /100

        elif vertical_joy_comp < 0:
            speed = -(vertical_joy_comp**2 / 100)
        
        else:
            speed = 0

        for wheel in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            self.msgs[f'wheel_{wheel}_angle'].data = steering
        
         
        for wheel in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            self.msgs[f'wheel_{wheel}_speed'].data = int(speed)

    def process_spin_mode(self, msg):
        """
        Process joystick input for spin driving mode.
        
        In spin mode, wheels are positioned to rotate the robot around its center,
        allowing for zero-radius turning.

        Args:
            msg (sensor_msgs.msg.Joy): The joystick input message
        """

        self.msgs['sallama'].data = 18
        self.msgs['sallama2'].data = 81

        zero_turning_degree_offset = mt.degrees(mt.atan(self.rover_length/self.rover_width))

        self.msgs['wheel_front_left_angle'].data = int(180 + zero_turning_degree_offset)
        self.msgs['wheel_front_right_angle'].data = int(180 - zero_turning_degree_offset)
        self.msgs['wheel_rear_left_angle'].data = int(180 - zero_turning_degree_offset)
        self.msgs['wheel_rear_right_angle'].data = int(180 + zero_turning_degree_offset)
        
        horizontal_joy_comp = msg.axes[0]*64 # Joystick vertical axis value mapping from [0,1] to [0,100]
        speed = int((horizontal_joy_comp**2)/64) 
        if horizontal_joy_comp >= 0:
            self.msgs['wheel_front_right_speed'].data = speed
            self.msgs['wheel_rear_right_speed'].data = speed
            self.msgs['wheel_front_left_speed'].data = -speed
            self.msgs['wheel_rear_left_speed'].data = -speed
        else:
            self.msgs['wheel_front_right_speed'].data = -speed
            self.msgs['wheel_rear_right_speed'].data = -speed
            self.msgs['wheel_front_left_speed'].data = speed
            self.msgs['wheel_rear_left_speed'].data = speed


#-------------------------------- CONTROL PROCESSING & SMOOTHING END -------------------------------


    def check_mode_changes(self, msg):
        """
        Check for mode button presses and update the current driving mode.

        Args:
            msg (sensor_msgs.msg.Joy): The joystick input message
        """
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
        light_status = "ON" if self.light_mode else "OFF"
        self.msgs['wheel_light'].data = light_status == "ON"
        # self.get_logger().info(f'Light mode toggled: {light_status}') 
    
        self.last_button_states['light'] = current_light_state
        
        

    def joy_callrear(self, msg):
        """
        Process incoming joystick messages.

        This callrear checks for mode changes and processes the joystick input
        according to the current driving mode.

        Args:
            msg (sensor_msgs.msg.Joy): The joystick input message
        """
        self.check_mode_changes(msg)
        
        if self.current_mode == 0:
            self.process_normal_mode(msg)
        elif self.current_mode == 1:
            self.process_crab_mode(msg)
        elif self.current_mode == 2:
            self.process_spin_mode(msg)

    def timer_callrear(self):
        """Publish wheel commands at the specified frequency."""
        for name, publisher in self.publishers_.items():
            publisher.publish(self.msgs[name])




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
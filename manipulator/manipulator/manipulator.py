
import rclpy
from controls_msgs.msg import SpeedClosedLoopControlMsgSentParams as SendSpeed, ReadMotorStatus1MsgSentParams as SendStatus1, SystemResetMsgSentParams as Reset
from rclpy.node import Node
from sensor_msgs.msg import Joy
from math import floor

RESTART_BTN = 4 # Top Face Button
RAIL_LEFT_BTN = 6 # Left Bumper
RAIL_RIGHT_BTN = 7 # Right Buper
SHOULDER_AXES = 1 # Left Stick
ELBOW_AXES = 3 # Right Stick
WRIST_SPEED_AXES = 4 # Right Trigger
WRIST_ROLL_AXES = 6 # Left/Right D Pad
WRIST_PITCH_AXES = 7 #Up/Down D Pad 

MOTOR_STATUS_HZ = 2
MAX_DPS = 20

def clamp(val) -> int:
    return max(-1,min(val,1))

def normalize_joystick_axes_vals(value: float):
    return clamp(-1 * value)
        
def normalize_bumper_axes_vals(value: float):
    return clamp(1 - ((value + 1)/2))

class Manipulator(Node):

    def __init__(self):
        super().__init__('manipulator')

        # Speed Publishers         
        self.shoulder_pub = self.create_publisher(SendSpeed,"/manipulator/shoulder/send/speed_control",     10)
        self.elbow_pub = self.create_publisher(SendSpeed,   "/manipulator/elbow/send/speed_control",        10)
        self.roll_pub = self.create_publisher(SendSpeed,    "/manipulator/wrist_roll/send/speed_control",   10)
        self.pitch_pub = self.create_publisher(SendSpeed,   "/manipulator/wrist_pitch/send/speed_control",  10)
        self.rail_pub = self.create_publisher(SendSpeed,    "/manipulator/linear_rail/send/speed_control",  10)

        # Status Publishers
        self.shoulder_status_pub = self.create_publisher(SendStatus1,"/manipulator/shoulder/send/status1",     10)
        self.elbow_status_pub = self.create_publisher(SendStatus1,   "/manipulator/elbow/send/status1",        10)
        self.roll_status_pub = self.create_publisher(SendStatus1,    "/manipulator/wrist_roll/send/status1",   10)
        self.pitch_status_pub = self.create_publisher(SendStatus1,   "/manipulator/wrist_pitch/send/status1",  10)
        self.rail_status_pub = self.create_publisher(SendStatus1,    "/manipulator/linear_rail/send/status1",  10)

        # Reset Publishers
        self.shoulder_reset_pub = self.create_publisher(Reset,"/manipulator/shoulder/send/reset",     10)
        self.elbow_reset_pub = self.create_publisher(Reset,   "/manipulator/elbow/send/reset",        10)
        self.roll_reset_pub = self.create_publisher(Reset,    "/manipulator/wrist_roll/send/reset",   10)
        self.pitch_reset_pub = self.create_publisher(Reset,   "/manipulator/wrist_pitch/send/reset",  10)
        self.rail_reset_pub = self.create_publisher(Reset,    "/manipulator/linear_rail/send/reset",  10)

        self.create_subscription(Joy,   "/joy",     self.calculate_motor_speeds)

        self.create_timer(1/MOTOR_STATUS_HZ,self.send_status_msgs)


    def send_reset_msgs(self):
        reset_msg = Reset()
        self.shoulder_reset_pub.publish(reset_msg)
        self.elbow_reset_pub.publish(reset_msg)
        self.roll_reset_pub.publish(reset_msg)
        self.pitch_reset_pub.publish(reset_msg)
        self.rail_reset_pub.publish(reset_msg)


    def send_status_msgs(self):
        status_msg = SendStatus1()
        self.shoulder_status_pub.publish(status_msg)
        self.elbow_status_pub.publish(status_msg)
        self.roll_status_pub.publish(status_msg)
        self.pitch_status_pub.publish(status_msg)
        self.rail_status_pub.publish(status_msg)


    def send_speed_commands(self, rail, shoulder, elbow, roll, pitch):
        rail_msg = SendSpeed(); rail_msg.speed_dps = rail
        shoulder_msg = SendSpeed(); shoulder_msg.speed_dps = shoulder
        elbow_msg = SendSpeed(); elbow_msg.speed_dps = elbow
        roll_msg = SendSpeed(); roll_msg.speed_dps = roll
        pitch_msg = SendSpeed(); pitch_msg.speed_dps = pitch
        self.shoulder_pub.publish(rail_msg)
        self.elbow_pub.publish(shoulder_msg)
        self.roll_pub.publish(roll_msg)
        self.pitch_pub.publish(pitch_msg)
        self.rail_pub.publish(rail_msg)


    def calculate_motor_speeds(self, joy_msg: Joy):
        wrist_speed = normalize_bumper_axes_vals(joy_msg.axes[WRIST_SPEED_AXES])
        shoulder_dps = floor(normalize_joystick_axes_vals(joy_msg.axes[SHOULDER_AXES]) * MAX_DPS * 0.5)
        elbow_dps = floor(normalize_joystick_axes_vals(joy_msg.axes[ELBOW_AXES]) * MAX_DPS * 0.3)
        roll_dps = int(joy_msg.axes[WRIST_ROLL_AXES] * wrist_speed * MAX_DPS * 3)
        pitch_dps = int(joy_msg.axes[WRIST_PITCH_AXES] * wrist_speed * MAX_DPS * -1)
        rail_dps = floor((joy_msg.buttons[RAIL_LEFT_BTN] - joy_msg.buttons[RAIL_RIGHT_BTN]) * 1000)
        self.send_speed_commands(rail_dps, shoulder_dps, elbow_dps, roll_dps, pitch_dps)
       
    
def main(args=None):
    rclpy.init(args=args)
    manipulator = Manipulator()
    rclpy.spin(manipulator)
    manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
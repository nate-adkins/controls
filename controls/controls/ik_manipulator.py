
import rclpy
from controls_msgs.msg import SpeedClosedLoopControlMsgSentParams as SendSpeed, ReadMotorStatus1MsgSentParams as SendStatus1, SystemResetMsgSentParams as Reset, SpeedClosedLoopControlMsgRecvParams as RecvSpeed, WriteEncoderMultiTurnZeroMsgSentParams as WriteAngle
from rclpy.node import Node
from sensor_msgs.msg import Joy
from math import floor, pi, cos, sin
from rclpy.publisher import Publisher

RESTART_BTN = 4 # Top Face Button
RAIL_LEFT_BTN = 6 # Left Bumper
RAIL_RIGHT_BTN = 7 # Right Buper
SHOULDER_AXES = 1 # Left Stick
ELBOW_AXES = 3 # Right Stick
WRIST_SPEED_AXES = 4 # Right Trigger
WRIST_ROLL_AXES = 6 # Left/Right D Pad
WRIST_PITCH_AXES = 7 #Up/Down D Pad 

TIMEOUT_DELAY_SEC = 2
TIMEOUT_CHECK_HZ = 2

MOTOR_STATUS_HZ = 1

MAX_DPS = 20

SHOULDER_LENGTH_MM = 437
ELBOW_LENGTH_MM = 462.25109
WRIST_LENGTH_MM = 236.45653

MAX_LINEAR_VEL_MPS = 0.05
SE_MIN = 20
EW_MIN = 20

def clamp(val) -> int:
    return max(-1,min(val,1))

def normalize_joystick_axes_vals(value: float):
    return clamp(-1 * value)
        
def normalize_bumper_axes_vals(value: float):
    return clamp(1 - ((value + 1)/2))

class Manipulator(Node):

    def __init__(self):
        super().__init__('manipulator')
    
    # PUBLISHERS

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

        # Zeroing Publishers
        self.shoulder_zero_pub = self.create_publisher(WriteAngle,"/manipulator/shoulder/send/reset",     10)
        self.elbow_zero_pub = self.create_publisher(WriteAngle,   "/manipulator/elbow/send/reset",        10)
        self.roll_zero_pub = self.create_publisher(WriteAngle,    "/manipulator/wrist_roll/send/reset",   10)
        self.pitch_zero_pub = self.create_publisher(WriteAngle,   "/manipulator/wrist_pitch/send/reset",  10)
        self.rail_zero_pub = self.create_publisher(WriteAngle,    "/manipulator/linear_rail/send/reset",  10)

        # Motor Angles Publishers
        self.pitch_angle_pub: Publisher = self.create_publisher(WriteAngle,     "/manipulator/wrist_pitch/send/write_zero_to_encoder",  10)
        self.shoulder_angle_pub: Publisher = self.create_publisher(WriteAngle,  "/manipulator/shoulder/send/write_zero_to_encoder",     10)
        self.elbow_angle_pub: Publisher = self.create_publisher(WriteAngle,     "/manipulator/elbow/send/write_zero_to_encoder",        10)

    # SUBSCRIBERS 

        # Angle Subscribers         
        self.create_subscription(RecvSpeed, "/manipulator/shoulder/rcvd/speed_control",     self.update_current_theta_s,    10)
        self.create_subscription(RecvSpeed, "/manipulator/elbow/rcvd/speed_control",        self.update_current_theta_e,    10)
        self.create_subscription(RecvSpeed, "/manipulator/wrist_pitch/rcvd/speed_control",  self.update_current_theta_w,    10)

        self.create_subscription(Joy,   "/joy",     self.calculate_motor_speeds,    10)
    
    # TIMERS 

        self.create_timer(1.0/TIMEOUT_CHECK_HZ, self.check_message_timeout)
        self.create_timer(1.0/MOTOR_STATUS_HZ,self.send_status_msgs)
        self.last_received_time = self.get_clock().now()

    # INITIALIZING MOTOR ANGLES

        def make_write_angle_msg(angle):
            msg = WriteAngle()
            msg.encoder_zero_offset = angle
            return msg
        
        initial_pitch_angle = 180
        initial_shoulder_angle = 90
        initial_elbow_angle = 180     

        self.pitch_angle_pub.publish(make_write_angle_msg(initial_pitch_angle))
        self.shoulder_angle_pub.publish(make_write_angle_msg(initial_shoulder_angle))
        self.elbow_angle_pub.publish(make_write_angle_msg(initial_elbow_angle))


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


    def update_current_theta_e(self, msg: RecvSpeed):
        self.current_theta_e = msg.angle_degrees


    def update_current_theta_w(self, msg: RecvSpeed):
        self.current_theta_w = msg.angle_degrees


    def update_current_theta_s(self, msg: RecvSpeed):
        self.current_theta_s = msg.angle_degrees


    def check_message_timeout(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_received_time).nanoseconds / 1e9 > TIMEOUT_DELAY_SEC:  
            self.get_logger().warning(f'No joystick control message received in the last {TIMEOUT_DELAY_SEC} seconds, zeroing motor speeds')
            self.send_speed_commands(0,0,0,0,0)
            self.last_received_time = self.get_clock().now()


    def calculate_motor_speeds(self, joy_msg: Joy):
        self.last_received_time = self.get_clock().now()

        if joy_msg[RESTART_BTN]:
            self.send_reset_msgs()

        def cosd(angle):
            return cos(180*angle/pi)
        def sind(angle):
            return sin(180*angle/pi)
        
        ThetaE = self.current_theta_e
        ThetaW = self.current_theta_w
        ThetaS = self.current_theta_s

        va = MAX_LINEAR_VEL_MPS * normalize_joystick_axes_vals(joy_msg.axes[SHOULDER_AXES]) * 0.5
        vb = MAX_LINEAR_VEL_MPS * normalize_joystick_axes_vals(joy_msg.axes[ELBOW_AXES]) * 0.5

        wrist_speed = normalize_bumper_axes_vals(joy_msg.axes[WRIST_SPEED_AXES])

        w3 = joy_msg.axes[WRIST_PITCH_AXES] * wrist_speed * MAX_DPS * -1
        ws = -(va*cosd(ThetaE - 180) + vb*sind(ThetaE - 180) + WRIST_LENGTH_MM*w3*cosd(ThetaE - 180)*sind(ThetaE + ThetaW - 360) - WRIST_LENGTH_MM*w3*sind(ThetaE - 180)*cosd(ThetaE + ThetaW - 360))/(SHOULDER_LENGTH_MM*(cosd(ThetaE - 180)*sind(ThetaS) - sind(ThetaE - 180)*cosd(ThetaS)))
        we = (va*cosd(ThetaS) + vb*sind(ThetaS) - WRIST_LENGTH_MM*w3*cosd(ThetaE + ThetaW - 360)*sind(ThetaS) + WRIST_LENGTH_MM*w3*sind(ThetaE + ThetaW - 360)*cosd(ThetaS))/(ELBOW_LENGTH_MM*(cosd(ThetaE - 180)*sind(ThetaS) - sind(ThetaE - 180)*cosd(ThetaS)))
        ww = w3 - we

        shoulder_dps = ws
        elbow_dps = we
        roll_dps = int(joy_msg.axes[WRIST_ROLL_AXES] * wrist_speed * MAX_DPS * 3)
        pitch_dps = ww
        rail_dps = floor((joy_msg.buttons[RAIL_LEFT_BTN] - joy_msg.buttons[RAIL_RIGHT_BTN]) * 1000)
        
        # if not any(max([we,ws,ww]) < MAX_DPS,ThetaS - ThetaE < SE_MIN,ThetaW < EW_MIN, ThetaW > 360 - EW_MIN ):

        self.send_speed_commands(rail_dps, shoulder_dps, elbow_dps, roll_dps, pitch_dps)
       
    
def main(args=None):
    rclpy.init(args=args)
    manipulator = Manipulator()
    rclpy.spin(manipulator)
    manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
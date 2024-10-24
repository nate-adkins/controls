from myactuator import SpeedClosedLoopControlMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from controls_interfaces.msg import CanMessage
import math, can

'''
TO DO:
- If not recieved messsage in x seconds, send zero speed
- If recieved many move msgs, then stopped, make sure to send zero speed
- Autodetect uart or can failure and switch to different system
'''

TRACK_WIDTH_M = 1.0
WHEEL_RADIUS = 1.0
GEAR_RATIO = 1/35

FRONT_LEFT_ID = 0x144
FRONT_RIGHT_ID = 0x142
BACK_LEFT_ID = 0x143
BACK_RIGHT_ID = 0x141

class DriveBaseControlNode(Node):

    def __init__(self):

        super().__init__('drivebase_control_node')

        self.cmd_vel_sub = self.create_subscription(
            msg_type= Twist,
            topic= "/cmd_vel",
            callback= self.send_drivebase_command,
            qos_profile= qos_profile_sensor_data
            )
        

    def send_drivebase_command(self, twist_msg: Twist):
        lin_vel = twist_msg.linear.x
        ang_vel = twist_msg.angular.z

        left_velocity = lin_vel - ang_vel * (TRACK_WIDTH_M / 2)
        right_velocity = lin_vel + ang_vel * (TRACK_WIDTH_M / 2)
        
        # Convert velocity to degrees per second for each wheel
        left_dps = (left_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360
        right_dps = (right_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360

    def send_can_message(can_command: can.Message):

        can_outgoing_ros_message = CanMessage()
        can_outgoing_ros_message.arbitration_id = can_command.arbitration_id
        can_outgoing_ros_message.data = can_command.data


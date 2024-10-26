from myactuator import SpeedClosedLoopControlMsg as Speed
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from controls_msgs.srv import CanSendRecv, UartSendRecv
import math

'''
TO DO:
- Change py constants to ros-accessible params
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

USING_CAN = True
USING_UART = False

assert USING_CAN != USING_UART

class DriveBaseControlNode(Node):

    def __init__(self):

        super().__init__('drivebase_control_node')

        self.cmd_vel_sub = self.create_subscription(
            msg_type= Twist,
            topic= "/cmd_vel",
            callback= self.send_drivebase_command,
            qos_profile= qos_profile_sensor_data
            )
        
        self.can_client = self.create_client(
            srv_type= CanSendRecv,
            srv_name= "/can_send_recv",
            qos_profile=10,
        )

        self.uart_client = self.create_client(
            srv_type= UartSendRecv,
            srv_name= "/uart_send_recv",
            qos_profile=10,
        )
        
    def send_drivebase_command(self, twist_msg: Twist):
        lin_vel = twist_msg.linear.x
        ang_vel = twist_msg.angular.z

        left_velocity = lin_vel - ang_vel * (TRACK_WIDTH_M / 2)
        right_velocity = lin_vel + ang_vel * (TRACK_WIDTH_M / 2)
        
        # Convert velocity to degrees per second for each wheel
        left_dps = (left_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360
        right_dps = (right_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360

        if USING_CAN:
            make_func = Speed.make_can_msg
            send_func = self.can_client.call_async

        if USING_UART:
            make_func = Speed.make_uart_msg
            send_func = self.uart_client.call_async
        
        else:
            raise ValueError(f'USING_CAN ({USING_CAN}) and USING_UART ({USING_UART} were not properly selected)')
    
    
        front_left_msg = make_func(FRONT_LEFT_ID,left_dps)
        front_right_msg = make_func(FRONT_RIGHT_ID,right_dps)
        back_left_msg = make_func(BACK_LEFT_ID,left_dps)
        back_right_msg = make_func(BACK_RIGHT_ID,right_dps)

        
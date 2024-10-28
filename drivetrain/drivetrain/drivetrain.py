from myactuator import SpeedClosedLoopControlMsg as Speed
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from controls_msgs.srv import CanSendRecv, UartSendRecv
import math, rclpy

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

from controls_msgs.msg import SpeedClosedLoopControlMsgSentParams

class Drivetrain(Node):

    def __init__(self):

        super().__init__('drivetrain')
        
        self.last_received_time = self.get_clock().now()
        self.check_timer = self.create_timer(1.0, self.check_message_timeout)

        self.cmd_vel_sub = self.create_subscription(
            msg_type= Twist,
            topic= "/cmd_vel",
            callback= self.send_drivebase_command,
            qos_profile= qos_profile_sensor_data
            )
        
        self.front_left_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/drivetrain/front_left/send/speed_control",
            10
        )
        
        self.front_right_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/drivetrain/front_right/send/speed_control",
            10
        )
        
        self.back_left_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/drivetrain/back_left/send/speed_control",
            10
        )
        
        self.back_right_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/drivetrain/back_right/send/speed_control",
            10
        )
        
    def send_drivebase_command(self, twist_msg: Twist):
        self.last_received_time = self.get_clock().now()

        lin_vel = twist_msg.linear.x
        ang_vel = twist_msg.angular.z

        left_velocity = lin_vel - ang_vel * (TRACK_WIDTH_M / 2)
        right_velocity = lin_vel + ang_vel * (TRACK_WIDTH_M / 2)
        
        left_dps = (left_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360
        right_dps = (right_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360
        
        left_msg = SpeedClosedLoopControlMsgSentParams()
        left_msg.speed_dps = left_dps
        
        right_msg = SpeedClosedLoopControlMsgSentParams()
        right_msg.speed_dps = right_dps

        self.front_left_pub.publish(left_msg)
        self.front_right_pub.publish(right_msg)
        self.back_left_pub.publish(left_msg)
        self.back_right_pub.publish(right_msg)
        
    def check_message_timeout(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_received_time).nanoseconds / 1e9 > 10:  
            self.get_logger().warning('No cmd_vel message received in the last 10 seconds, zeroing motors')
            
            zero_msg = SpeedClosedLoopControlMsgSentParams()
            zero_msg.speed_dps = 0
            self.front_left_pub.publish(zero_msg)
            self.front_right_pub.publish(zero_msg)
            self.back_left_pub.publish(zero_msg)
            self.back_right_pub.publish(zero_msg)
            self.last_received_time = self.get_clock().now()
            
def main(args=None):

    rclpy.init(args=args)
    drivetrain = Drivetrain()
    rclpy.spin(drivetrain)
    drivetrain.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
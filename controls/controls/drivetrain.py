from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from controls_msgs.msg import SpeedClosedLoopControlMsgSentParams as SendSpeed, ReadMotorStatus1MsgSentParams as SendStatus1
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

TIMEOUT_DELAY_SEC = 2
TIMEOUT_CHECK_HZ = 2

MOTOR_STATUS_HZ = 1

SPEED_COEFF = 100

DEBUGGING = False



class Drivetrain(Node):

    def __init__(self):

        super().__init__('drivetrain')
        
        self.cmd_vel_sub = self.create_subscription(Twist,"/cmd_vel",self.send_drivebase_command,qos_profile_sensor_data)
        
        # Speed Publishers
        self.front_left_pub = self.create_publisher(SendSpeed,  "/drivetrain/front_left/send/speed_control",    10)
        self.front_right_pub = self.create_publisher(SendSpeed, "/drivetrain/front_right/send/speed_control",   10)
        self.back_left_pub = self.create_publisher(SendSpeed,   "/drivetrain/back_left/send/speed_control",     10)
        self.back_right_pub = self.create_publisher(SendSpeed,  "/drivetrain/back_right/send/speed_control",    10)

        # Status Publishers 
        self.front_left_status1_pub = self.create_publisher(SendStatus1,"/drivetrain/front_left/send/status1",10)
        self.front_right_status1_pub = self.create_publisher(SendStatus1,"/drivetrain/front_right/send/status1",10)
        self.back_left_status1_pub = self.create_publisher(SendStatus1,"/drivetrain/back_left/send/status1",10)
        self.back_right_status1_pub = self.create_publisher(SendStatus1,"/drivetrain/back_right/send/status1",10)

        self.create_timer(1/MOTOR_STATUS_HZ,self.send_status_msgs)
        self.create_timer(1.0/TIMEOUT_CHECK_HZ, self.check_message_timeout)
        self.last_received_time = self.get_clock().now()

    def send_status_msgs(self):
        status_msg = SendStatus1()
        self.front_left_status1_pub.publish(status_msg)
        self.front_right_status1_pub.publish(status_msg)
        self.back_left_status1_pub.publish(status_msg)
        self.back_right_status1_pub.publish(status_msg)

    def send_speed_commands(self, left_dps, right_dps):
        right_msg = SendSpeed(); right_msg.speed_dps = right_dps
        left_msg = SendSpeed(); left_msg.speed_dps = left_dps

        self.front_left_pub.publish(left_msg)
        self.front_right_pub.publish(right_msg)
        self.back_left_pub.publish(left_msg)
        self.back_right_pub.publish(right_msg)

        
    def check_message_timeout(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_received_time).nanoseconds / 1e9 > TIMEOUT_DELAY_SEC:  
            self.get_logger().warning(f'No cmd_vel message received in the last {TIMEOUT_DELAY_SEC} seconds, zeroing motor speeds')
            self.send_speed_commands(0,0)
            self.last_received_time = self.get_clock().now() # repeated zeros has TIMEOUT_DELAY_SEC delay between them

        
    def send_drivebase_command(self, twist_msg: Twist):
        try:
            self.last_received_time = self.get_clock().now()

            lin_vel = twist_msg.linear.x
            ang_vel = twist_msg.angular.z

            left_velocity = lin_vel - ang_vel * (TRACK_WIDTH_M / 2)
            right_velocity = lin_vel + ang_vel * (TRACK_WIDTH_M / 2)
            
            left_dps = int(math.floor(SPEED_COEFF * (left_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360))
            right_dps = int(math.floor(SPEED_COEFF * (right_velocity / (2 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO * 360))

            if DEBUGGING:
                self.get_logger().info(f"left_dps{left_dps}\nright_dps{right_dps}")

            self.send_speed_commands(left_dps,right_dps)

        except Exception as e:
            self.get_logger().warning(f"Error in handling of drivetrain control {e.with_traceback()}, sending zero speeds")
            self.send_speed_commands(0,0)


            
def main(args=None):

    rclpy.init(args=args)
    drivetrain = Drivetrain()
    rclpy.spin(drivetrain)
    drivetrain.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
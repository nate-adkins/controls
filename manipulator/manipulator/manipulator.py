
import rclpy
from controls_msgs.msg import ManipulatorControl, SpeedClosedLoopControlMsgSentParams, ReadMotorStatus1MsgSentParams
from rclpy.node import Node

class Manipulator(Node):

    def __init__(self):
        super().__init__('manipulator')

        self.control_sub = self.create_subscription(
            msg_type= ManipulatorControl,
            topic= self.get_parameter('manipulator_control_topic').get_parameter_value().string_value,
            callback= self.send_arm_command,
            qos_profile= 1
            )
        

        # Speed Publishers         
        self.shoulder_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/manipulator/shoulder/send/speed_control",
            10
        )
        
        self.elbow_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/manipulator/elbow/send/speed_control",
            10
        )
        
        self.roll_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/manipulator/wrist_roll/send/speed_control",
            10
        )
        
        self.pitch_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/manipulator/wrist_pitch/send/speed_control",
            10
        )

        self.pitch_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/manipulator/wrist_pitch/send/speed_control",
            10
        )

        self.rail_pub = self.create_publisher(
            SpeedClosedLoopControlMsgSentParams,
            "/manipulator/linear_rail/send/speed_control",
            10
        )

       
        
    def send_arm_command(self, manipulator_control_msg):


        pass

    
def main(args=None):
    rclpy.init(args=args)
    manipulator = Manipulator()
    rclpy.spin(manipulator)
    manipulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
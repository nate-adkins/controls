import rclpy
from rclpy.node import Node
from controls_msgs.msg import * # CanMessage, SpeedClosedLoopControlMsgSentParams, SpeedClosedLoopControlMsgRecvParams
from myactuator import * # SpeedClosedLoopControlMsg, ReadMotorStatus1Msg, ReadMotorStatus2Msg
import can

class MotorNode(Node):
    def __init__(self):
        super().__init__(node_name='default_motor', namespace='/default_motor')
        
        self.declare_parameter('arbitration_id', 0x141)
        
        # Publisher/Subscriber Pairs for Can Node    
        self.pub = self.create_publisher(CanMessage,'/can_interface/send',10)
        self.sub = self.create_subscription(CanMessage,'/can_interface/rcvd',self.parse_responses, 10)
        
    
        # Publisher/Subscriber Pairs for Motor Data
        self.create_subscription(SystemResetMsgSentParams,'send/reset',self.send_reset,10)
        self.reset_pub = self.create_publisher(SystemResetMsgRecvParams,'rcvd/reset',10)   
        
        self.create_subscription(SpeedClosedLoopControlMsgSentParams,'send/speed_control',self.send_speed,10)
        self.speed_pub = self.create_publisher(SpeedClosedLoopControlMsgRecvParams,'rcvd/speed_control',10)
        
        self.create_subscription(WriteCurrentMultiTurnZeroMsgSentParams,'send/write_current_encoder_as_zero',self.send_current_encoder_zero,10)
        self.current_encoder_zero_pub = self.create_publisher(WriteCurrentMultiTurnZeroMsgRecvParams,'rcvd/write_current_encoder_as_zero',10)
        
        self.create_subscription(WriteEncoderMultiTurnZeroMsgSentParams,'send/write_zero_to_encoder',self.send_encoder_zero,10)
        self.encoder_zero_pub = self.create_publisher(WriteEncoderMultiTurnZeroMsgRecvParams,'rcvd/write_zero_to_encoder',10)
        
        self.create_subscription(ReadMotorStatus1MsgSentParams,'send/status1',self.send_status1,10)
        self.status1_pub = self.create_publisher(ReadMotorStatus1MsgRecvParams,'rcvd/status1',10)
        
        self.create_subscription(ReadMotorStatus2MsgSentParams,'send/status2',self.send_status2,10)
        self.status2_pub = self.create_publisher(ReadMotorStatus2MsgRecvParams,'rcvd/status2',10)
        
        self.create_subscription(ReadMotorStatus3MsgSentParams,'send/status3',self.send_status3,10)
        self.status3_pub = self.create_publisher(ReadMotorStatus3MsgRecvParams,'rcvd/status3',10)
        
    def can_to_ros(self, can_msg: can.Message) -> CanMessage:
        '''convert can.Message to ros CanMessage class'''
        ros_can_message = CanMessage()
        ros_can_message.arbitration_id = can_msg.arbitration_id
        ros_can_message.data = can_msg.data
        return ros_can_message
        
    def parse_responses(self, rcvd_can_msg: CanMessage):
        
        curr_motor_id = self.get_parameter('arbitration_id').get_parameter_value().integer_value
        if curr_motor_id + 0x100 == rcvd_can_msg.arbitration_id:
            cmd_byte = rcvd_can_msg.data[0]
            
            if cmd_byte == SystemResetMsg._cmd_byte:
                __, mappings = SystemResetMsg.parse_can_msg(rcvd_can_msg)
                pub_param_msg = SystemResetMsgRecvParams()
                self.reset_pub.publish(pub_param_msg)

            if cmd_byte == SpeedClosedLoopControlMsg._cmd_byte:
                __, mappings = SpeedClosedLoopControlMsg.parse_can_msg(rcvd_can_msg)
                pub_param_msg = SpeedClosedLoopControlMsgRecvParams()
                pub_param_msg.motor_temperature_c = mappings.get('motor_temperature_c')
                pub_param_msg.current_amps = mappings.get('current_amps')
                pub_param_msg.speed_dps = mappings.get('speed_dps')
                pub_param_msg.angle_degrees = mappings.get('angle_degrees')
                self.speed_pub.publish(pub_param_msg)
                
            if cmd_byte == WriteCurrentMultiTurnZeroMsg._cmd_byte:
                __, mappings = WriteCurrentMultiTurnZeroMsg.parse_can_msg(rcvd_can_msg)
                pub_param_msg = WriteCurrentMultiTurnZeroMsgRecvParams()
                pub_param_msg.encoder_zero_offset = mappings.get('encoder_zero_offset')
                self.current_encoder_zero_pub.publish(pub_param_msg)
                
            if cmd_byte == WriteEncoderMultiTurnZeroMsg._cmd_byte:
                __, mappings = WriteEncoderMultiTurnZeroMsg.parse_can_msg(rcvd_can_msg)
                pub_param_msg = WriteEncoderMultiTurnZeroMsgRecvParams()
                pub_param_msg.encoder_zero_offset
                pub_param_msg.encoder_zero_offset = mappings.get('encoder_zero_offset')
                self.encoder_zero_pub.publish(pub_param_msg)
                
            if cmd_byte == ReadMotorStatus1Msg._cmd_byte:
                __, mappings = ReadMotorStatus1Msg.parse_can_msg(rcvd_can_msg)
                pub_param_msg = ReadMotorStatus1MsgRecvParams()
                pub_param_msg.motor_temperature_c = mappings.get('motor_temperature_c')
                pub_param_msg.mos_temperature = mappings.get('mos_temperature')
                pub_param_msg.break_state = mappings.get('break_state')
                pub_param_msg.voltage_volts = mappings.get('voltage_volts')
                pub_param_msg.error_status = mappings.get('error_status')
                self.status1_pub.publish(pub_param_msg)
                
            if cmd_byte == ReadMotorStatus2Msg._cmd_byte:
                __, mappings = ReadMotorStatus2Msg.parse_can_msg(rcvd_can_msg)
                pub_param_msg = ReadMotorStatus2MsgRecvParams()
                pub_param_msg.motor_temperature_c = mappings.get('motor_temperature_c')
                pub_param_msg.current_amps = mappings.get('current_amps')
                pub_param_msg.speed_dps = mappings.get('speed_dps')
                pub_param_msg.angle_degrees = mappings.get('angle_degrees')
                self.status2_pub.publish(pub_param_msg)

            if cmd_byte == ReadMotorStatus3Msg._cmd_byte:
                __, mappings = ReadMotorStatus3Msg.parse_can_msg(rcvd_can_msg)
                pub_param_msg = ReadMotorStatus3MsgRecvParams()
                pub_param_msg.motor_temperature_c = mappings.get('motor_temperature_c')
                pub_param_msg.phase_a_current_amps = mappings.get('phase_a_current_amps')
                pub_param_msg.phase_b_current_amps = mappings.get('phase_b_current_amps')
                pub_param_msg.phase_c_current_amps = mappings.get('phase_c_current_amps')
                self.status3_pub.publish(pub_param_msg)
    
    def send_reset(self, msg: SystemResetMsgSentParams):
        id = self.get_parameter('arbitration_id').get_parameter_value().integer_value
        ros_can_msg = self.can_to_ros(SystemResetMsg.make_can_msg(id))
        self.pub.publish(ros_can_msg)
    
    def send_speed(self, msg: SpeedClosedLoopControlMsgSentParams):
        id = self.get_parameter('arbitration_id').get_parameter_value().integer_value
        ros_can_msg = self.can_to_ros(SpeedClosedLoopControlMsg.make_can_msg(id, msg.speed_dps))
        self.pub.publish(ros_can_msg) 
           
    def send_current_encoder_zero(self, msg: WriteCurrentMultiTurnZeroMsgSentParams):
        id = self.get_parameter('arbitration_id').get_parameter_value().integer_value
        ros_can_msg = self.can_to_ros(WriteCurrentMultiTurnZeroMsg.make_can_msg(id))
        self.pub.publish(ros_can_msg) 
    
    def send_encoder_zero(self, msg: WriteEncoderMultiTurnZeroMsgSentParams):
        id = self.get_parameter('arbitration_id').get_parameter_value().integer_value
        ros_can_msg = self.can_to_ros(WriteEncoderMultiTurnZeroMsg.make_can_msg(id, msg.encoder_zero_offset))
        self.pub.publish(ros_can_msg) 
      
    def send_status1(self, msg):
        id = self.get_parameter('arbitration_id').get_parameter_value().integer_value
        ros_can_msg = self.can_to_ros(ReadMotorStatus1Msg.make_can_msg(id,))
        self.pub.publish(ros_can_msg) 
    
    def send_status2(self, msg):
        id = self.get_parameter('arbitration_id').get_parameter_value().integer_value
        ros_can_msg = self.can_to_ros(ReadMotorStatus2Msg.make_can_msg(id,))
        self.pub.publish(ros_can_msg) 
    
    def send_status3(self, msg):
        id = self.get_parameter('arbitration_id').get_parameter_value().integer_value
        ros_can_msg = self.can_to_ros(ReadMotorStatus3Msg.make_can_msg(id,))
        self.pub.publish(ros_can_msg) 
     
def main():  
    rclpy.init()
    motor_node = MotorNode()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
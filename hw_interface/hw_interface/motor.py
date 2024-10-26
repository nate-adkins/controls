import rclpy
from rclpy.node import Node
from controls_msgs.srv import CanSendRecv, SpeedClosedLoopControlMsg

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # Create service to handle incoming requests
        self.speed_srv = self.create_service(
            SpeedClosedLoopControlMsg,
            '/hello',
            self.process
        )
        
        # Create a client for the CAN interface service
        self.can_client = self.create_client(
            CanSendRecv,
            '/can_send_recv',
        )

    def process(self, request, response):
        # Create CAN service request
        can_req = CanSendRecv.Request()
        can_req.arbitration_id = 1
        can_req.can_data = [1, 1, 1]

        # Send async request to CAN service
        future = self.can_client.call_async(can_req)
        
        # Attach a callback to handle the future result
        future.add_done_callback(lambda f: self.process_can_req(f, response))
        
        # Immediately return the response as required by the callback structure
        return response  

    def process_can_req(self, future, response):
        # Process the response from CAN service
        try:
            curr_response = future.result()
            response.motor_temperature_c = float(curr_response.arbitration_id)
            response.current_amps = 1.0
            response.speed_dps = 1.0
            response.angle_degrees = 1.0
            self.get_logger().info('Successfully received CAN response')
        
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main():
    rclpy.init()
    motor_node = MotorNode()
    rclpy.spin(motor_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

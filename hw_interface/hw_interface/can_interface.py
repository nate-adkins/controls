import rclpy, threading, can
from rclpy.node import Node
from controls_msgs.srv import CanSendRecv

TIMEOUT_MS = 0.01

class CanInterfaceNode(Node):

    thread_lock = threading.Lock()

    def __init__(self):

        super().__init__('can_interface_node')

        self.can_service = self.create_service(
            CanSendRecv,
            '/can_send_recv',
            self.send_can_command
        )

        # self.bus = can.Bus(
        #     interface       = 'socketcan', 
        #     channel         = 'can0', 
        #     is_extended_id  = False, 
        #     bitrate         = 1000000, 
        # )

    def send_can_command(self, request, response):

        new_msg = can.Message(
            arbitration_id=request.arbitration_id,
            data=request.can_data,
            is_extended_id=False,
            dlc=8,
        )

        # Delays and timings may need to be played with here
        # with CanInterfaceNode.thread_lock:
        #     self.bus.send(new_msg,TIMEOUT_MS)
        #     recv_msg = self.bus.recv(TIMEOUT_MS)
        
        response = CanSendRecv.Response()
        response.arbitration_id = 69  # recv_msg.arbitration_id
        response.can_data = [1,1] # recv_msg.data

        return response
        
def main():
    rclpy.init()
    can_interface_node = CanInterfaceNode()
    rclpy.spin(can_interface_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
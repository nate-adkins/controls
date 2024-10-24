import rclpy, threading, serial
from rclpy.node import Node
from controls_interfaces.srv import CanSendRecv

def make_uart_port(port):
    return serial.Serial(
        port = port,
        baudrate=115200,
        timeout=3,
    )


class UartInterfaceNode(Node):

    port_name_mappings = {

        # Drivebase
        0x141 : '/dev/ttyUSB0',
        0x142 : '/dev/ttyUSB1',
        0x143 : '/dev/ttyUSB2',
        0x144 : '/dev/ttyUSB3',

        # Manipulator
        0x145 : '/dev/ttyUSB4',
        0x146 : '/dev/ttyUSB5',
        0x147 : '/dev/ttyUSB6',
        0x148 : '/dev/ttyUSB7',
        0x149 : '/dev/ttyUSB8',

    }

    port_mappings = 

    thread_lock = threading.Lock()

    def __init__(self):

        super().__init__('uart_interface_node')

        self.can_service = self.create_service(
            CanSendRecv,
            '/uart_send_recv',
        )

    def send_can_command(self, request, response):

        new_msg = can.Message(
            arbitration_id=request.arbitration_id,
            data=request.data,
            is_extended_id=False,
            dlc=8,
        )

        # Delays and timings may need to be played with here
        with UartInterfaceNode.thread_lock:
            self.bus.send(new_msg,TIMEOUT_MS)
            recv_msg = self.bus.recv(TIMEOUT_MS)

        response.arbitration_id = recv_msg.arbitration_id
        response.data = recv_msg.data

        return response
        
def main():
    rclpy.init()
    uart_interface_node = UartInterfaceNode()
    rclpy.spin(uart_interface_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
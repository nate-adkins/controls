import rclpy
from rclpy.node import Node
import controls_msgs.msg
import inspect

def list_all_message_classes():
    print("Available message classes in controls_msgs.msg:")
    for name, obj in inspect.getmembers(controls_msgs.msg):
        if inspect.isclass(obj) and not name.startswith("_"):
            print(name)

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        list_all_message_classes()

def main():
    rclpy.init()
    motor_node = MotorNode()
    rclpy.spin(motor_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

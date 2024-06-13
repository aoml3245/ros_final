import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MessagePrinter(Node):

    def __init__(self):
        super().__init__('message_printer')
        self.srv = self.create_service(Trigger, 'print_message', self.print_message_callback)

    def print_message_callback(self, request, response):
        self.get_logger().info('Hello, World!')
        response.success = True
        response.message = 'Hello, World!'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MessagePrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

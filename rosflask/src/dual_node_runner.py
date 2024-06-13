import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from flask import Flask, request
import threading

class FlaskServer(Node):

    def __init__(self):
        super().__init__('flask_server')
        self.cli = self.create_client(Trigger, 'print_message')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available?, waiting again...')
        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

app = Flask(__name__)
ros_node = None

@app.route('/hello', methods=['GET'])
def hello():
    if ros_node:
        response = ros_node.send_request()
        if response.success:
            return "Message printed: " + response.message
        else:
            return "Failed to print message"
    else:
        return "ROS node not initialized", 500

class MessagePrinter(Node):

    def __init__(self):
        super().__init__('message_printer')
        self.srv = self.create_service(Trigger, 'print_message', self.print_message_callback)

    def print_message_callback(self, request, response):
        self.get_logger().info('Hello, World!')
        response.success = True
        response.message = 'Hello, World!'
        return response

def run_flask_app():
    # Flask 서버 실행
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    global ros_node
    rclpy.init(args=args)

    message_printer_node = MessagePrinter()

    flask_server_node = FlaskServer()
    ros_node = flask_server_node

    executor = MultiThreadedExecutor()
    executor.add_node(message_printer_node)
    executor.add_node(flask_server_node)
    
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.start()
    
    
    try:
        print("excute spin()")
        executor.spin()
    finally:
        executor.shutdown()
        flask_server_node.destroy_node()
        message_printer_node.destroy_node()
        rclpy.shutdown()
        flask_thread.join()

if __name__ == '__main__':
    main()
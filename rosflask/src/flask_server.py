import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from flask import Flask

app = Flask(__name__)
ros_node = None

class FlaskServer(Node):

    def __init__(self):
        super().__init__('flask_server')
        self.cli = self.create_client(Trigger, 'print_message')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

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

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = FlaskServer()
    app.run(host='0.0.0.0', port=5000)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from flask import Flask, request, jsonify
import threading
import requests

class FlaskServer(Node):

    def __init__(self):
        super().__init__('flask_server')
        self.publisher_ = self.create_publisher(String, 'command_topic', 10)

    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)

app = Flask(__name__)
ros_node = None
url_storage = ""

@app.route('/hello', methods=['GET'])
def hello():
    if ros_node:
        ros_node.send_command("print_message")
        return "Message command sent"
    else:
        return "ROS node not initialized", 500

@app.route('/seturl', methods=['POST'])
def set_url():
    global url_storage
    data = request.get_json()
    if 'url' in data:
        url_storage = data['url']
        return jsonify({"message": "URL set successfully"}), 200
    else:
        return jsonify({"error": "No URL provided"}), 400

@app.route('/move', methods=['POST'])
def move():
    data = request.get_json()
    if 'command' in data:
        command = data['command']
        if ros_node:
            ros_node.send_command(f"move {command}")
            return jsonify({"message": "Move command sent"}), 200
        else:
            return jsonify({"error": "ROS node not initialized"}), 500
    else:
        return jsonify({"error": "No command provided"}), 400

@app.route('/movedir', methods=['POST'])
def move_dir():
    data = request.get_json()
    if 'direction' in data:
        direction = data['direction']
        if ros_node:
            ros_node.send_command(f"movedir {direction}")
            return jsonify({"message": "Move direction command sent"}), 200
        else:
            return jsonify({"error": "ROS node not initialized"}), 500
    else:
        return jsonify({"error": "No direction provided"}), 400

class ROSController(Node):

    def __init__(self):
        super().__init__('ros_controller')
        self.subscription = self.create_subscription(String, 'command_topic', self.listener_callback, 10)
        self.url = ""

    def listener_callback(self, msg: String):
        command = msg.data.split()
        if command[0] == "print_message":
            self.print_message()
        elif command[0] == "move":
            self.send_move_command(command[1])
        elif command[0] == "movedir":
            self.send_move_direction(int(command[1]))

    def print_message(self):
        self.get_logger().info('Hello, World!')

    def send_move_command(self, command: str):
        if url_storage:
            full_url = f"http://{url_storage}/command?commandText=G1%20{command.replace(' ', '%20')}"
            response = requests.get(full_url)
            self.get_logger().info(f"Move command response: {response.text}")
        else:
            self.get_logger().error("URL not set")

    def send_move_direction(self, direction: int):
        if url_storage:
            if direction == 0:
                command_text = "G92%20X0%20Y0"
            elif direction == 1:
                command_text = "G1%20X10%20Y10"
            elif direction == -1:
                command_text = "G1%20X-10%20Y-10"
            else:
                self.get_logger().error("Invalid direction")
                return
            full_url = f"http://{url_storage}/command?commandText={command_text}"
            response = requests.get(full_url)
            self.get_logger().info(f"Move direction response: {response.text}")
        else:
            self.get_logger().error("URL not set")

def run_flask_app():
    # Flask 서버 실행
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    global ros_node
    rclpy.init(args=args)

    ros_controller_node = ROSController()

    flask_server_node = FlaskServer()
    ros_node = flask_server_node

    executor = MultiThreadedExecutor()
    executor.add_node(ros_controller_node)
    executor.add_node(flask_server_node)
    
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.start()
    
    try:
        print("execute spin()")
        executor.spin()
    finally:
        executor.shutdown()
        flask_server_node.destroy_node()
        ros_controller_node.destroy_node()
        rclpy.shutdown()
        flask_thread.join()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from std_srvs.srv import Trigger
# from flask import Flask, request
# import threading

# class FlaskServer(Node):

#     def __init__(self):
#         super().__init__('flask_server')
#         self.cli = self.create_client(Trigger, 'print_message')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available?, waiting again...')
#         self.req = Trigger.Request()

#     def send_request(self):
#         self.future = self.cli.call_async(self.req)
#         rclpy.spin_until_future_complete(self, self.future)
#         return self.future.result()

# app = Flask(__name__)
# ros_node = None
# url_storage = ""

# @app.route('/hello', methods=['GET'])
# def hello():
#     if ros_node:
#         response = ros_node.send_request()
#         if response.success:
#             return "Message printed: " + response.message
#         else:
#             return "Failed to print message"
#     else:
#         return "ROS node not initialized", 500

# @app.route('/seturl', methods=['POST'])
# def set_url():
#     global url_storage
#     data = request.get_json()
#     if 'url' in data:
#         url_storage = data['url']
#         return jsonify({"message": "URL set successfully"}), 200
#     else:
#         return jsonify({"error": "No URL provided"}), 400

# @app.route('/move', methods=['POST'])
# def move():
#     if not url_storage:
#         return jsonify({"error": "URL not set"}), 400
#     data = request.get_json()
#     if 'command' in data:
#         command = data['command']
#         full_url = f"http://{url_storage}/command?commandText=G1%20{command.replace(' ', '%20')}"
#         response = requests.get(full_url)
#         return jsonify({"response": response.text}), response.status_code
#     else:
#         return jsonify({"error": "No command provided"}), 400

# @app.route('/movedir', methods=['POST'])
# def move_dir():
#     if not url_storage:
#         return jsonify({"error": "URL not set"}), 400
#     data = request.get_json()
#     if 'direction' in data:
#         direction = data['direction']
#         if direction == 0:
#             command_text = "G92%20X0%20Y0"
#         elif direction == 1:
#             command_text = "G1%20X10%20Y10"
#         elif direction == -1:
#             command_text = "G1%20X-10%20Y-10"
#         else:
#             return jsonify({"error": "Invalid direction"}), 400
#         full_url = f"http://{url_storage}/command?commandText={command_text}"
#         response = requests.get(full_url)
#         return jsonify({"response": response.text}), response.status_code
#     else:
#         return jsonify({"error": "No direction provided"}), 400


# class MessagePrinter(Node):

#     def __init__(self):
#         super().__init__('message_printer')
#         self.subscription = self.create_subscription(String, 'command_topic', self.listener_callback, 10)
#         self.url = ""

#     def listener_callback(self, msg: String):
#         command = msg.data.split()
#         if command[0] == "print_message":
#             self.print_message()
#         elif command[0] == "move":
#             self.send_move_command(command[1])
#         elif command[0] == "movedir":
#             self.send_move_direction(int(command[1]))

#     def print_message(self):
#         self.get_logger().info('Hello, World!')

#     def send_move_command(self, command: str):
#         if self.url:
#             full_url = f"http://{self.url}/command?commandText=G1%20{command.replace(' ', '%20')}"
#             response = requests.get(full_url)
#             self.get_logger().info(f"Move command response: {response.text}")
#         else:
#             self.get_logger().error("URL not set")

#     def send_move_direction(self, direction: int):
#         if self.url:
#             if direction == 0:
#                 command_text = "G92%20X0%20Y0"
#             elif direction == 1:
#                 command_text = "G1%20X10%20Y10"
#             elif direction == -1:
#                 command_text = "G1%20X-10%20Y-10"
#             else:
#                 self.get_logger().error("Invalid direction")
#                 return
#             full_url = f"http://{self.url}/command?commandText={command_text}"
#             response = requests.get(full_url)
#             self.get_logger().info(f"Move direction response: {response.text}")
#         else:
#             self.get_logger().error("URL not set")

# def run_flask_app():
#     # Flask 서버 실행
#     app.run(host='0.0.0.0', port=5000)

# def main(args=None):
#     global ros_node

#     rclpy.init(args=args)

#     message_printer_node = MessagePrinter()

#     flask_server_node = FlaskServer()
#     ros_node = flask_server_node

#     executor = MultiThreadedExecutor()
#     executor.add_node(message_printer_node)
#     executor.add_node(flask_server_node)
    
#     flask_thread = threading.Thread(target=run_flask_app)
#     flask_thread.start()
    
    
#     try:
#         print("excute spin()")
#         executor.spin()
#     finally:
#         executor.shutdown()
#         flask_server_node.destroy_node()
#         message_printer_node.destroy_node()
#         rclpy.shutdown()
#         flask_thread.join()

# if __name__ == '__main__':
#     main()
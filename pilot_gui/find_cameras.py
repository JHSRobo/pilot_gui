import rclpy
from rclpy.node import Node
import flask
from core.srv import AddCamera
import logging

class Camera_Finder(Node):

    def __init__(self):
        super().__init__('camera_finder')
        
        self.log = self.get_logger() # Quick reference for ROS logging

        # Suppress the flask warning for development server
        # Not a big deal, because all other errors are handled through ROS
        # And those warnings are not suppressed by this.
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        # Dict of ips that are already recognized and added
        # Is a dict so that we can store timers in there to monitor connection
        self.known_ips = {}

        # Set up the add camera service
        self.camera_adder = self.create_client(AddCamera, 'AddCamera')
        self.request = AddCamera.Request()

        # Run the app
        self.find_cameras()

    def find_cameras(self):

        # Create an application with flask
        app = flask.Flask(__name__)

        # Write the Flask Callback
        # Yes, I know it's gross to define a function within a function.
        # There are cleaner ways to do this, but this is intelligible and works well.
        @app.route('/', methods=["POST", "GET"])
        def page():

            incoming_ip = flask.request.remote_addr

            # If the IP is new, add it to known_ips and send it out to the camera viewer.
            if incoming_ip not in self.known_ips.keys():# and len(incoming_form) > 0:
                self.known_ips[incoming_ip] = None
                self.add_cam(incoming_ip)

            return ""
        app.run(host='0.0.0.0', port=12345)
    

    def add_cam(self, new_ip):
        self.request.ip = str(new_ip)
        self.future = self.camera_adder.call_async(self.request)


def main(args=None):
    rclpy.init(args=args)

    camera_finder = Camera_Finder()

    # Runs the program until shutdown is recieved
    rclpy.spin(camera_finder)

    # On shutdown, kill node
    camera_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
import flask
from core.srv import AddCamera

class Camera_Finder(Node):

    def __init__(self):
        super().__init__('camera_finder')
        
        self.log = self.get_logger() # Quick reference for logging

        self.known_ips = [] # Index of ips that are already recognized and added

        # Set up the add camera service
        self.client = self.create_client(AddCamera, 'AddCamera')
        self.request = AddCamera.Request() 
    
    def add_cams(self, new_ip):
        self.request.ip = str(new_ip)
        self.future = self.client.call_async(self.req)

    def find_cameras(self):

        # Create an application with flask
        app = flask.Flask(__name__)

        # Write the Flask Callback
        # Yes, I know it's gross to define a function within a function.
        # There are cleaner ways to do this, but this is intelligible and works well.

        @app.route('/', methods=["POST", "GET"])
        def page():
            incoming_ip = flask.request.remote_addr
            incoming_form = flask.request.form

            # If the IP is new, add it to known_ips and send it out to the camera viewer.
            if incoming_ip not in self.known_ips and len(incoming_form) > 0:
                self.log.info("Detected new camera. IP: {}".format(incoming_ip))
                self.known_ips.append(incoming_ip)
                self.add_cams(incoming_ip)

            return ""
        
        app.run(host='0.0.0.0', port=12345)


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

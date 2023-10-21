import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Camera_Finder(Node):

    def __init__(self):
        super().__init__('camera_finder')
        
        self.log = self.get_logger() # Quick reference for logging

        self.known_ips = [] # Index of ips that are already recognized and added


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

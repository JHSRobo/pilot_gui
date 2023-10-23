import json
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from core.srv import AddCamera
from core.msg import Cam

class Camera_Switcher(Node):

    def __init__(self):
        super().__init__('camera_switcher')
        
        self.log = self.get_logger() # Quick reference for ROS logging

        # Stores IPs of active cameras
        # 0 means slot is unused
        self.active_cameras = { i + 1 : 0 for i in range(8)}
        self.current_camera_index = 0

        self.camera_adder = self.create_service(AddCamera, "AddCamera", self.add_camera_callback)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.change_cam_callback, 10)
        self.camera_pub = self.create_publisher(Cam, "active_camera", 10)

        # For remembering previous camera button presses
        # This way we only try to change when the button is first pushed
        # And not repeatedly when it is held down
        self.cached_button_input = [0, 0, 0, 0]
        self.cached_camera_index = None

        # Define config file path
        self.config_path = os.path.join(os.path.dirname(__file__), "../cam_config.json")

        self.check_config_integrity()
        self.open_config()


    # When the AddCamera service is requested (from find_cameras.py), 
    # add the attached IP to the camera dict.
    def add_camera_callback(self, request, response):

        # Check to see if this IP is pre-registered in the config file
        ip_index = self.get_ip_index(request.ip)
        if ip_index is not None:
            self.log.info("Camera @ {} assigned to index {}".format(request.ip, ip_index))
        else:
            self.create_config_entry(request.ip)
            ip_index = self.get_ip_index(request.ip) # Get new index once it's been added to config
            
        self.active_cameras[int(ip_index)] = request.ip

        return response


    # Monitor controller input for D-Pad press
    # Used to change cameras
    def change_cam_callback(self, joy):

        change = False

        if joy.axes[6] or joy.axes[7]:
            # Minor consequence of this logic is:
            # If multiple buttons are pressed, the furthest clockwise takes priority
            # No big deal tbh. About as reasonable a solution as any.

            # Also, this cached input stuff is so that holding the button does not trigger this repeatedly.
            if joy.axes[7] == 1.0 and not self.cached_button_input[0]: # Up button
                self.current_camera_index = 1
                change = True
            if joy.axes[6] == -1.0 and not self.cached_button_input[1]: # Right Button
                self.current_camera_index = 2
                change = True
            if joy.axes[7] == -1.0 and not self.cached_button_input[2]: # Down button
                self.current_camera_index = 3
                change = True
            if joy.axes[6] == 1.0 and not self.cached_button_input[3]: # Left Button
                self.current_camera_index = 4
                change = True
            
            camera_msg = self.create_camera_msg()

            # Check if the selected index has a camera mapped to it
            if change: 
                if self.active_cameras[self.current_camera_index]:
                    if self.cached_camera_index is not self.current_camera_index:
                        self.camera_pub.publish(camera_msg)
                else: 
                    self.log.warn("No active camera mapped to that button")
        
        self.cached_button_input = [joy.axes[7], joy.axes[6], joy.axes[7], joy.axes[6]]
        self.cached_camera_index = self.current_camera_index

    # Populate a camera msg with all of the info about the current camera
    def create_camera_msg(self):
        index = str(self.current_camera_index)
        camera_msg = Cam()
        camera_msg.index = int(index)
        camera_msg.ip = self.camera_config[index]["ip"]
        camera_msg.gripper = self.camera_config[index]["gripper"]
        camera_msg.nickname = self.camera_config[index]["nickname"]

        return camera_msg


    # Read the existing camera config into a dictionary
    # JSON files are read into nested dictionaries.
    # Also, keep in mind that although the key values are numbers 1-4, they are strings, not ints.
    def open_config(self):
        with open(self.config_path) as f:
            self.camera_config = json.load(f)


    # Checks if the config file still exists.
    # If it doesn't, it creates a new one.
    def check_config_integrity(self):
        if os.path.isfile(self.config_path): 
            return True
        else:
            empty_dict = {}
            with open(self.config_path, "w") as f:
                json.dump(empty_dict, f)


    # Scans the config for a certain ip in inner dictionaries
    # Returns the index of that IP if found
    def get_ip_index(self, value):
        for index in self.camera_config:
            for key in self.camera_config[index]:
                if self.camera_config[index][key] == value:
                    return index
        return None
    

    def create_config_entry(self, ip, gripper="front", nickname="unnamed"):
        index = len(self.camera_config) + 1 # Assign a new index
        self.camera_config[index] = { "ip" : ip,
                                      "gripper" : gripper,
                                      "nickname" : nickname }

        self.write_to_config()
        self.log.info("Created new camera entry. Camera @ {} assigned to index {}".format(ip, index))
    

    def write_to_config(self):
        config_json = json.dumps(self.camera_config, indent=2)
        with open(self.config_path, "w") as f:
            f.write(config_json)


def main(args=None):
    rclpy.init(args=args)

    camera_switcher = Camera_Switcher()

    # Runs the program until shutdown is recieved
    rclpy.spin(camera_switcher)

    # On shutdown, kill node
    camera_switcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

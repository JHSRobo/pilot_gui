import rclpy
from rclpy.node import Node
from core.msg import Cam, Sensitivity
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Trigger, SetBool
from core_lib import camera_overlay
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge

import time
import toml
import cv2

class Camera_Viewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')
        
        self.log = self.get_logger() # Quick reference for ROS logging

        self.vid_capture = None # Variable for storing connection to camera stream

        # Create a client and sub for getting sensitivity
        self.first_sense_request = self.create_client(Trigger, "first_sensitivity")
        self.sensitivity_sub = self.create_subscription(Sensitivity, "sensitivity", self.sensitivity_callback, 10)
        self.camera_pub = self.create_publisher(Cam, "active_camera", 10) # published information about current camera

        # Create a service for adjusting thruster status
        self.thruster_status_srv = self.create_service(SetBool, 'thruster_status', self.thruster_status_callback)

        # Create a joystick subscriber for switching cameras
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Last frame's buttons pressed
        self.cached_button_input = [0, 0, 0, 0, 0]

        # Get the number of cameras to receive streams from 
        self.camera_count = -1
        self.compared_to_config = False
        self.camera_count_subscriber = self.create_subscription(Int32, 'camera_count', self.camera_count_callback, 10)

        # Create window used for displaying camera feed
        cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

        self.config_path = "/home/jhsrobo/corews/src/pilot_gui/cam_config.toml"
        # Open Config File for Cameras
        try:
            with open(self.config_path, "r") as f:
                self.config = toml.load(f)
                self.log.info(str(self.config))

        except FileNotFoundError:
            self.log.info("Could not find cam_config.toml...")
            self.log.info("Generating new cam_config.toml...")
            self.config = {}

        # # Create variables for keeping track of ROV Data
        self.thrusters_enabled = False
        self.sensitivity = { "Horizontal": None, "Vertical": None, "Angular": None, "Slow Factor": None}

        # Create your HUD editor
        resolution = (1920, 1080)
        self.hud = camera_overlay.HUD(resolution)
        self.bridge = CvBridge()

        # Create Framerate and callback timer
        framerate = 0.001
        self.create_timer(framerate, self.display_camera)


    def camera_count_callback(self, msg):
        if self.camera_count == -1:
            self.camera_count = msg.data
            if self.camera_count == 0:
                self.log.info("Error: No cameras found") 
                self.log.info("Quitting Program")
                exit()

            self.log.info(f"{self.camera_count} Cameras Found")

    # Grab the most recent frame from the camera feed
    def read_frame(self):
        success, frame = self.vid_capture.read()
        if not success:
            return None
        else: return frame


    # Display the most recent frame in our camera feed window
    def display_camera(self):
        if self.camera_count > 0 and not self.compared_to_config: # if we've received a camera count but haven't compared it to the config yet 
            if self.camera_count != len(self.config):
                self.config = {}
                for i in range(self.camera_count):
                    self.config[str(i+1)] = {'port': 5000 + i, 'nickname': f'Unnamed{i+1}', 'gripper': 'Front', 'vertical-flip': False}
                    self.log.info(str(self.config))

            self.compared_to_config = True 
            self.cur_cam = "1"
            self.change_camera()

        elif self.camera_count > 0:
            if self.vid_capture is None:
                time.sleep(0.1)
                return
            frame = self.read_frame()
            if frame is None:
                return

            if self.config[self.cur_cam]['vertical-flip']:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            
            # Add the overlay to the camera feed
            frame = self.hud.add_camera_details(frame, self.cur_cam, self.config[self.cur_cam]['nickname'])
            frame = self.hud.add_thruster_status(frame, self.thrusters_enabled)
            frame = self.hud.add_sensitivity(frame, self.sensitivity)
            frame = self.hud.add_gripper(frame, self.config[self.cur_cam]['gripper'])

            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)
    
    # Switches the captured video feed whenever we change cameras.
    # Also updates with most recent camera data.
    def change_camera(self):
        try: self.vid_capture.release()
        except: pass

        self.log.info(f"Changing Camera to {self.cur_cam}")
        
        self.vid_capture = cv2.VideoCapture(f"http://192.168.88.86:{self.config[self.cur_cam]["port"]}/stream")
        
        msg = Cam()
        msg.port = self.config[self.cur_cam]["port"]
        msg.nickname = self.config[self.cur_cam]["nickname"]
        msg.gripper = self.config[self.cur_cam]["gripper"]

        self.camera_pub.publish(msg)

    # Sets the thrusters_enabled variable to match thruster status
    def thruster_status_callback(self, request, response):
        self.thrusters_enabled = request.data
        response.success = True
        return response
    

    # Updates the sensetivity values to be displayed
    def sensitivity_callback(self, sensitivity_data):
        # Rounding to deal with some floating point imprecision
        self.sensitivity["Horizontal"] = round(sensitivity_data.horizontal, 2)
        self.sensitivity["Vertical"] = round(sensitivity_data.vertical, 2)
        self.sensitivity["Angular"] = round(sensitivity_data.angular, 2)
        self.sensitivity["Slow Factor"] = round(sensitivity_data.slow_factor, 2)

    # Checks to see if we are broadcasting to topic or not
    def joy_callback(self, joy):
        change = False

        if joy.buttons[4] or joy.buttons[5] or joy.buttons[6] or joy.buttons[7] or joy.buttons[8]:
            # Minor consequence of this logic is:
            # If multiple buttons are pressed, the furthest clockwise takes priority
            # No big deal tbh. About as reasonable a solution as any.
            
            desired_camera_index = False

            # Also, this cached input stuff is so that holding the button does not trigger this repeatedly.
            if joy.buttons[4] and not self.cached_button_input[0]: # Up button
                desired_camera_index = "1"
                change = True
            if joy.buttons[5] and not self.cached_button_input[1]: # Right Button
                desired_camera_index = "2"
                change = True
            if joy.buttons[6] and not self.cached_button_input[2]: # Down button
                desired_camera_index = "3"
                change = True
            if joy.buttons[7] and not self.cached_button_input[3]: # Left Button
                desired_camera_index = "4"
                change = True
            if joy.buttons[8] and not self.cached_button_input[4]:
                desired_camera_index = "5"
                change = True 

            # If a button state has changed...
            if change: 
                # If there's an entry in the config file for this index...
                if desired_camera_index in self.config.keys():   
                        if self.cur_cam is not desired_camera_index:
                            self.cur_cam = desired_camera_index
                            self.change_camera()

                else: # If there is no entry in the config file for this index:
                    self.log.warn("No camera mapped to that button")
            
        self.cached_button_input = [joy.buttons[4], joy.buttons[5], joy.buttons[6], joy.buttons[7], joy.buttons[8]]

    def write_to_config(self):
        with open(self.config_path, "w") as f:
            toml.dump(self.config, f)

def main(args=None):
    rclpy.init(args=args)

    camera_viewer = Camera_Viewer()

    # Runs the program until shutdown is recieved
    try: rclpy.spin(camera_viewer)
    except KeyboardInterrupt: camera_viewer.write_to_config()

    # On shutdown, kill node
    camera_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

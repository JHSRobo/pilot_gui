import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image, Joy
import cv2 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image, Joy
from std_srvs.srv import SetBool
from core.msg import Sensitivity, Cam
from core_lib import camera_overlay
import time
import toml

# w W
class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.log = self.get_logger()

        # Getting known information from the config file in the config dictionary and storing currently available cameras in the cameras dictionary
        with open("/home/jhsrobo/corews/src/pilot_gui/cam_config.toml", "r") as f:
            self.config = toml.load(f)
        self.cameras = {}
        self.cur_cam = "1"
        self.cached_cam = None

        # Time for cameras to turn on 
        time.sleep(1)

        # Look at all ros topics on the network
        for topic, topic_type in self.get_topic_names_and_types():
            # If the topic is a camera, take note of the ip. If that ip is in the config, use the config data
            # when displaying the camera. Otherwise, use some of the defaults.
            if "camera" in topic:
                index = get_index(topic) # returns - 1 if the camera is not already in the config
                if index == -1:
                    cam_dict = {}
                    cam_dict["ip"] = "192.168.1." + topic.replace("camera", "")
                    cam_dict["gripper"] = "Front"
                    cam_dict["nickname"] = "Unnamed"
                    cam_dict["topic"] = topic
                    self.config[str(len(config) + 1)] = cam_dict
                    self.cameras[str(len(cameras) + 1)] = cam_dict
                else: 
                    self.cameras[str(len(cameras) + 1)] = config[index]

        # If no cameras are found, end the program. Otherwise, continue.
        if len(self.cameras.keys()) == 0:
            self.log.info("No Cameras Found")
            exit()

        # Defaults to viewing the camera feed at index 1 
        self.cam_sub = self.create_subscription(Image, self.cameras[self.cur_cam]["topic"], self.image_callback, 10)

        # Sub to joy for camera changes
        self.joy_sub = self.create_subscription(Joy, "joy", self.change_camera_callback, 10)
        self.cached_button_input = [0, 0, 0, 0]

        # Various hud information subscribers
        self.sensitivity_sub = self.create_subscription(Sensitivity, "sensitivity", self.sensitivity_callback, 10)
        self.sensitivity = { "Horizontal": None, "Vertical": None, "Angular": None, "Slow Factor": None}

        self.thruster_status_strv = self.create_service(SetBool, 'thruster_status', self.thruster_status_callback)
        self.thrusters_enabled = False

        self.leak_detect_srv = self.create_service(SetBool, "leak_detection", self.leak_detection_callback)
        self.leak_detected = False

        # Camera Data Publisher (primarily so that grippers know what gripper to use)
        self.camera_pub(Cam, "active_camera", 10) 
        self.camera_pub.publish(self.create_camera_msg())
        
        # Create camera feed window
        cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

        # Create your HUD editor
        resolution = (1440, 810)
        self.hud = camera_overlay.HUD(resolution)
        self.bridge = CvBridge()

    def image_callback(self, frame):
        # Overlay the HUD
        frame = self.bridge.imgmsg_to_cv2(frame, desired_encoding="passthrough")
        frame = self.hud.add_camera_details(frame, self.cur_cam, self.cameras[self.cur_cam]["nickname"])
        frame = self.hud.add_thruster_status(frame, self.thrusters_enabled)
        frame = self.hud.add_sensitivity(frame, self.sensitivity) 
        frame = self.hud.add_gripper(frame, self.cameras[self.cur_cam]["gripper"])
        if self.leak_detected: frame = self.hud.leak_notification(frame) 

        # Display the Actual Frame
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

    def change_camera_callback(self, joy):
        change = False 

        if joy.buttons[4] or joy.buttons[5] or joy.buttons[6] or joy.buttons[7]:
            # Minor consequence of this logic is:
            # If multiple buttons are pressed, the furthest clockwise takes priority
            # No big deal tbh. About as reasonable a solution as any.
            
            desired_camera_index = -1

            # Also, this cached input stuff is so that holding the button does not trigger this repeatedly.
            if joy.buttons[4] and not self.cached_button_input[0]: # Up button
                desired_camera_index = 1
                change = True
            if joy.buttons[5] and not self.cached_button_input[1]: # Right Button
                desired_camera_index = 2
                change = True
            if joy.buttons[6] and not self.cached_button_input[2]: # Down button
                desired_camera_index = 3
                change = True
            if joy.buttons[7] and not self.cached_button_input[3]: # Left Button
                desired_camera_index = 4
                change = True

            if change:
                if str(desired_camera_index) in self.cameras.keys():
                    if self.cached_cam is not desired_camera_index:
                        self.cur_cam = str(desired_camera_index)

                        self.destroy_subscription(self.cam_sub)
                        self.cam_sub = self.create_subscription(Image, self.cameras[self.cur_cam]["topic"], self.image_callback, 10)

                        camera_msg = self.create_camera_msg()
                        self.camera_pub.publish(camera_msg)
                else:
                    self.log.warn("No camera mapped to that button")

        self.cached_button_input = [joy.buttons[4], joy.buttons[5], joy.buttons[6], joy.buttons[7]]
        self.cached_cam = self.cur_cam

    def sensitivity_callback(self, sensitivity_data):
        self.sensitivity["Horizontal"] = round(sensitivity_data.horizontal, 2)
        self.sensitivity["Vertical"] = round(sensitivity_data.vertical, 2)
        self.sensitivity["Angular"] = round(sensitivity_data.angular, 2)
        self.sensitivity["Slow Factor"] = round(sensitivity_data.slow_factor, 2)

    def thruster_status_callback(self, request, response):
        self.thrusters_enabled = request.data 
        response.success = True 
        return response

    def leak_detection_callback(self, request, response): 
        self.leak_detected = request.data 
        response.success = True 
        return response 

    def get_index(self, val):
        # Returns the index of the first camera with an attribute equal to val
        for cam in cameras:
            for key in cam.keys():
                if cam[key] == val:
                    return cam
        return -1

    def create_camera_msg(self):
        # Creates the current camera msg
        camera_msg = Cam()
        camera_msg.index = int(self.cur_cam)
        camera_msg.ip = self.cameras[self.cur_cam]["ip"]
        camera_msg.topic = self.cameras[self.cur_cam]["topic"]
        camera_msg.gripper = self.cameras[self.cur_cam]["gripper"]
        camera_msg.nickname = self.cameras[self.cur_cam]["nickname"]
        return camera_msg

def main(args=None):
    rclpy.init(args=args)
    camera_viewer_node = CameraViewer()
    rclpy.spin(camera_viewer_node)
    camera_viewer_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

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
        
        self.log = self.get_logger()

        self.vid_capture = None

        self.first_sense_request = self.create_client(Trigger, "first_sensitivity")
        self.sensitivity_sub = self.create_subscription(Sensitivity, "sensitivity", self.sensitivity_callback, 10)
        self.camera_pub = self.create_publisher(Cam, "active_camera", 10)

        self.thruster_status_srv = self.create_service(SetBool, 'thruster_status', self.thruster_status_callback)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.cached_button_input = [0, 0, 0, 0, 0, 0]


        self.cur_cam = "1" 

        self.camera_count = -1
        self.compared_to_config = False
        self.camera_count_subscriber = self.create_subscription(Int32, 'camera_count', self.camera_count_callback, 10)

        cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        
        self.apply_crop = False
        self.x1, self.y1, self.x2, self.y2 = 420, 0, 1500, 1120

        self.alignment_bar = False
        self.declare_parameter('alignment_bar', self.alignment_bar)

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.config_path = "/home/jhsrobo/corews/src/pilot_gui/cam_config.toml"
        try:
            with open(self.config_path, "r") as f:
                self.config = toml.load(f)
                self.log.info(str(self.config))
        except FileNotFoundError:
            self.log.info("Could not find cam_config.toml, generating new one...")
            self.config = {}

        self.thrusters_enabled = False
        self.sensitivity = {"Horizontal": None, "Vertical": None, "Yaw": None, "Roll": None, "Pitch": None, "Slow Factor": None}

        resolution = (1920, 1080)
        self.hud = camera_overlay.HUD(resolution)
        self.bridge = CvBridge()

        framerate = 0.001
        self.create_timer(framerate, self.display_camera)
        self.crab = False

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'alignment_bar':
                self.alignment_bar = param.value

    def camera_count_callback(self, msg):
        if self.camera_count == -1:
            self.camera_count = msg.data
            if self.camera_count == 0:
                self.log.info("Error: No cameras found, quitting...")
                exit()
            self.log.info(f"{self.camera_count} Cameras Found")

    def read_frame(self):
        success, frame = self.vid_capture.read()
        if not success:
            return None
        return frame

    def display_camera(self):
        if self.camera_count > 0 and not self.compared_to_config:
            if self.camera_count != len(self.config):
                self.config = {}
                for i in range(self.camera_count):
                    self.config[str(i + 1)] = {
                        'nickname': f'Unnamed{i + 1}',
                        'gripper': 'Front',
                        'rotation': 0
                    }
                self.log.info(str(self.config))

            self.compared_to_config = True
            self.cur_cam = "1"
            self.connect_to_stream()

        elif self.camera_count > 0:
            if self.vid_capture is None:
                time.sleep(0.1)
                return
            frame = self.read_frame()
            if frame is None:
                return

            if self.config[self.cur_cam]['rotation'] == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif self.config[self.cur_cam]['rotation'] == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif self.config[self.cur_cam]['rotation'] == 270:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            if self.apply_crop:
                frame = frame[self.y1:self.y2, self.x1:self.x2]

            if self.crab:
                frame = self.hud.crab_model(frame)

            if self.alignment_bar:
                frame = self.hud.add_horizontal_bar(frame)

            frame = self.hud.add_camera_details(frame, self.cur_cam, self.config[self.cur_cam]['nickname'])
            frame = self.hud.add_thruster_status(frame, self.thrusters_enabled)
            frame = self.hud.add_sensitivity(frame, self.sensitivity)
            frame = self.hud.add_gripper(frame, self.config[self.cur_cam]['gripper'])

            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)

    def connect_to_stream(self):
        try:
            self.vid_capture.release()
        except:
            pass

        self.log.info(f"Connecting to stream for camera {self.cur_cam}")

        time.sleep(0.3)
        self.vid_capture = cv2.VideoCapture("http://192.168.88.111:5000/stream")

        msg = Cam()
        msg.nickname = self.config[self.cur_cam]["nickname"]
        msg.gripper = self.config[self.cur_cam]["gripper"]
        self.camera_pub.publish(msg)

    def thruster_status_callback(self, request, response):
        self.thrusters_enabled = request.data
        response.success = True
        return response

    def sensitivity_callback(self, sensitivity_data):
        self.sensitivity["Horizontal"] = round(sensitivity_data.horizontal, 2)
        self.sensitivity["Vertical"] = round(sensitivity_data.vertical, 2)
        self.sensitivity["Yaw"] = round(sensitivity_data.yaw, 2)
        self.sensitivity["Roll"] = round(sensitivity_data.roll, 2)
        self.sensitivity["Pitch"] = round(sensitivity_data.pitch, 2)
        self.sensitivity["Slow Factor"] = round(sensitivity_data.slow_factor, 2)

    def joy_callback(self, joy):
        b = joy.buttons
        c = self.cached_button_input
        change = False

        # Crab AI toggle
        if b[2] and not c[5]:
            self.crab = not self.crab
            self.log.info(f"Crabs {'enabled' if self.crab else 'disabled'}")

        if b[4] or b[5] or b[6] or b[7] or b[8] or b[9]:
            desired_camera = None

            if b[9] and not c[6]:
                self.apply_crop = not self.apply_crop
                self.log.info("Set crop variable")
            if b[4] and not c[0]:
                desired_camera = "1"; change = True
            if b[5] and not c[1]:
                desired_camera = "2"; change = True
            if b[6] and not c[2]:
                desired_camera = "3"; change = True
            if b[7] and not c[3]:
                desired_camera = "4"; change = True
            if b[8] and not c[4]:
                desired_camera = "5"; change = True

            if change:
                if desired_camera in self.config:
                    if self.cur_cam != desired_camera:
                        self.cur_cam = desired_camera
                        self.connect_to_stream()
                else:
                    self.log.warn("No camera mapped to that button")

        self.cached_button_input = [b[4], b[5], b[6], b[7], b[8], b[2], b[9]]

    def write_to_config(self):
        with open(self.config_path, "w") as f:
            toml.dump(self.config, f)


def main(args=None):
    rclpy.init(args=args)
    camera_viewer = Camera_Viewer()
    try:
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        camera_viewer.write_to_config()
    camera_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

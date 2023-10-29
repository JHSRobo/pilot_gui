import rclpy
from rclpy.node import Node
from core.msg import Cam, Sensitivity
from std_srvs.srv import Trigger, SetBool
from core_lib import camera_overlay

import time
import cv2


class Camera_Viewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')
        
        self.log = self.get_logger() # Quick reference for ROS logging

        self.vid_capture = None # Variable for storing connection to camera stream

        # Create subsciber for changing cameras
        self.camera_sub = self.create_subscription(Cam, "active_camera", self.change_camera_callback, 10)

        # Create client for adding the first camera
        self.first_cam_request = self.create_client(Trigger, "first_camera")

        # Create a client and sub for getting sensitivity
        self.first_sense_request = self.create_client(Trigger, "first_sensitivity")
        self.sensitivity_sub = self.create_subscription(Sensitivity, "sensitivity", self.sensitivity_callback, 10)

        # Create a service for adjusting thruster status
        self.thruster_status_srv = self.create_service(SetBool, 'thruster_status', self.thruster_status_callback)

        # Create a service for reporting leak detection
        self.leak_detect_srv = self.create_service(SetBool, "leak_detection", self.leak_detection_callback)

        # Create window used for displaying camera feed
        cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

        # Create variables for keeping track of ROV Data
        self.nickname = None
        self.index = 0
        self.thrusters_enabled = False
        self.sensitivity = { "Horizontal": None, "Vertical": None, "Angular": None }
        self.leak_detected = False
        
        # Create your HUD editor
        resolution = (1280, 720)
        self.hud = camera_overlay.HUD(resolution)

        # Create Framerate and callback timer
        framerate = 1.0 / 50.0
        self.create_timer(framerate, self.display_camera)


    # Grab the most recent frame from the camera feed
    def read_frame(self):
        success, frame = self.vid_capture.read()
        if not success:
            return None
        else: return frame


    # Display the most recent frame in our camera feed window
    def display_camera(self):
        if self.vid_capture is None:
            time.sleep(0.1)
            request = Trigger.Request()
            self.first_cam_request.call_async(request)
            self.first_sense_request.call_async(request)
            return
        
        frame = self.read_frame()
        if frame is None:
            return
        
        # Add the overlay to the camera feed
        frame = self.hud.add_camera_details(frame, self.index, self.nickname)
        frame = self.hud.add_thruster_status(frame, self.thrusters_enabled)
        frame = self.hud.add_sensitivity(frame, self.sensitivity)
        if self.leak_detected: frame - self.hud.leak_notification(frame)

        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)
    

    # Switches the captured video feed whenever we change cameras.
    # Also updates with most recent camera data.
    def change_camera_callback(self, cam_msg=Cam):
        try: self.vid_capture.release()
        except: pass

        self.vid_capture = cv2.VideoCapture("http://{}:5000/1".format(cam_msg.ip))
        self.nickname = cam_msg.nickname
        self.index = cam_msg.index


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

    # Updates leak detection status
    def leak_detection_callback(self, request, response):
        self.leak_detected = request.data
        response.success = True
        return response
    

def main(args=None):
    rclpy.init(args=args)

    camera_viewer = Camera_Viewer()

    # Runs the program until shutdown is recieved
    rclpy.spin(camera_viewer)

    # On shutdown, kill node
    camera_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

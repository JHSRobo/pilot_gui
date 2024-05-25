import rclpy
from rclpy.node import Node
from core.msg import Cam, Sensitivity, RectDimensions
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger, SetBool
from core_lib import camera_overlay
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge

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
        self.outer_temp_sub = self.create_subscription(Float32, "outer_thermometer", self.empty_callback, 10)
        self.inner_temp_sub = self.create_subscription(Float32, "inner_thermometer", self.empty_callback, 10)
        self.depth_sub = self.create_subscription(Float32, "depth_sensor", self.empty_callback, 10)

        # Create a service for adjusting thruster status
        self.thruster_status_srv = self.create_service(SetBool, 'thruster_status', self.thruster_status_callback)

        # Create a service for reporting leak detection
        self.leak_detect_srv = self.create_service(SetBool, "leak_detection", self.leak_detection_callback)

        # Create a publisher for publishing the active feed
        self.image_pub = self.create_publisher(Image, 'camera_feed', 10)

        # Create a joystick subscriber for toggling recording
        self.joy_sub = self.create_subscription(Joy, 'joy', self.empty_callback, 10)

        # Create a subscriber to receive parameters for bounding box (autonomous movement task)
        self.msg = RectDimensions()
        self.last_msg_time = self.get_clock().now().nanoseconds
        self.red_sub = self.create_subscription(RectDimensions, 'parameters', self.red_callback, 10)
        self.timer_duration_ns = 2e9 # 2 seconds
        self.previous_timer = self.create_timer(0.1, self.previous_timer_callback)

        # Create window used for displaying camera feed
        cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

        # Create variables for keeping track of ROV Data
        self.nickname = None
        self.index = 0
        self.thrusters_enabled = False
        self.sensitivity = { "Horizontal": None, "Vertical": None, "Angular": None, "Slow Factor": None}
        self.leak_detected = False
        self.gripper = None

        self.publish_img = False
        
        # Create your HUD editor
        resolution = (1440, 810)
        self.hud = camera_overlay.HUD(resolution)
        self.bridge = CvBridge()

        # Create Framerate and callback timer
        framerate = 1.0 / 50.0
        self.create_timer(framerate, self.display_camera)

    def red_callback(self, msg):
        self.msg = msg
        self.last_msg_time = self.get_clock().now().nanoseconds

    def previous_timer_callback(self):
        current_time = self.get_clock().now().nanoseconds
        time_elapsed = current_time - self.last_msg_time

        if time_elapsed > self.timer_duration_ns:
            self.msg = RectDimensions()

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
        
        # Clear frame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        frame[:, :, 3] = 0

        # Publish our image to camera feed if enabled
        if self.publish_img:
            self.broadcast_feed(frame)
        
        # Add the overlay to the camera feed
        frame = self.hud.add_camera_details(frame, self.index, self.nickname)
        frame = self.hud.add_thruster_status(frame, self.thrusters_enabled)
        frame = self.hud.add_sensitivity(frame, self.sensitivity)
        frame = self.hud.add_gripper(frame, self.gripper)
        frame = self.hud.add_publish_status(frame, self.publish_img)

        if self.msg is not None and self.msg.x != 0 and self.msg.y != 0 and self.msg.w != 0 and self.msg.h != 0:
           frame = cv2.rectangle(frame, (self.msg.x, self.msg.y), (self.msg.x + self.msg.w, self.msg.y + self.msg.h), (0, 255, 0), 2)
           cv2.putText(frame, "Target Area, " + str(self.msg.x) + ", " + str(self.msg.y), (self.msg.x, self.msg.y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
        else:
            frame = cv2.putText(frame, "No Target Area", (10, 910), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        if self.leak_detected: frame - self.hud.leak_notification(frame)

        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

    # Publish our current camera feed frame to ROS topic
    def broadcast_feed(self, frame):
        img = Image()
        img = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.image_pub.publish(img)


    # Switches the captured video feed whenever we change cameras.
    # Also updates with most recent camera data.
    def change_camera_callback(self, cam_msg=Cam):
        try: self.vid_capture.release()
        except: pass

        self.vid_capture = cv2.VideoCapture("http://{}:5000".format(cam_msg.ip))
        self.nickname = cam_msg.nickname
        self.index = cam_msg.index
        self.gripper = cam_msg.gripper

    def empty_callback(self, data):
        pass

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

    # Updates leak detection status
    def leak_detection_callback(self, request, response):
        self.leak_detected = request.data
        response.success = True
        return response
    
    # Checks to see if we are broadcasting to topic or not
    def joy_callback(self, joy):

        # Enable or disable thrusters based on button press
        if joy.buttons[2] and not self.cached_input:
            self.publish_img = not self.publish_img
            if self.publish_img: self.log.info("Starting to publish camera feed")
            else: self.log.info("No longer publishing camera feed")

        self.cached_input = joy.buttons[2]
    

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

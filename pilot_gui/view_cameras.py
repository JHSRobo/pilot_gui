import rclpy
from rclpy.node import Node
from core.msg import Cam
from std_srvs.srv import Trigger

import time
import cv2

class Camera_Viewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')
        
        self.log = self.get_logger() # Quick reference for ROS logging

        self.vid_capture = None # Variable for storing connection to camera stream

        # Create subsciber for changing cameras
        self.camera_sub = self.create_subscription(Cam, "active_camera", self.change_camera_callback, 10)

        # Create service for adding the first camera
        self.first_cam_request = self.create_client(Trigger, "FirstCamera")

        # Create window used for displaying camera feed
        cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

        # Create Framerate and callback timer
        framerate = 1.0 / 50.0
        self.create_timer(framerate, self.display_camera)

    # Grab the most recent frame from the camera feed
    def read_frame(self):
        success, frame = self.vid_capture.read()
        if not success:
            return None
        else: return frame


    def display_camera(self):
        if self.vid_capture is None:
            time.sleep(0.1)
            request = Trigger.Request()
            self.future = self.first_cam_request.call_async(request)
            return
        
        frame = self.read_frame()
        if frame is not None:
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)
    
    def change_camera_callback(self, cam_msg=Cam):
        try: self.vid_capture.release()
        except: pass

        self.vid_capture = cv2.VideoCapture("http://{}:5000/1".format(cam_msg.ip))
    

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

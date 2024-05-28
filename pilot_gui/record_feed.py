import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import os
import cv2
from cv_bridge import CvBridge

class RecordFeedNode(Node):
    def __init__(self):

        super().__init__('record_feed')

        self.log = self.get_logger()
        
        # Image Bridge
        self.bridge = CvBridge()
        
        # Feed Subscriber
        self.image_sub = self.create_subscription(Image, 'camera_feed', self.write_feed, 10)

        # Num of Photos
        self.coral_count = 0

        # Path to Flashdrive
        self.path = "/media/jhsrobo/471C-18FD/img_capture"

    def write_feed(self, frame=Image):
        img = self.bridge.imgmsg_to_cv2(frame)
        cv2.imwrite("{}/{}.png".format(self.path, self.coral_count), img)
        self.coral_count += 1

def main(args=None):
    rclpy.init(args=args)

    camera_feed = RecordFeedNode()

    os.system('rm -rf /media/jhsrobo/471C-18FD/img_capture/*.png')

    rclpy.spin(camera_feed)

    camera_feed.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


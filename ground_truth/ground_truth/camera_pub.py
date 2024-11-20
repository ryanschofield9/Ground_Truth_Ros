from groun_truth_msgs.srv import CameraRecord

from geometry_msgs.msg import TwistStamped, Vector3

from sensor_msgs.msg import Image

import numpy as np

import rclpy
from rclpy.node import Node

import cv2
import time 



# TO DO: CREATE THE MESSAGE PACKAGE AND ADD THE SERVICE INTO IT SEE GITHUB FROM ROB599 HW3 FOR EXPLANATION
class Pub_Camera(Node):

    def __init__(self):
        super().__init__('pub_camera')

        
        #Create Publisher 
        self.pub_camera_image= self.create_publisher(Image, 'camera_image', 10)
        self.timer = self.create_timer(1/50, self.camera_image)
        self.video =   video = cv2.VideoCapture(0)
        if (self.video.isOpened() == False):
            print("Error reading video file")

        self.frame_width = int(self.video.get(3))
        self.frame_height = int(self.video.get(4))
        
    def camera_image(self):
        
        ret, frame =self.video.read()

        msg = Image()
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'Camera'
        msg.height = self.frame_height
        msg.width= self.frame_width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] *np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()

        self.pub_camera_image.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    camera = Pub_Camera()
    rclpy.spin(camera)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
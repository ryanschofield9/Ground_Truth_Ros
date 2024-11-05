from groun_truth_msgs.srv import CameraRecord

from geometry_msgs.msg import TwistStamped, Vector3

import rclpy
from rclpy.node import Node

import cv2
import time 



# TO DO: CREATE THE MESSAGE PACKAGE AND ADD THE SERVICE INTO IT SEE GITHUB FROM ROB599 HW3 FOR EXPLANATION
class RecordVideoService(Node):

    def __init__(self):
        super().__init__('record_video_service')

        #Create Service 
        self.service_pixel = self.create_service(CameraRecord, 'camera_record', self.camera_record) 

        #Create Publisher 
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
    def camera_record(self, request, response):
        file = request.file_name
        response = CameraRecord.Response()
        video = cv2.VideoCapture(0)
        if (video.isOpened() == False):
            print("Error reading video file")
            response.recorded = False
            return response 
        
        frame_width = int(video.get(3))
        frame_height = int(video.get(4))
        size = (frame_width, frame_height)
        self.get_logger().info(f"Video size: {size}")
        result = cv2.VideoWriter(file, cv2.VideoWriter_fourcc(*'MJPG'), 10.0, size) 
        start_time = time.time()
        now = time.time()
        while (now - start_time ) < 3: 
            ret, frame =video.read()
            if ret == True:
                result.write(frame)
                now = time.time()
                self.publish_twist([0.0, 0.1, 0.0], [0.0, 0.0, 0.0]) #move down at 0.1 m/s 
            else:
                for x in range (0,3):
                    self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) # stop moving 
                response.recorded = False
                return response 
        for x in range (0,3):
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) # stop moving 

        video.release()
        result.release()
        cv2.destroyAllWindows()
        self.get_logger().info(f"The video was successfully saved")
        response.recorded = True
        print("The response is ", response)
        return response 
    
    def publish_twist(self, linear_speed, rot_speed):
        #publish a velocity command with the specified linear speed and rotation speed 
        my_twist_linear = linear_speed 
        my_twist_angular = rot_speed
        cmd = TwistStamped()
        cmd.header.frame_id = 'tool0'
        cmd.twist.angular = Vector3(x=my_twist_angular[0], y=my_twist_angular[1], z=my_twist_angular[2])
        cmd.twist.linear = Vector3(x=my_twist_linear[0], y=my_twist_linear[1], z=my_twist_linear[2])
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_vel_commands.publish(cmd)
        #self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")

def main(args=None):
    rclpy.init(args=args)
    record_vid = RecordVideoService()
    rclpy.spin(record_vid)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
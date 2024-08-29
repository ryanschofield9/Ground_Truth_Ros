
from groun_truth_msgs.srv import CalcDiameter, CameraRecord

import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TransformStamped, Point, TwistStamped, Vector3
from tf2_ros import TransformException

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from controller_manager_msgs.srv import SwitchController
import numpy as np

from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState
import time 
import matplotlib.pyplot as plt

'''
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
)
from rclpy.action import ActionClient
'''

import time 



# TO DO: CREATE THE MESSAGE PACKAGE AND ADD THE SERVICE INTO IT SEE GITHUB FROM ROB599 HW3 FOR EXPLANATION
class TouchTree(Node):

    def __init__(self):
        super().__init__('tree_touch')

        #Create clients 
        '''
        self.calc_diameter_client = self.create_client(CalcDiameter, 'calc_diameter')
        while not self.calc_diameter_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for calc_diameter service to start')
        '''

        self.camera_record_client = self.create_client(CameraRecord, 'camera_record')
        while not self.camera_record_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for camera_record service to start')

        #Create publisher and subscribers 
        self.sub_flag = self.create_subscription(Bool, 'step3',self.calback_step3_flag, 10 )
        self.pub_step5 = self.create_publisher(Bool, 'step5', 10)
        self.sub_reset = self.create_subscription(Bool, 'reset',self.calback_reset, 10 )
        
        #create timers 
        self.control_timer = self.create_timer(1/10, self.main_control)
        self.tool_timer_pos_y = self.create_timer(1/50, self.pub_tool_pose_y)

        
    def main_control (self):
        #This function is called every 0.1 seconds and holds the main control structure for touching the tree 
        #The arm will move forward until the tree is touched 
        
        if self.step_3: 
            if not self.future: # A call is not pending
                request = CameraRecord.Request()
                request.file_name = 'src/Ground_Truth_Ros/ground_truth/videos/testing.avi'
                self.future = self.camera_record_client.call_async(request)
                self.starting_y = self.tool_y 

            if self.future.done(): # a call just completed
                if self.first == False: 
                    self.get_logger().info("Done")
                    print(self.future.result())
                    self.first = True 
                elif self.back_to_original == False:
                    self.move_up_to_y(self.starting_y) 
                    self.step_3 = False 
                    self.step_4 = True 
                    self.future = None
            
        elif self.step_4: 
            ## HAVE TO ADD HERE TO DO OPTICAL FLOW 
            msg = Bool()
            msg.data = True 
            plt.plot([1,2], [1,2])
            plt.show()
            self.pub_step5.publish(msg)
            self.step_4 = False 

            
    def calback_step3_flag(self,msg):
        self.step_3= msg.data
    
    def move_up_to_y(self, y_pose_want):
        #Using the given y pose that we want to go back to, move down until we reach that location 
        y_pose= self.tool_y #get the current tool y pose 
        #print("y_pose: ", y_pose, " y_pose_want", y_pose_want)
        if (y_pose_want - y_pose) < 0.001: 
            #when within 1mm of wanted y pose 
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
            self.back_to_original= True
        return 

    def pub_tool_pose_y(self):
        #save the current tool pose y 
        #there is a timer calling this function every 0.1 seconds 
        self.tool_y = self.get_tool_pose_y()


    def get_tool_pose_y(self, as_array=True):
            #Get the y position of the tool pose in respect to the base in m 
            try:
                #try to get the transform of from the base frame to the tool frame 
                current_time = rclpy.time.Time()
                tf = self.tf_buffer.lookup_transform(
                    self.tool_frame, self.base_frame, current_time
                )
                
            except TransformException as ex:
                #if the tranform returns a TransformException error print the error
                self.get_logger().warn("Received TF Exception: {}".format(ex))
                return
            pose = convert_tf_to_pose(tf) 
            if as_array:
                #if the pose is given as an array save the position and orientation as p and o respectively 
                p = pose.pose.position
                o = pose.pose.orientation
                return p.y #return just the tool y position with respect to teh base in m 
            else:
                return pose
            
    def calback_reset(self,msg): 
        if msg.data == True: 
            self.reset()
        
    def reset(self):
        #Flags initialize 
        self.step_3 = False
        self.step_4 = False
        self.back_to_original = False
        self.first = False  
        
        #set future 
        self.future = None

def convert_tf_to_pose(tf: TransformStamped):
    #take the tf transform and turn that into a position  
    pose = PoseStamped() #create a pose which is of type Pose stamped
    pose.header = tf.header
    tl = tf.transform.translation
    pose.pose.position = Point(x=tl.x, y=tl.y, z=tl.z)
    pose.pose.orientation = tf.transform.rotation

    return pose

def main(args=None):
    rclpy.init(args=args)

    ang_check = TouchTree()

    rclpy.spin(ang_check)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
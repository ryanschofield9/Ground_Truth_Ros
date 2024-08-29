
from groun_truth_msgs.srv import CalcDiameter

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


class TouchTree(Node):

    def __init__(self):
        super().__init__('tree_touch')

        #Create clients 
        self.calc_diameter_client = self.create_client(CalcDiameter, 'calc_diameter')
        while not self.calc_diameter_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for calc_diameter service to start')
    
        #Create publishers and subscripers 
        self.sub = self.create_subscription(Bool, 'touching_tree_flag', self.callback_tree, 10) 
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.sub_flag = self.create_subscription(Bool, 'step5',self.calback_step5_flag, 10 )
        self.sub_reset = self.create_subscription(Bool, 'reset',self.calback_reset, 10 )


        #Create tf buffer and listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #initalize states 
        self.step_5 = False
        self.step_6 = False
        self.tree_touch = False
        self.first = True 
        
        #set future 
        self.future = None
        
        #create timers 
        self.control_timer = self.create_timer(1/10, self.main_control)

        #constant variables 
        self.base_frame = 'base_link' #base frame that doesn't move 
        self.tool_frame = 'tool0' #frame that the end effector is attached to

        #initialaize variables 
        self.initial_z = self.get_tool_pose_z()
        self.count = 0
        
        
        time.sleep(3) # wait 3 seconds 

        


    def main_control (self):
        #This function is called every 0.1 seconds and holds the main control structure for touching the tree 
        #The arm will move forward until the tree is touched 
        
        if self.step_5:
            if self.first: 
                self.initial_z = self.get_tool_pose_z()
                self.first = False 
            if self.tree_touch == True: 
                self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving
                self.count += 1
                if self.count == 3:
                     self.step_5 = False 
                     self.step_6 = True
            else: 
                self.publish_twist([0.0, 0.0, 0.1], [0.0, 0.0, 0.0]) #move forward in the z position at 0.1 m/s
        elif self.step_6: 
                if not self.future: # A call is not pending
                    self.final_z = self.get_tool_pose_z()
                    dif_z = abs(self.initial_z - self.final_z) * 39.37 #have to conver from meters to inches 
                    self.video_dis = 6 + dif_z
                    print(f"Initial Z position: {self.initial_z}")
                    print(f"Final Z position: {self.final_z}")
                    print(f"change in Z position: {dif_z}")
                    print(f"Original distance: {self.video_dis}")
                    request = CalcDiameter.Request()
                    request.video_dis = 7.0
                    self.future = self.calc_diameter_client.call_async(request)

                if self.future.done(): # a call just completed
                    self.get_logger().info("Done")
                    print(self.future.result())
                    self.step_4 = False 
                    self.future = None
            

                

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
        self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")

    def callback_tree (self, msg):
        #self.get_logger().info(f"GOT msg and setting self.touch_tree to: {msg.data}")
        if self.step_5: 
            self.tree_touch = msg.data 
    
    def calback_step5_flag(self, msg): 
        self.step_5 = msg.data 

    def get_tool_pose_z(self, as_array=True):
            #Get the z position of the tool pose in respect to the base in m 
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
                return p.z #return just the tool z position with respect to the base in m 
            else:
                return pose
            
    def calback_reset(self,msg): 
        if msg.data == True: 
            self.reset()
        
    def reset(self): 
        #reinitalize states 
        self.step_5 = False
        self.step_6 = False
        self.tree_touch = False
        self.first = True 

        #reinitialaize variables 
        self.initial_z = self.get_tool_pose_z()
        self.count = 0  
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
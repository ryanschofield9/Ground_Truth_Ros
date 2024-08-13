from rob599_hw3_msgs.srv import TouchTree

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

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
)
from rclpy.action import ActionClient



# TO DO: CREATE THE MESSAGE PACKAGE AND ADD THE SERVICE INTO IT SEE GITHUB FROM ROB599 HW3 FOR EXPLANATION
class AngleCheckClass(Node):

    def __init__(self):
        super().__init__('angle_check_service')

        #Create Service 
        self.srv = self.create_service(TouchTree, 'touching_tree', self.main_control)
    
        #Create publishers and subscripers 
        self.sub = self.create_subscription(Bool, 'tof1', self.callback_tree, 10) 
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        #Create tf buffer and listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Create Callback group
        self.service_handler_group = ReentrantCallbackGroup()
        
        #create timers 
        self.control_timer = self.create_timer(1/10, self.main_control)

        #set inital states 
        self.tree_touch = False 


    def main_control (self, request, response):
        #This function is called every 0.1 seconds and holds the main control structure for touching the tree 
        #The arm will move forward until the tree is touched 
        if self.tree_touch == True: 
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
            response.six_z_pose = self.get_tool_pose_z()
            return response
        else: 
           self.publish_twist([0.0, 0.0, 0.1], [0.0, 0.0, 0.0]) #move forward in the z position at 0.1 m/s
        
    
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
        self.tree_touch = msg.data 

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

    ang_check = AngleCheckClass()

    rclpy.spin(ang_check)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
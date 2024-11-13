
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


class Touch(Node):

    def __init__(self):
        super().__init__('touch')

    
        #Create publishers and subscripers 
        self.sub = self.create_subscription(Bool, 'touching_tree_flag', self.callback_tree, 10) 
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.sub_flag = self.create_subscription(Bool, 'touch',self.calback_touch_flag, 10 )
        

        #initalize states '
        self.touch_flag = False
        self.tree_touch = False
        
        
        #create timers 
        self.control_timer = self.create_timer(1/10, self.main_control)

        #initialaize variables 
        self.count = 0
        

        


    def main_control (self):
        #This function is called every 0.1 seconds and holds the main control structure for touching the tree 
        #The arm will move forward until the tree is touched 
        
        if self.touch_flag:
            if self.tree_touch == True: 
                self.get_logger().info("Touch Detected ")
                for x in range (0,3):
                    self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving
               
                self.count = 0 
                self.touch_flag = False
                self.tree_touch = False
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
        #self.get_logger().info(f"GOT msg and setting self.touch_tree to: {msg.data}")
        if self.touch_flag: 
            self.tree_touch = msg.data 
    
    def calback_touch_flag(self, msg): 
        self.touch_flag = msg.data 




def main(args=None):
    rclpy.init(args=args)

    ang_check = Touch()

    rclpy.spin(ang_check)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
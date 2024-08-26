from groun_truth_msgs.srv import AngleCheck

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

from std_msgs.msg import Float32, Bool, Int64
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
    
        #Create publishers and subscripers 
        self.sub_tof1 = self.create_subscription(Float32, 'tof1', self.callback_tof1, 10) 
        self.sub_tof2 = self.create_subscription(Float32, 'tof2', self.callback_tof2, 10)
        self.sub_tof1 = self.create_subscription(Float32, 'tof1_filter', self.callback_tof1_filtered, 10)
        self.sub_tof2 = self.create_subscription(Float32, 'tof2_filter', self.callback_tof2_filtered, 10)
        self.sub_joints = self.create_subscription(JointState, 'joint_states',self.callback_joints, 10 )
        self.sub_flag = self.create_subscription(Bool, 'step2',self.calback_step2_flag, 10 )
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pub_step3 = self.create_publisher(Bool, 'step3', 10)

        #create timers 
        self.control_timer = self.create_timer(1/10, self.main_control)
        self.tool_timer_pos_y = self.create_timer(1/50, self.pub_tool_pose_y)
        self.tool_timer_orient_z = self.create_timer(1/10,self.pub_tool_orient_z )
        self.tool_timer_angle = self.create_timer(1/50,self.pub_tool_angle)

        #Create tf buffer and listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        #Create Callback group
        self.service_handler_group = ReentrantCallbackGroup()
        
        #Create clients 
        
        self.switch_ctrl = self.create_client(
            SwitchController, "/controller_manager/switch_controller", callback_group=self.service_handler_group
        ) 
        self.moveit_planning_client = ActionClient(self, MoveGroup, "move_action")


        #inital states
        self.at_y= False #a flag to determine if the arm is at the wanted y position 
        self.rot_up_flag = False #a flag to determine if the rotation up step has been completed 
        self.rot_down_flag = False #a flag to determine if the rotation down step has been completed 
        self.move_up_flag = False #a flag to determine if the move up step has been completed 
        self.move_down_flag = False #a flag to determine if the move down step has been completed  
        self.joint_rot_flag = False #a flag to detemine if the joint controller has finished its rotation 
        self.done = False #a flag to determine if the check angle process has finished  
        self.saved_request = False #a flag to determine if the request has been saved
        self.got_request = True 
        self.send_request = False
        self.rotated_to_new = False
        self.moved_to_new = False 
        self.calc_first_time = False 

        #Wait three seconds to let everything get up and running (may not need)
        time.sleep(3)

        #constant variables 
        self.forward_cntr = 'forward_position_controller' #name of controller that uses velocity commands 
        self.joint_cntr = 'scaled_joint_trajectory_controller' # name of controller that uses joint commands 
        self.base_frame = 'base_link' #base frame that doesn't move 
        self.tool_frame = 'tool0' #frame that the end effector is attached to 
        self.move_collect = 3 #alloted time in seconds for moving up and collecting tof data  
        self.rot_collect = 3 #alloted time in seconds for rotating and collecting tof data
        self.dis_sensors = 0.0508 # meters 
        self.step_2 = False 
        self.first = True 

        #initialize variables 
        #anytime tof is used, it is filtered
        #if the raw data is being used it will be mentioned 
        self.tof1_readings = [] #holds raw tof data during tof collection period for tof1 
        self.tof2_readings = [] #holds raw tof data during tof collection period for tof2
        self.tof1_filtered_rot = [] #holds tof data during tof rotation collection period for tof1
        self.tof2_filtered_rot = [] #holds tof data during tof rotation collection period for tof2
        self.tof1_filtered_up = [] #holds tof data during tof moving collection period for tof1
        self.tof2_filtered_up = [] #holds tof data during tof moving collection period for tof2
        self.tool_orient_tof1 = [] # holds the tool orientation everytime a new tof reading comes in for tof1
        self.tool_orient_tof2 = [] # holds the tool orientation everytime a new tof reading comes in for tof2
        self.tool_pos_tof1 = [] # holds the tool position everytime a new tof reading comes in for tof1
        self.tool_pos_tof2 = [] # holds the tool position everytime a new tof reading comes in for tof2 
        self.tool_y = None #y position of the tool at the current moment 
        self.orient_z = None #z orientation of the tool at the current moment  
        self.desired_angle = 0.0 #need a way to get this #maybe make this a service and call the service giving the starting angle
        self.desired_y = 0.0 # need to get this value
        self.tries = 0 #tracking how many times the check angle control has been attempted 
        self.tool_angle = None

        #switch to forward position controller 
        #self.switch_controller(self.forward_cntr, self.joint_cntr)
          
        #set start_time 
        self.start_time = time.time() 

        
    def main_control (self):
        #TO DO: WRITE A SERVICE MSG TYPE  
        #This function is called every 0.1 seconds and holds the main control structure for checking the angle
        # Corrects the angle by continously collecting tof data and comparing where the low value was found until one angle and y are settled on  
        now = time.time()
        if self.step_2:
            if self.first:
                self.desired_angle = self.tool_angle
                self.start_time = time.time()
                self.desired_y = self.tool_y

                print(f"Desired Angle: {self.desired_angle}  Desired Y: {self.desired_y}")
                
                #print(f"pose: {self.get_tool_pose()}")
                self.first = False 

            if self.done == False:
                #if the check angle hasn't gotten into the correct range and hasn't attempted to more than 3 times
                if self.rot_up_flag == False: 
                    #if the rotation up step has not be completed 
                    if (now - self.start_time ) <self.rot_collect:
                        #if it hasn't been the alloted time for rotating and collecting tof data  
                        self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.5]) #rotate the tool in the z direction by 0.5 rads/s
                    else:
                        #if the alloted time for rotating and collecting tof data has passed 
                        if self.send_request == False: 
                            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
                            self.switch_controller(self.joint_cntr, self.forward_cntr) #switch from forward_position controller to scaled_joint_trajectory controller
                            self.rotate_to_w(self.desired_angle)
                            self.send_request = True 
                        #print(f"Desired = {self.desired_angle}   Tool Angle: {self.tool_angle}")
                        if (abs(self.desired_angle- self.tool_angle) < 0.0001):
                            #print("In function ")
                            #if the joint controller has finished its rotation 
                            self.rot_up_flag = True #set the rotation up step as completed 
                            self.joint_rot_flag = False #set the joint controller as NOT finished its rotation
                            self.switch_controller(self.forward_cntr, self.joint_cntr) #switch from scaled_joint_trajectory controller to forward_position controller
                            print("Finished Rot Up moving to Rot Down  ")
                            self.send_request = False
                            #print("TOF1 filtered rot 0 to 10: ",self.tof1_filtered_rot[0:10])
                            #print("TOF2 filtered rot 0 to 10: ",self.tof2_filtered_rot[0:10])

                            self.start_time = time.time() #reset the start time 

                
                elif self.rot_down_flag == False:
                    #if the rotation down step has not be completed 
                    if (now - self.start_time ) <self.rot_collect:
                        #if it hasn't been the alloted time for rotating and collecting tof data
                        self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, -0.5]) #rotate the tool in the z direction by -0.5 rads/s
                    else:
                        #if the alloted time for rotating and collecting tof data has passed 
                        if self.send_request == False: 
                            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving
                            self.switch_controller(self.joint_cntr, self.forward_cntr) #switch from forward_position controller to scaled_joint_trajectory controller
                            self.rotate_to_w(self.desired_angle)
                            self.send_request = True 
                        if (abs(self.desired_angle- self.tool_angle) < 0.0001):
                            #if the joint controller has finished its rotation
                            self.rot_down_flag = True #set the rotation down step as completed 
                            self.send_request = False
                            self.joint_rot_flag = False #set the joint controller as NOT finished its rotation
                            self.switch_controller(self.forward_cntr, self.joint_cntr) #switch from scaled_joint_trajectory controller to forward_position controller
                            #print("TOF1 filtered rot 0 to 10: ",self.tof1_filtered_rot[0:10])
                            #print("TOF2 filtered rot 0 to 10: ",self.tof2_filtered_rot[0:10])
                            self.start_time = time.time() #reset the start time 
                
                
                elif self.move_up_flag == False: 
                    #if the move up step has not be completed
                    if (now - self.start_time ) <self.move_collect: 
                        #if it hasn't been the alloted time for moving and collecting tof data
                        self.publish_twist([0.0, -0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at -0.1m/s
                    else:
                        #if the alloted time for moving and collecting tof data has passed 
                        self.publish_twist([0.0, 0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at 0.1m/s
                        self.move_down_to_y(self.desired_y)
                        if self.at_y == True:
                            #if the arm is at the wanted y position 
                            self.move_up_flag = True #set the move up step as completed 
                            self.at_y = False #set the arm as NOT at the wanted y position 
                            #print("TOF1 filtered up 0 to 10: ",self.tof1_filtered_up[0:10])
                            #print("TOF2 filtered up 0 to 10: ",self.tof2_filtered_up[0:10])
                            self.start_time = time.time() #reset the start time 
                            
                elif self.move_down_flag == False:
                    #if the move down step has not be completed 
                    print("IN MOVE DOWN FLAG")
                    if (now - self.start_time) <self.move_collect: 
                        #if it hasn't been the alloted time for moving and collecting tof data
                        self.publish_twist([0.0, 0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at 0.1m/s
                    else:
                        #if the alloted time for moving and collecting tof data has passed 
                        self.publish_twist([0.0, -0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at -0.1m/s
                        self.move_up_to_y(self.desired_y) # have to change this to move up 
                        if self.at_y == True: 
                            #if the arm is at the wanted y position
                            self.move_down_flag = True #set the move down step as completed 
                            self.at_y = False #set the arm as NOT at the wanted y position
                            #print("TOF1 filtered up 0 to 10: ",self.tof1_filtered_up[0:10])
                            #print("TOF2 filtered up 0 to 10: ",self.tof2_filtered_up[0:10])
                            #self.start_time = time.time() #reset the start time 
                            
                else: 
                    if self.calc_first_time == False: 
                        #if all the steps have been completed 
                        self.new_desired_angle = self.calculate_desired_angle()
                        self.new_desired_y = self.calculate_desired_y()
                        self.dif_angle = abs(self.desired_angle -self.new_desired_angle) 
                        self.dif_y = (self.desired_y - self.new_desired_y)
                        print(f"dif angle = {self.dif_angle}     dif_y = {self.dif_y}")
                        print(f"OLD desired_angle: {self.desired_angle} NEW desired_angle: {self.new_desired_angle}")
                        print(f"OLD desired_y: {self.desired_y} NEW desired_angle: {self.new_desired_y}")
                        print(f"Tries: {self.tries}")
                        self.calc_first_time = True 
                    if self.dif_angle <= 0.02 and abs(self.dif_y) <= 0.001:
                        #if the angles are within ~1 degree of each other and the y is within 1 mm of each other 
                        self.done = True  #set the check angle step as done 
                    else: 
                        #if the angles or y values are outside the given ranges 
                        if self.tries >3:
                            #if the check angles has tried more than 3 times 
                            self.tries = -1 
                            self.done = True #set the check angle step as done 
                        else: 
                            print("Trying to get to new pos ")
                            print(f"rotated to new: {self.rotated_to_new} moved to new: {self.moved_to_new}")
                            #if the check angle hasn't tried more than 3 times 
                            #self.tries += 1 #increase tries by one 
                            self.desired_angle = self.new_desired_angle #reset the desired_angle with the new_desired angle
                            if self.rotated_to_new == False: 
                                if self.send_request == False: 
                                    self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving
                                    self.switch_controller(self.joint_cntr, self.forward_cntr) #switch from forward_position controller to scaled_joint_trajectory controller
                                    self.rotate_to_w(self.desired_angle)
                                    self.send_request = True 
                                if (abs(self.desired_angle- self.tool_angle) < 0.001):
                                    self.switch_controller(self.forward_cntr, self.joint_cntr) #switch from scaled_joint_trajectory controller to forward_position controller
                                    self.rotated_to_new = True 
                
                            elif self.moved_to_new == False : 
                                self.desired_y = self.new_desired_y #reset the desired_y with the new_desired_y
                                if self.dif_y < 0: 
                                    self.publish_twist([0.0, 0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at 0.1m/s
                                    self.move_down_to_y(self.desired_y)
                                    if self.at_y == True:
                                        self.moved_to_new= True 
                                else: 
                                    self.publish_twist([0.0, -0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at 0.1m/s
                                    self.move_up_to_y(self.desired_y)
                                    if self.at_y == True:
                                        self.moved_to_new = True
                            else:
                                self.plot_tof()
                                self.tries += 1 #increase tries by one 
                                self.reset()
                        
                 
            else: 
                #if the check angle has gotten into the correct range or failed 4 times 
                if self.tries == -1:
                    #if tries is -1 meaning that the system has failed to get in the acceptable range after 4 tries 
                    print("Could not get a good location please restart")
                    self.step_2 = False
                    #self.response.position_found = False 
                else:
                    #if the system has gotten into the acceptable range in 4 or less tries  
                    print("found a good location")
                    plt.plot([1,2], [1,2])
                    plt.show()
                    #self.response.position_found = True
                    self.step_2 = False
                    self.switch_controller(self.forward_cntr, self.joint_cntr) #switch from scaled_joint_trajectory controller to forward_position controller
                    msg = Bool()
                    msg.data = True 
                    self.pub_step3.publish(msg)
                    
                 
        
    
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

    def callback_tof1 (self, msg):
        #collect raw tof1 data (in mm)
        now = time.time()
        if (now - self.start_time ) < self.move_collect:
             #if it hasn't been the alloted time for moving up and collecting tof data 
             self.tof1_readings.append(msg.data) #add raw data reading to list of tof1 readings 

    
    def callback_tof2(self, msg):
        #collect raw tof2 data (in mm)
        now = time.time()
        if (now - self.start_time ) < self.move_collect:
            #if it hasn't been the alloted time for moving up and collecting tof data
            self.tof2_readings.append(msg.data) #add raw data reading to list of tof2 readings 
    
    def callback_tof1_filtered(self, msg):
        #collect filtered tof1 data (in mm) and tool z orientation 
        now = time.time()
        if self.rot_up_flag == False or self.rot_down_flag == False: 
            #print("in orient collect ")
            if (self.start_time-now) < self.rot_collect:  
                self.tool_orient_tof1.append(self.tool_angle)
                self.tof1_filtered_rot.append(msg.data)
        else: 
            #print("in orient collect ")
            if (self.start_time-now) < self.move_collect:  
                self.tool_pos_tof1.append(self.tool_y)
                self.tof1_filtered_up.append(msg.data)


    
    def callback_tof2_filtered(self, msg):
        #collect filtered tof2 data (in mm) and tool z orientation 
        now = time.time()
        if self.rot_up_flag == False or self.rot_down_flag == False:
            if (self.start_time-now) < self.rot_collect:  
                self.tool_orient_tof2.append(self.tool_angle)
                self.tof2_filtered_rot.append(msg.data)
        else:
            if (self.start_time-now) < self.move_collect:  
                self.tool_pos_tof2.append(self.tool_y)
                self.tof2_filtered_up.append(msg.data)

    
    def move_down_to_y(self, y_pose_want):
        #Using the given y pose that we want to go back to, move down until we reach that location 
        y_pose= self.tool_y #get the current tool y pose 
        #print("y_pose: ", y_pose, " y_pose_want", y_pose_want)
        if (y_pose - y_pose_want) < 0.001: 
            #when within 1mm of wanted y pose 
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving  
            self.at_y = True
        return 
    
    def move_up_to_y(self, y_pose_want):
        #Using the given y pose that we want to go back to, move down until we reach that location 
        y_pose= self.tool_y #get the current tool y pose 
        #print("y_pose: ", y_pose, " y_pose_want", y_pose_want)
        if (y_pose_want - y_pose) < 0.001: 
            #when within 1mm of wanted y pose 
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
            self.at_y = True
        return 

    def rotate_to_w(self, angle):
        #rotate the tool to the given angle  
        names = self.joint_names 
        pos = np.array(self.joints) 

        #for all the joints, use the current angle for all joints, but wrist 3. Set wrist 3 to the given angle  
        for idx, vals in enumerate(names):
            if vals == "wrist_3_joint":
                pos[idx]= angle

        self.send_joint_pos(names, pos) 
        return 
    

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
    
    def get_tool_pose(self, time=None, as_array=True):
                #Get the position (position and orientation) of the tool pose in respect to the base in m 
                try:
                    #try to get the transform of from the base frame to the tool frame
                    tf = self.tf_buffer.lookup_transform(
                        self.tool_frame, self.base_frame, time or rclpy.time.Time()
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
                    return np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]) #return the tool position (position and orientation) with respect to the base in m 
                else:
                    return pose

    def get_tool_orient_z(self, as_array=True):
            #Get the z orientation of the tool pose in respect to the base in m 
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
                return o.z #return just the tool z orientation with respect to the base in m 
            else:
                return pose

    def pub_tool_pose_y(self):
        #save the current tool pose y 
        #there is a timer calling this function every 0.1 seconds 
        self.tool_y = self.get_tool_pose_y()

    def pub_tool_orient_z(self):
        #save the current tool orientation z  
        #there is a timer calling this function every 0.1 seconds 
        self.orient_z = self.get_tool_orient_z()
    
    def switch_controller(self, act, deact):
        #activate the act controllers given and deactivate the deact controllers given 
        switch_ctrl_req = SwitchController.Request(
            activate_controllers = [act], deactivate_controllers= [deact], strictness = 2
            ) #create request for activating and deactivating controllers with a strictness level of STRICT 
        self.switch_ctrl.call_async(switch_ctrl_req) #make a service call to switch controllers 
        print("Controllers have been switched")
        print(f"Activated: {act}  Deactivated: {deact}")
            
        return
    
    def send_joint_pos(self, joint_names, joints):
        #make a service call to moveit to go to given joint values 
        print(f"GOAL")
        #for n, p in zip (joint_names, joints):
         #    print(f"{n}: {p}")
        print(f"NOW SENDING")
        joint_constraints = [JointConstraint(joint_name=n, position=p) for n, p in zip(joint_names, joints)]
        kwargs = {"joint_constraints": joint_constraints}
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest(
            group_name="ur_manipulator",
            goal_constraints=[Constraints(**kwargs)],
            allowed_planning_time=5.0,
        ) #create a service request of given joint values and an allowed planning time of 5 seconds 

        goal_msg.planning_options = PlanningOptions(plan_only=False)

        self.moveit_planning_client.wait_for_server() #wait for service to be active 
        future = self.moveit_planning_client.send_goal_async(goal_msg) #make service call 
        future.add_done_callback(self.goal_complete) #set done callback

    def goal_complete(self, future):
            #function that is called once a service call is made to moveit_planning 
            rez = future.result()
            if not rez.accepted:
                print("Planning failed!")
                return
            else:
                print("Plan succeeded!")
                
            if future.done():
                #print("BACVK TO ORIGINAL ")
                self.joint_rot_flag = True 


    def callback_joints(self,msg ):
        #function that saves the current joint names and positions 
        self.joint_names = msg.name
        self.joints= msg.position

    def calculate_desired_angle(self):
        # find the new desired angle by finding the average of the tool z orientation at the lowest tof readings 
        #print(f"tool orient_tof1: {self.tool_orient_tof1}")
        #print(f"tool orient_tof1: {self.tool_orient_tof2}")
        low_angle_tof1 = self.tool_orient_tof1[np.argmin(self.tof1_filtered_rot)]
        low_angle_tof2 = self.tool_orient_tof2[np.argmin(self.tof2_filtered_rot)]
        low_tof1_reading = np.min(self.tof1_filtered_rot)
        low_tof2_reading = np.min(self.tof2_filtered_rot)
        high_tof1_reading = np.max(self.tof1_filtered_rot)
        high_tof2_reading = np.max(self.tof2_filtered_rot)

        print(f"Dif TOF1 is {abs (low_tof1_reading - high_tof1_reading)}")
        print(f"Dif TOF2 is {abs (low_tof2_reading - high_tof2_reading)}")
        print(f"idx of tof1 filtered_rot: {np.argmin(self.tof1_filtered_rot)}")
        print(f"idx of tof2 filtered_rot: {np.argmin(self.tof2_filtered_rot)}")
        print(f"valule of tool orient_tof1 {self.tool_orient_tof1[np.argmin(self.tof1_filtered_rot)]}")
        print(f"valule of tool orient_tof2 {self.tool_orient_tof2[np.argmin(self.tof2_filtered_rot)]}")
        if abs (low_tof1_reading - high_tof1_reading)>50 and abs (low_tof2_reading - high_tof2_reading)>50: 
            new_desired_angle = (low_angle_tof1 + low_angle_tof2)/2
        else: 
            print("The diferences were too small so the angle is good")
            new_desired_angle = self.desired_angle
            print(self.desired_angle)
        return new_desired_angle

    def calculate_desired_y(self):
        # find the new desired y by finding the average of the tool y pos at the lowest tof readings 
        low_y_tof1 = self.tool_pos_tof1[np.argmin(self.tof1_filtered_up)]
        low_y_tof2 = self.tool_pos_tof2[np.argmin(self.tof2_filtered_up)]
        new_desired_y = (low_y_tof1+low_y_tof2)/2
        return new_desired_y
    
    def reset(self):
        #reset the flags, data holders, and start time to prepare to start the checking system again 
        self.rot_up_flag = False
        self.rot_down_flag = False
        self.move_up_flag = False
        self.move_down_flag = False 
        self.joint_rot_flag = False
        self.at_y= False
        self.tof1_readings = [] 
        self.tof2_readings = [] 
        self.tool_orient_tof1 = [] 
        self.tool_orient_tof2 = [] 
        self.tof1_filtered_rot = [] 
        self.tof2_filtered_rot = [] 
        self.tof1_filtered_up = [] 
        self.tof2_filtered_up = [] 
        self.tool_pos_tof1 = [] 
        self.tool_pos_tof2 = [] 
        self.start_time = time.time()
        self.send_request = False
        self.rotated_to_new = False
        self.moved_to_new = False 
        self.calc_first_time = False 
        

    def calback_step2_flag(self,msg):
        self.step_2= msg.data

    def pub_tool_angle(self):
        try:
            names = self.joint_names 
            pos = np.array(self.joints) 
            for idx,vals in enumerate(names):
                if vals == "wrist_3_joint":
                    self.tool_angle = pos[idx] 
        except:
            print("Tool angle could not be found ")

    def plot_tof(self):
        #plot the the tof readings 
        # y axis is tof reading in mm
        # x axis is number tof reading 
        t = []
        t_f = []
        t2 = []
        t2_f = []
        t_orient = []
        t_orient2 = []
        
        for idx, val in enumerate(self.tof1_filtered_rot): 
            t.append(idx)
        
        for idx, val in enumerate(self.tof2_filtered_rot): 
            t2.append(idx)

        for idx, val in enumerate(self.tof1_filtered_up): 
            t_f.append(idx)
        
        for idx, val in enumerate(self.tof2_filtered_up): 
            t2_f.append(idx)
        
        for idx, val in enumerate(self.tool_orient_tof1):
            t_orient.append(idx )

        for idx, val in enumerate(self.tool_orient_tof2):
            t_orient2.append(idx )
       
        plt.plot(t,self.tof1_filtered_rot,'b', label = 'TOF1 filtered rot ')
        plt.plot(t2, self.tof2_filtered_rot, 'r', label = 'TOF2 filtered rot ')
        plt.plot(t_f, self.tof1_filtered_up, 'c', label = 'TOF1 filtered up')
        plt.plot(t2_f, self.tof2_filtered_up, 'k', label = 'TOF2 filtered up')
        plt.legend()
        plt.show()
        plt.plot(t_orient, self.tool_orient_tof1, 'b', label = 'TOF1 tool orient')
        plt.plot(t_orient2, self.tool_orient_tof2, 'r', label = 'TOF2 tool orient')
        plt.legend()
        plt.show()


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
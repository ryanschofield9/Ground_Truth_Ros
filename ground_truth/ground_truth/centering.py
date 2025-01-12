#TO DO CHECK IF WE NEED ALL THE IMPORTS 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.optimize import curve_fit

from geometry_msgs.msg import PoseStamped, TransformStamped, Point, TwistStamped, Vector3
from tf2_ros import TransformException

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
import numpy as np

from std_msgs.msg import Float32, Int64, Bool

from sensor_msgs.msg import JointState
import time as t
import matplotlib.pyplot as plt
import math 

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
)
from rclpy.action import ActionClient

import datetime
import logging



#from filterpy.kalman import KalmanFilter

class CenteringCleaned(Node):
    def __init__(self):
        # TO DO: CHECK WHAT CAN BE DELETED
        #TO DO: CAN GET RID OF TOF1 and TOF2 unfiltered data because we don't need it 

        super().__init__('center_cleaned')
        #Create publishers and subscripers (and timers as necessary )
        self.sub_tof1 = self.create_subscription(Float32, 'tof1', self.callback_tof1, 10) 
        self.sub_tof2 = self.create_subscription(Float32, 'tof2', self.callback_tof2, 10)
        self.sub_tof1 = self.create_subscription(Float32, 'tof1_filter', self.callback_tof1_filtered, 10)
        self.sub_tof2 = self.create_subscription(Float32, 'tof2_filter', self.callback_tof2_filtered, 10)
        self.sub_joints = self.create_subscription(JointState, 'joint_states',self.callback_joints, 10 )
        self.sub_reset = self.create_subscription(Bool, 'reset',self.calback_reset, 10 )
        self.sub_step1 = self.create_subscription(Bool, 'step_1',self.callback_step_1, 10 )
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pub_step2 = self.create_publisher(Bool, 'step2', 10)
        self.pub_step3 = self.create_publisher(Bool, 'step3', 10)
        self.pub_angle =self.create_publisher(Float32, 'angle',10)
        self.pub_camera_show = self.create_publisher(Bool, 'showing_camera', 10)
        self.pub_correction =self.create_publisher(Bool, 'correction',10)
        self.pub_timer = self.create_timer(1/10, self.main_control)
        self.tool_timer = self.create_timer(1/10, self.pub_tool_pose_y)
        self.tool_timer_angle = self.create_timer(1/50,self.pub_tool_angle)

        #Create tf buffer and listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Create Callback group
        self.service_handler_group = ReentrantCallbackGroup()
        
        #Create clients 
        self.enable_servo = self.create_client(
            Trigger, "/servo_node/start_servo", callback_group=self.service_handler_group
        )
        self.disable_servo = self.create_client(
            Trigger, "/servo_node/stop_servo", callback_group=self.service_handler_group
        )
        self.switch_ctrl = self.create_client(
            SwitchController, "/controller_manager/switch_controller", callback_group=self.service_handler_group
        )

        self.moveit_planning_client = ActionClient(self, MoveGroup, "move_action")

        #inital states
        self.servo_active = False #if the servos have been activated 
        self.tof_collected = False #if the tof data has been collected 
        self.calc_angle_done = False #if the angle of the branch has calculated 
        self.move_down = False #if the system has reached the center of the branch 
        self.done_step1= False #if the system has gotten parallel with the branch 
        self.step_1 = False #if it is time for step 1 to start 
        self.first = False #if this is the first time running through and the start time needs to be set  
        self.move_down_initialize = False #if the syste has moved down to start data collection 
        self.collect_data = False # if the system is collecting data 


        #constant variables 
        self.forward_cntr = 'forward_position_controller' #name of controller that uses velocity commands 
        self.joint_cntr = 'scaled_joint_trajectory_controller' # name of controller that uses joint commands 
        self.base_frame = 'base_link' #base frame that doesn't move 
        self.tool_frame = 'tool0' #frame that the end effector is attached to 
        self.move_up_collect = 7 #alloted time in seconds for moving up and collecting tof data  
        self.move_down_start = 2 # alloted time in seconds for moving down to start tf and position correctly
        self.dis_sensors = 0.0508 # meters 
        self.branch_angle = 0 #initial angle of branch

        #initialize variables 
        self.tof1_readings = [] #holds raw tof data during tof collection period for tof1 
        self.tof2_readings = [] #holds raw tof data during tof collection period for tof2
        self.tof1_filtered = [] #holds tof data during tof collection period for tof1 
        self.tof2_filtered = [] #holds tof data during tof collection period for tof2
        self.tof1_inrange = [] #holds all the tof readings in the range 50 mm to 500 mm for tof1 
        self.tof2_inrange = [] #holds all the tof readings in the range 50 mm to 500 mm for tof2 
        self.tof1_pos_inrange = [] #holds all the poses for the readings in the range 50 mm to 500 mm for tof1 
        self.tof2_pos_inrange = [] #holds all the poses for the readings in the range 50 mm to 500 mm for tof2
        #anytime tof is used, it is filtered
        #if the raw data is being used it will be mentioned 
        self.lowest_tof1 = 550 #start with value that can not be saved 
        self.lowest_tof2 = 550 #start with values that can not be saved
        self.lowest_pos_tof1 = None #y tool position at the lowest raw tof1 reading 
        self.lowest_pos_tof2 = None #y tool position at the lowest raw tof2 reading 
        self.tool_y = None #y position of the tool at the current moment 
        self.tool_angle = 0 # storing the tool angle 

        #start servoing and switch to forward position controller 
        self.start_servo()
        self.switch_controller(self.forward_cntr, self.joint_cntr)
        
        #set start_time 
        self.start_time = t.time() 

        #create logging 
        time_file = datetime.datetime.now()
        time_formated = time_file.strftime("_%Y_%m_%d_%H_%M_%S")
        log_file = "/home/ryan/ros2_ws_groundtruth/src/Ground_Truth_Ros/ground_truth/log_files/Centering Log File" + time_formated +".log"
        logging.basicConfig(filename=log_file, level=logging.INFO,
                            format='%(asctime)s - %(levelname)s - %(message)s')

#TO DO, DON'T NEED TO DO MUCH WITH RAW DATA IN THIS ANYMORE 

    def main_control (self): 
        #TO DO: RESTRUCTURE FOR MORE ORGANIZATION 
        #TO DO: GET RID OF ALL THE PRINT STATEMENTS 
        #TO DO: DETERMINE NEED OF ALL THE COMMENTS 
        #this function is called every 0.1 seconds and holds the main control structure for getting parallel to the branch 
        if self.step_1: 
            if self.first:
                self.start_time = t.time()
                self.first = False
                self.switch_controller(self.forward_cntr, self.joint_cntr)
                msg = Bool()
                msg.data = True 
                for x in range (0,3):
                    self.pub_camera_show.publish(msg)
            if self.done_step1 == False: 
                #if the system has not yet gotten parallel to the branch with a first guess 
                if self.move_down_initialize == False:
                    now = t.time()
                    if (now - self.start_time ) < self.move_down_start:
                         #if it hasn't been the alloted time for moving down
                        self.publish_twist([0.0, 0.1, 0.0], [0.0, 0.0, 0.0]) #move down at 0.1 m/s (negative y is up ) 
                    else:
                        self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving (move at 0 m/s) 
                        self.move_down_initialize = True 
                        self.move_down_start = 2
                        self.start_time = t.time()
                elif self.tof_collected == False: 

                    self.collect_data = True
                    # if tof data has not been collected 
                    now = t.time()
                    if (now - self.start_time ) < self.move_up_collect:
                        #if it hasn't been the alloted time for moving up and collecting tof data 
                        self.publish_twist([0.0, -0.1, 0.0], [0.0, 0.0, 0.0]) #move up at 0.1 m/s (negative y is up ) 
                    else:
                        #if the alloted time for moving up and collected tof data has passed 
                        self.tof_collected = True #set tof data collection to true 
                        self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving (move at 0 m/s) 
                        self.collect_data = False

                elif self.calc_angle_done ==False:
                    #if the angle of the branch hasn't been calculated 
                    self.create_fit()
                    self.calculate_angle() 

                elif self.move_down == False: 
                    #if the angle has been calculated, but the system has not reached the center of the branch
                    self.publish_twist([0.0, 0.1, 0.0], [0.0, 0.0, 0.0])  #move down at 0.05 m/s (negative y is up )
                    #y_pose_want = (self.lowest_pos_tof1+self.lowest_pos_tof2)/2 #find y_pose_want as the average of y tool position at the lowest filtered tof readings
                    idx_tof1_cleaned = (np.argmin(self.y_fit_tof1)/len(self.x_fit_tof1) ) * len(self.tof1_inrange_cleaned) + (len(self.tof1_inrange) - len(self.tof1_inrange_cleaned))
                    idx_tof2_cleaned = (np.argmin(self.y_fit_tof2)/len(self.x_fit_tof2) ) * len(self.tof2_inrange_cleaned) + (len(self.tof2_inrange) - len(self.tof2_inrange_cleaned))
                    interpolated_reading_tof1 = (idx_tof1_cleaned - math.ceil(idx_tof1_cleaned))* self.tof1_pos_inrange[math.ceil(idx_tof1_cleaned)] + (1-( (idx_tof1_cleaned - math.ceil(idx_tof1_cleaned))))* self.tof1_pos_inrange[math.floor(idx_tof1_cleaned)]
                    interpolated_reading_tof2 = (idx_tof2_cleaned - math.ceil(idx_tof2_cleaned))* self.tof2_pos_inrange[math.ceil(idx_tof2_cleaned)] + (1-( (idx_tof2_cleaned - math.ceil(idx_tof2_cleaned))))* self.tof2_pos_inrange[math.floor(idx_tof2_cleaned)]
                    y_pose_want = (interpolated_reading_tof1+ interpolated_reading_tof2)/2
                    self.move_down_to_y(y_pose_want) 

                elif self.move_down == True: 
                    #if the angle has been calculated and the system has reached the center of the branch
                    self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving (move at 0 m/s) 
                    print("calculated angle needed to rotate: ", self.branch_angle)
                    #print("TOF1: ", self.tof1_filtered)
                    #print("TOF2: ", self.tof2_filtered)
                    self.switch_controller(self.joint_cntr, self.forward_cntr) #switch from forward_position controller to scaled_joint_trajectory controller 
                    self.rotate_to_w(self.branch_angle)
                    self.plot_tof()
                    logging.info(f"TOF1: { self.tof1_filtered}" )
                    logging.info(f"TOF1 in range: { self.tof1_inrange}" )

                    logging.info(f"TOF2: { self.tof2_filtered}" )
                    logging.info(f"TOF2 in range: { self.tof2_inrange}" )
                    self.done_step1 = True #the system has gotten parallel with the branch, set as True 
                    msg= Bool()
                    msg.data = True
                    #publish to step_2 topic true 3 times to ensure that it gets there 
                    for x in range (0, 3):
                        #self.pub_step3.publish(msg)
                        self.pub_correction.publish(msg)
                    #self.switch_controller(self.forward_cntr, self.joint_cntr) #switch from scaled_joint_trajectory controller to forward position controller
                    self.step_1 = False
                    #possible add reset here   

        
    
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
        if self.collect_data:
            now = t.time()
            if (now - self.start_time ) < self.move_up_collect:
                #if it hasn't been the alloted time for moving up and collecting tof data 
                self.tof1_readings.append(msg.data) #add raw data reading to list of tof1 readings 

    
    def callback_tof2(self, msg):
        #collect raw tof2 data (in mm)
        if self.collect_data:
            now = t.time()
            if (now - self.start_time ) < self.move_up_collect:
                #if it hasn't been the alloted time for moving up and collecting tof data
                self.tof2_readings.append(msg.data) #add raw data reading to list of tof2 readings 
    
    def callback_tof1_filtered(self, msg):
        #collect and use filtered tof1 data (in mm)
        if self.collect_data:
            now = t.time()
            if (now - self.start_time ) < self.move_up_collect:
                #if it hasn't been the alloted time for moving up and collecting tof data
                self.tof1_filtered.append(msg.data) #add data reading to list of tof1 readings
                if 50 < msg.data < 500:
                    #if the tof1 reading is between 50 mm and 500mm (~5.9in to 20in)
                    self.tof1_inrange.append(msg.data )
                    self.tof1_pos_inrange.append(self.tool_y)
                    if msg.data < self.lowest_tof1:
                        #if the tof1 reading is lower that the previous lowest tof1 reading
                        self.lowest_tof1 = msg.data #set the tof1 reading as the lowest tof1 reading
                        self.lowest_pos_tof1= self.tool_y #save the current tool position as the lowest tool position for tof1
        
    def callback_tof2_filtered(self, msg):
        #collect and use filtered tof2 data (in mm)
        if self.collect_data:
            now = t.time()
            if (now - self.start_time ) < self.move_up_collect:
                #if it hasn't been the alloted time for moving up and collecting tof data
                self.tof2_filtered.append(msg.data) #add data reading to list of tof2 readings
                if 50 < msg.data < 500:
                    #if the tof2 reading is between 50 mm and 500mm 
                    self.tof2_inrange.append(msg.data )
                    self.tof2_pos_inrange.append(self.tool_y)
                    if msg.data < self.lowest_tof2:
                        #if the tof2 reading is lower that the previous lowest tof2 reading
                        self.lowest_tof2 = msg.data #set the tof2 reading as the lowest tof2 reading
                        self.lowest_pos_tof2= self.tool_y #save the current tool position as the lowest tool position for tof2


    def calculate_angle(self):
        #calculate the angle the branch is at based on the tof readings 
        print("TOF1: ", self.tof1_filtered)
        print("TOF1: ", min(self.tof1_filtered))
        print("TOF2: ", self.tof2_filtered)
        print("TOF2: ", min(self.tof2_filtered))
        print("TOF1 poses: ", self.tof1_pos_inrange)
        print("TOF2 poses: ", self.tof2_pos_inrange)
        print("argmin tof1: ",np.argmin(self.y_fit_tof1) )
        print("argmin tof1: ",np.argmin(self.y_fit_tof2) )
        
        
        idx_tof1_cleaned = (np.argmin(self.y_fit_tof1)/len(self.x_fit_tof1) ) * len(self.tof1_inrange_cleaned) + (len(self.tof1_inrange) - len(self.tof1_inrange_cleaned))

        idx_tof2_cleaned = (np.argmin(self.y_fit_tof2)/len(self.x_fit_tof2) ) * len(self.tof2_inrange_cleaned) + (len(self.tof2_inrange) - len(self.tof2_inrange_cleaned))
    
        print("Idx of tof1_cleaned: ", idx_tof1_cleaned)
        print("idx of tof1 poses: ", len(self.tof1_pos_inrange))
        print("tof1 poses rounded : ", self.tof1_pos_inrange[round(idx_tof1_cleaned)])
        print("tof2 poses averaged : ", (self.tof1_pos_inrange[math.ceil(idx_tof1_cleaned)]+ self.tof1_pos_inrange[math.floor(idx_tof1_cleaned)])/2)
        print("Idx of tof2_cleaned: ", idx_tof2_cleaned)
        print("idx of tof2 poses: ", len(self.tof2_pos_inrange))
        print("tof1 poses rounded : ", self.tof2_pos_inrange[round(idx_tof2_cleaned)])
        print("tof2 poses averaged : ", (self.tof2_pos_inrange[math.ceil(idx_tof2_cleaned)]+ self.tof2_pos_inrange[math.floor(idx_tof2_cleaned)])/2)
        print("Distance_readings new: ", self.tof1_pos_inrange[round(idx_tof1_cleaned)] - self.tof2_pos_inrange[round(idx_tof2_cleaned)] )
        print("Angle new: ", np.arctan( (self.tof1_pos_inrange[round(idx_tof1_cleaned)] - self.tof2_pos_inrange[round(idx_tof2_cleaned)])/ self.dis_sensors) + self.tool_angle)
        ## TO DO: THE MATH FOR INTERPOLATED IS WRONG NEED TO LOOK INTO 
        #try:
        distance_readings = self.lowest_pos_tof1 - self.lowest_pos_tof2 #find the distance between the lowest tof readings 
        distance_readings_rounded = self.tof1_pos_inrange[round(idx_tof1_cleaned)] - self.tof2_pos_inrange[round(idx_tof2_cleaned)]
        average_reading_tof1 = (self.tof1_pos_inrange[math.ceil(idx_tof1_cleaned)]+ self.tof1_pos_inrange[math.floor(idx_tof1_cleaned)])/2
        average_reading_tof2 =  (self.tof2_pos_inrange[math.ceil(idx_tof2_cleaned)]+ self.tof2_pos_inrange[math.floor(idx_tof2_cleaned)])/2
        distance_reading_averaged = average_reading_tof1 - average_reading_tof2
        interpolated_reading_tof1 = abs(idx_tof1_cleaned - math.ceil(idx_tof1_cleaned))* self.tof1_pos_inrange[math.floor(idx_tof1_cleaned)] + (1-abs( (idx_tof1_cleaned - math.ceil(idx_tof1_cleaned))))* self.tof1_pos_inrange[math.ceil(idx_tof1_cleaned)]
        interpolated_reading_tof2 = abs((idx_tof2_cleaned - math.ceil(idx_tof2_cleaned)))* self.tof2_pos_inrange[math.floor(idx_tof2_cleaned)] + (1-abs( (idx_tof2_cleaned - math.ceil(idx_tof2_cleaned))))* self.tof2_pos_inrange[math.ceil(idx_tof2_cleaned)]
        distance_reading_interpolated = interpolated_reading_tof1 - interpolated_reading_tof2
        print("Distance_readings: ", distance_readings )
        print("Distance reading rounded: ", distance_readings_rounded)
        print("Distance reading averaged: ", distance_reading_averaged)
        print("Low tof1 idx: ",[math.floor(idx_tof1_cleaned)])
        print("Low tof1 position: ", self.tof1_pos_inrange[math.floor(idx_tof1_cleaned)])
        print("High tof1 position: ",self.tof1_pos_inrange[math.ceil(idx_tof1_cleaned)] )
        print("Low tof2 position: ", self.tof2_pos_inrange[math.floor(idx_tof2_cleaned)])
        print("High tof2 position: ",self.tof2_pos_inrange[math.ceil(idx_tof2_cleaned)] )
        print("Distance reading interpolated: ", distance_reading_interpolated)
        branch_angle = np.arctan(distance_readings / self.dis_sensors) + self.tool_angle #using the distance between the sensors (known) and the lowest filtered tof readings (calculated), calculate the angle, also have to add the current angle
        branch_angle_rounded = np.arctan(distance_readings_rounded / self.dis_sensors) + self.tool_angle
        branch_angled_averaged = np.arctan(distance_reading_averaged / self.dis_sensors) + self.tool_angle
        branch_angled_interpolated = np.arctan(distance_reading_interpolated / self.dis_sensors) + self.tool_angle
        self.get_logger().info(f"Branch Angle: {branch_angle}")
        self.get_logger().info(f"Branch Angle Rounded: {branch_angle_rounded}")
        self.get_logger().info(f"Branch Angle Averaged: {branch_angled_averaged}")
        self.get_logger().info(f"Branch Angle Interpolated: {branch_angled_interpolated}")
        self.branch_angle = branch_angled_interpolated
        self.get_logger().info(f"Sending branch angle as: {self.branch_angle}")
        self.calc_angle_done = True #set calculate angle done flag to true
        msg = Float32()
        msg.data = self.branch_angle
        for x in range (0,3):
            self.pub_angle.publish(msg)
        #except:
            #print("No branch was found. The system will restart its moving up process") 
            #self.move_down_start += self.move_up_collect 
            #self.move_down_initialize = False 
            #self.start_time = t.time()
            #self.tof_collected = False 

    def pub_tool_angle(self):
        try:
            names = self.joint_names 
            pos = np.array(self.joints) 
            for idx,vals in enumerate(names):
                if vals == "wrist_3_joint":
                    self.tool_angle = pos[idx] 
        except:
            print("Tool angle could not be found ")
    
    def move_down_to_y(self, y_pose_want):
        #Using the given y pose that we want to go back to, move down until we reach that location 
        y_pose= self.tool_y #get the current tool y pose 
        if (y_pose - y_pose_want) < -0.005: 
            #when within 1mm of wanted y pose 
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
            self.move_down = True #set move done flag to True 
            self.control_switch = False # set move control switch flag to False to get it reset   
    
    def rotate_to_w(self, angle):
        #rotate the tool to the given angle  
        names = self.joint_names 
        pos = self.joints 

        #for all the joints, use the current angle for all joints, but wrist 3. Set wrist 3 to the given angle  
        for idx, vals in enumerate (names):
            if vals == "wrist_3_joint":
                pos[idx]= angle

        self.send_joint_pos(names, pos)  

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
                return p.y #return just the tool y position with respect to the base in m 
            else:
                return pose
            
    def pub_tool_pose_y(self): 
        #there is a timer calling this function every 0.1 seconds and save the current y tool pose 
        self.tool_y = self.get_tool_pose_y()

    def create_fit(self):
        self.t_fit = []
        self.t_2_fit = []
        tof1_inrange_cleaned = []
        tof2_inrange_cleaned = []
        cleaned = False
        cleaned_2 = False
        count = 0 
        count_2 = 0 

        for x in range (1, len(self.tof1_inrange)):
            if cleaned == False: 
                if self.tof1_inrange[x]- self.tof1_inrange[x-1] > 0: 
                    count = count 
                else:
                    count += 1 
                    if count == 3: 
                        cleaned = True 
                        tof1_inrange_cleaned.append(self.tof1_inrange[x-2])
                        tof1_inrange_cleaned.append(self.tof1_inrange[x-1])
                        tof1_inrange_cleaned.append(self.tof1_inrange[x])
            else: 
                 tof1_inrange_cleaned.append(self.tof1_inrange[x])
        

        for x in range (1, len(self.tof2_inrange)):
            if cleaned_2 == False: 
                if self.tof2_inrange[x]- self.tof2_inrange[x-1] > 0: 
                    count_2 = 0 
                else:
                    count_2 += 1 
                    if count_2 == 3:
                        cleaned_2 = True 
                        tof2_inrange_cleaned.append(self.tof2_inrange[x-2])
                        tof2_inrange_cleaned.append(self.tof2_inrange[x-1])
                        tof2_inrange_cleaned.append(self.tof2_inrange[x-0])
                    
            else: 
                 tof2_inrange_cleaned.append(self.tof2_inrange[x])

        for idx, val in enumerate(tof1_inrange_cleaned): 
            self.t_fit.append(idx)
        
        for idx, val in enumerate(tof2_inrange_cleaned): 
            self.t_2_fit.append(idx)

      

        self.tof1_inrange_cleaned = tof1_inrange_cleaned
        self.tof2_inrange_cleaned = tof2_inrange_cleaned

        self.get_logger().info(f"in range tof1 : {self.tof1_inrange}")
        self.get_logger().info(f"len in range tof1 : {len(self.tof1_inrange)}")
        self.get_logger().info(f"in range cleaned tof1 : {self.tof1_inrange_cleaned}")
        self.get_logger().info(f"len in range cleaned tof1 : {len(self.tof1_inrange_cleaned)}")

        self.get_logger().info(f"in range tof2 : {self.tof2_inrange}")
        self.get_logger().info(f"len in range tof2 : {len(self.tof2_inrange)}")
        self.get_logger().info(f"in range cleaned tof2 : {self.tof2_inrange_cleaned}")
        self.get_logger().info(f"len in range cleaned tof2 : {len(self.tof2_inrange_cleaned)}")

        params_tof1, covariance_tof1 = curve_fit(parabola, self.t_fit, tof1_inrange_cleaned)
        params_tof2, covariance_tof2 = curve_fit(parabola, self.t_2_fit, tof2_inrange_cleaned)
        a1, b1, c1 = params_tof1
        a2, b2, c2 = params_tof2
        self.x_fit_tof1 = np.linspace(min(self.t_fit), max(self.t_fit), 500)
        self.x_fit_tof2 = np.linspace(min(self.t_2_fit), max(self.t_2_fit), 500)
        self.y_fit_tof1 = parabola(self.x_fit_tof1, a1, b1, c1)
        self.y_fit_tof2 = parabola(self.x_fit_tof2, a2, b2, c2)

        print("len x_fit tof1: ", len(self.x_fit_tof1))
        


    def plot_tof(self):
        #plot the the tof readings 
        # y axis is tof reading in mm
        # x axis is number tof reading 

        t = []
        t_f = []
        t2 = []
        t2_f = []
        
        for idx, val in enumerate(self.tof1_readings): 
            t.append(idx)
        
        for idx, val in enumerate(self.tof2_readings): 
            t2.append(idx)

        for idx, val in enumerate(self.tof1_filtered): 
            t_f.append(idx)
        
        for idx, val in enumerate(self.tof2_filtered): 
            t2_f.append(idx)
    
       
        #plt.plot(t,self.tof1_readings,'b', label = 'TOF1 raw readings')
        #plt.plot(t2, self.tof2_readings, 'r', label = 'TOF2 raw readings')
        #plt.plot(t_f, self.tof1_filtered, 'c', label = 'TOF1 filtered reading')
        #plt.plot(t2_f, self.tof2_filtered, 'k', label = 'TOF2 filtered reading')
        #plt.legend()
        #plt.show()
        
        plt.plot(self.t_fit, self.tof1_inrange_cleaned, label = "TOF1 raw cleaned in range")
        plt.plot(self.t_2_fit, self.tof2_inrange_cleaned, label = "TOF1 raw cleaned in range")

        #plt.scatter(self.t, self.tof1_inrange_cleaned, label='Tof 1 ')
        #plt.scatter(self.t_2, self.tof2_inrange_cleaned, label='Tof 2 ')
        plt.plot(self.x_fit_tof1, self.y_fit_tof1, color='r', label='Fitted parabola tof1 ')
        plt.plot(self.x_fit_tof2, self.y_fit_tof2, color='b', label='Fitted parabola tof2 ')
        plt.legend()
        plt.show()
    
        
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
                
    def start_servo(self):
        #start the arms servos 
        print("in start")
        if self.servo_active:
            #if the servo is already active 
            print ("Servo is already active")
        else:
            #if the servo has not yet been activated 
            self.enable_servo.call_async(Trigger.Request()) #make a service call to activate the servos 
            self.active = True #set the servo active flag as true 
            print("Servo has been activated") 
        return
    
    def switch_controller(self, act, deact):
        #activate the act controllers given and deactivate the deact controllers given 
        switch_ctrl_req = SwitchController.Request(
            activate_controllers = [act], deactivate_controllers= [deact], strictness = 2
            ) #create request for activating and deactivating controllers with a strictness level of STRICT 
        self.switch_ctrl.call_async(switch_ctrl_req) #make a service call to switch controllers 
        print(f"Activated: {act}  Deactivated: {deact}")
            
        return
    
    def send_joint_pos(self, joint_names, joints):
        #make a service call to moveit to go to given joint values 
        for n, p in zip (joint_names, joints):
             print(f"{n}: {p}")
        print(f"NOW SENDING GOAL TO MOVE GROUP")
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

    def callback_joints(self,msg ):
        #function that saves the current joint names and positions 
        self.joint_names = msg.name
        self.joints= msg.position

    def calback_reset(self,msg): 
        if msg.data == True: 
            self.reset()
        
    def reset(self):
        print("resetting")
        #reinital states
        self.servo_active = True
        self.tof_collected = False 
        self.calc_angle_done = False 
        self.move_down = False 
        self.done_step1 = False  
        self.step_1 = False 
        self.move_down_initialize = False 

        #initialize variables 
        self.tof1_readings = [] 
        self.tof2_readings = []
        self.tof1_filtered = []  
        self.tof2_filtered = [] 
        self.tof1_inrange = [] 
        self.tof2_inrange = []  
        self.tof1_pos_inrange = [] 
        self.tof2_pos_inrange = [] 
        self.lowest_tof1 = 550 
        self.lowest_tof2 = 550 
        self.lowest_pos_tof1 = None 
        self.lowest_pos_tof2 = None  
        self.tool_y = None

        #switch controller 
        self.switch_controller(self.forward_cntr, self.joint_cntr) #switch from scaled_joint_trajectory controller to forward position controller 
        
        #start_time will be reset by doing this
        self.first = True

    def callback_step_1 (self,msg):
        print("In step 1 ")
        self.reset()
        self.step_1 = True 

def convert_tf_to_pose(tf: TransformStamped):
    #take the tf transform and turn that into a position  
    pose = PoseStamped() #create a pose which is of type Pose stamped
    pose.header = tf.header
    tl = tf.transform.translation
    pose.pose.position = Point(x=tl.x, y=tl.y, z=tl.z)
    pose.pose.orientation = tf.transform.rotation

    return pose

def parabola(x, a, b, c):
    return a * x**2 + b * x + c

def main(args=None):
    rclpy.init(args=args)
    center = CenteringCleaned()
    rclpy.spin(center)
    rclpy.shutdown ()

if __name__ == '__main__':
   main()
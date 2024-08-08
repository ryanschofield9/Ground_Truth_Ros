#TO DO CHECK IF WE NEED ALL THE IMPORTS 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TransformStamped, Point, TwistStamped, Vector3
from tf2_ros import TransformException

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
import numpy as np

from std_msgs.msg import Float32
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
import scipy.optimize

from filterpy.kalman import KalmanFilter

class MoveArm(Node):
    def __init__(self):
        # TO DO: CHECK WHAT CAN BE DELETED

        super().__init__('center_cleaned')
        #Create publishers and subscripers (and timers as necessary )
        self.sub_tof1 = self.create_subscription(Float32, 'tof1', self.callback_tof1, 10) 
        self.sub_tof2 = self.create_subscription(Float32, 'tof2', self.callback_tof2, 10)
        self.sub_tof1 = self.create_subscription(Float32, 'tof1_filter', self.callback_tof1_filtered, 10)
        self.sub_tof2 = self.create_subscription(Float32, 'tof2_filter', self.callback_tof2_filtered, 10)
        self.sub_joints = self.create_subscription(JointState, 'joint_states',self.callback_joints, 10 )
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pub_timer = self.create_timer(1/10, self.main_control)
        self.tool_timer = self.create_timer(1/10, self.pub_tool_pose_y)
        self.tool_timer_orient_z = self.create_timer(1/10,self.get_tool_orient_z )

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
        self.done = False #if the system has gotten parallel with the branch 

        self.rot_up_flag = False 
        self.rot_down_flag = False 
        self.move_up_flag = False 
        self.move_down_flag = False 
        self.joint_rot_flag = False 
        self.done = False 
        self.tries = 0 

        #Wait three seconds to let everything get up and running (may not need)
        time.sleep(3)

        #constant variables 
        self.forward_cntr = 'forward_position_controller' #name of controller that uses velocity commands 
        self.joint_cntr = 'scaled_joint_trajectory_controller' # name of controller that uses joint commands 
        self.base_frame = 'base_link' #base frame that doesn't move 
        self.tool_frame = 'tool0' #frame that the end effector is attached to 
        self.move_collect = 5 #alloted time in seconds for moving up and collecting tof data  
        self.dis_sensors = 0.0508 # meters 
        self.branch_angle = 0 #initial angle of branch
        self.rot_collect = 5 #alloted time in seconds for rotating and collecting tof data  

        #initialize variables 
        self.tof1_readings = [] #holds raw tof data during tof collection period for tof1 
        self.tof2_readings = [] #holds raw tof data during tof collection period for tof2
        self.tof1_filtered = [] #holds tof data during tof collection period for tof1 
        self.tof2_filtered = [] #holds tof data during tof collection period for tof2
        #anytime tof is used, it is filtered
        #if the raw data is being used it will be mentioned 
        self.lowest_tof1 = 550 #start with value that can not be saved 
        self.lowest_tof2 = 550 #start with values that can not be saved
        self.lowest_pos_tof1 = None #y tool position at the lowest raw tof1 reading 
        self.lowest_pos_tof2 = None #y tool position at the lowest raw tof2 reading 
        self.tool_y = None #y position of the tool at the current moment 
        self.orient_z = None #z orientation of the tool at the current moment 
        self.count_tof1 = 0 # counts how many readings there have been for tof1 
        self.count_tof2 = 0 #counts how many readings there have been for tof2 
        self.tool_orient_tof1 = [] # holds the tool orientation everytime a new tof reading comes in for tof1
        self.tool_orient_tof2 = [] # holds the tool orientation everytime a new tof reading comes in for tof2
        self.tof1_filtered_rot = [] #holds tof data during tof rotation collection period for tof1
        self.tof2_filtered_rot = [] #holds tof data during tof rotation collection period for tof2
        self.tof1_filtered_up = [] #holds tof data during tof moving collection period for tof1
        self.tof2_filtered_up = [] #holds tof data during tof moving collection period for tof2
        self.tool_pos_tof1 = [] # holds the tool position everytime a new tof reading comes in for tof1
        self.tool_pos_tof2 = [] # holds the tool position everytime a new tof reading comes in for tof2

        #start servoing and switch to forward position controller 
        self.start_servo()
        self.switch_controller(self.forward_cntr, self.joint_cntr)
        
        self.desired_angle = 0.0 #need a way to get this #maybe make this a service and call the service giving the starting angle
        self.desired_y = 0.0 # need to get this value  
        #set start_time 
        self.start_time = time.time() 

#TO DO, DON'T NEED TO DO MUCH WITH RAW DATA IN THIS ANYMORE 

    def main_control (self): 
        #Correct the angle by continously collecting tof data and comparing where the low value was found 
        if self.done == False:
            now = time.time()
            if self.rot_up_flag == False: 
                if (self.start_time - now) <self.rot_collect: 
                    self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.5]) #rotate the tool in the z direction by 0.5 rads/s
                else:
                    self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
                    self.switch_controller(self.joint_cntr, self.forward_cntr)
                    self.rotate_to_w(self, self.desired_angle)
                    if self.joint_rot_flag == True:
                        self.switch_controller(self.forward_cntr, self.joint_cntr)
                        self.rot_up_flag = True
                        self.joint_rot_flag = False
                        self.start_time = time.time()

            elif self.rot_down_flag == False:
                if (self.start_time - now) <self.rot_collect:
                    self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, -0.5]) #rotate the tool in the z direction by -0.5 rads/s
                else:
                    self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving
                    self.switch_controller(self.joint_cntr, self.forward_cntr)
                    self.rotate_to_w(self, self.desired_angle)
                    if self.joint_rot_flag == True:
                        self.switch_controller(self.forward_cntr, self.joint_cntr)
                        self.rot_down_flag = True
                        self.joint_rot_flag = False
                        self.start_time = time.time()
            elif self.move_up_flag == False: 
                if (self.start_time - now) <self.move_collect: 
                    self.publish_twist([0.0, -0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at -0.1m/s
                else:
                    self.publish_twist([0.0, 0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at 0.1m/s
                    self.move_down_to_y(self.desired_y)
                    self.move_up_flag = True
                    self.start_time = time.time()
            elif self.move_down_flag == False: 
                if (self.start_time - now) <self.move_collect: 
                    self.publish_twist([0.0, 0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at 0.1m/s
                else:
                    self.publish_twist([0.0, -0.1, 0.0], [0.0, 0.0, 0.0]) #move the tool in the y direction at -0.1m/s
                    self.move_up_to_y(self.desired_y) # have to change this to move up 
                    self.move_down_flag = True
                    self.start_time = time.time()
            else: 
                #WRITE FUNCTION TO CALCULATE THE NEW DESIRED ANGLES AND NEW DESIRED Y 
                new_desired_angle = self.calculate_desired_angle()
                new_desired_y = self.calculate_desired_y()
                if self.desired_angle == new_desired_angle and self.desired_y == new_desired_y:
                    self.done = True  
                else: 
                    if self.tries >3:
                        self.tries = -1
                        self.done = True 
                    else: 
                        self.tries += 1 
                        self.reset()
                    
                        
        else: 
            if self.tries == -1: 
                print("Could not get a good location please restart")
            else: 
                print("found a good location")



        
    
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

    def callback_tof1 (self, msg):
        #collect raw tof1 data (in mm)
        now = time.time()
        if (now - self.start_time ) < self.move_up_collect:
             #if it hasn't been the alloted time for moving up and collecting tof data 
             self.tof1_readings.append(msg.data) #add raw data reading to list of tof1 readings 

    
    def callback_tof2(self, msg):
        #collect raw tof2 data (in mm)
        now = time.time()
        if (now - self.start_time ) < self.move_up_collect:
            #if it hasn't been the alloted time for moving up and collecting tof data
            self.tof2_readings.append(msg.data) #add raw data reading to list of tof2 readings 
    
    def callback_tof1_filtered(self, msg):
        #collect filtered tof1 data (in mm) and tool z orientation 
        now = time.time()
        if self.rot_up_flag or self.rot_down == False: 
            if (self.start_time-now) < self.rot_collect:  
                self.tool_orient_tof1.append(self.orient_z)
                self.tof1_filtered_rot.append(msg.data)
        else: 
            if (self.start_time-now) < self.move_collect:  
                self.tool_pos_tof1.append(self.tool_y)
                self.tof1_filtered_up.append(msg.data)


    
    def callback_tof2_filtered(self, msg):
        #collect filtered tof2 data (in mm) and tool z orientation 
        now = time.time()
        if self.rot_up_flag or self.rot_down == False:
            if (self.start_time-now) < self.rot_collect:  
                self.tool_orient_tof2.append(self.orient_z)
                self.tof2_filtered_rot.append(msg.data)
        else:
            if (self.start_time-now) < self.move_collect:  
                self.tool_pos_tof2.append(self.tool_y)
                self.tof2_filtered_up.append(msg.data)


    def calculate_angle(self):
        #calculate the angle the branch is at based on the tof readings 
        print("tof1 readings: ", self.tof1_readings)
        print("tof2 readings: ", self.tof2_readings)
        print("lowest y pose for tof1: ", self.lowest_pos_tof1)
        print("lowest y pose for tof2: ", self.lowest_pos_tof2)
        print("actual pose y value: ", self.tool_y)
        print("actual pose: ", self.get_tool_pose())
        distance_readings = self.lowest_pos_tof1 - self.lowest_pos_tof2 #find the distance between the lowest tof readings 
        self.branch_angle = np.arctan(distance_readings / self.dis_sensors) #using the distance between the sensors (known) and the lowest filtered tof readings (calculated), calculate the angle
        self.calc_angle_done = True #set calculate angle done flag to true 


    
    def move_down_to_y(self, y_pose_want):
        #Using the given y pose that we want to go back to, move down until we reach that location 
        y_pose= self.tool_y #get the current tool y pose 
        print("y_pose: ", y_pose, " y_pose_want", y_pose_want)
        if (y_pose - y_pose_want) < 0.001: 
            #when within 1mm of wanted y pose 
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
            self.move_down = True #set move done flag to True 
        return 
    
    def move_up_to_y(self, y_pose_want):
        #Using the given y pose that we want to go back to, move down until we reach that location 
        y_pose= self.tool_y #get the current tool y pose 
        print("y_pose: ", y_pose, " y_pose_want", y_pose_want)
        if (y_pose_want - y_pose) < 0.001: 
            #when within 1mm of wanted y pose 
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
            self.move_down = True #set move done flag to True 
        return 

    def rotate_to_w(self, angle):
        #rotate the tool to the given angle  
        names = self.joint_names 
        pos = self.joints 

        #for all the joints, use the current angle for all joints, but wrist 3. Set wrist 3 to the given angle  
        for idx, vals in enumerate (names):
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
        #save the current tool pose y 
        #there is a timer calling this function every 0.1 seconds 
        self.orient_z = self.get_tool_orient_z()

    def plot_tof(self):
        #plot the the tof readings 
        # y axis is tof reading in mm
        # x axis is number tof reading 
        t = []
        t_f = []
        t2 = []
        t2_f = []
        found = False
        for idx, val in enumerate(self.tof1_filtered): 
            t_f.append(idx)
            if val == min(self.tof1_filtered) and found == False:
                lowest_x_f = idx
                found = True
        
        found = False
        for idx, val in enumerate(self.tof2_filtered): 
            t2_f.append(idx)
            if val == min(self.tof2_filtered) and found == False:
                lowest_x2_f = idx
                found = True

        plt.plot(t,self.tof1_readings,'b', label = 'TOF1 raw readings')
        plt.plot(t2, self.tof2_readings, 'r', label = 'TOF2 raw readings')
        plt.plot(t_f, self.tof1_filtered, 'c', label = 'TOF1 filtered reading')
        plt.plot(t2_f, self.tof2_filtered, 'k', label = 'TOF2 filtered reading')
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
        print("Controllers have been switched")
        print(f"Activated: {act}  Deactivated: {deact}")
            
        return
    
    def send_joint_pos(self, joint_names, joints):
        #make a service call to moveit to go to given joint values 
        print(f"GOAL")
        for n, p in zip (joint_names, joints):
             print(f"{n}: {p}")
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
            
            if rez.SUCCEEDED:
                self.joint_rot_flag = True 
            else: 
                self.joint_rot_flag = False

    def callback_joints(self,msg ):
        #function that saves the current joint names and positions 
        self.joint_names = msg.name
        self.joints= msg.position

    def calculate_desired_angle(self):
        low_angle_tof1 = self.tool_orient_tof1[np.argmin(self.tof1_filtered_rot)]
        low_angle_tof2 = self.tool_orient_tof2[np.argmin(self.tof2_filtered_rot)]
        new_desired_angle = (low_angle_tof1+low_angle_tof2)/2
        return new_desired_angle

    def calculate_desired_y(self):
        low_y_tof1 = self.tool_pos_tof1[np.argmin(self.tof1_filtered_up)]
        low_y_tof2 = self.tool_pos_tof2[np.argmin(self.tof2_filtered_up)]
        new_desired_y = (low_y_tof1+low_y_tof2)/2
        return new_desired_y
    
    def reset(self):
        self.rot_up_flag = False
        self.rot_down_flag = False
        self.move_up_flag = False
        self.move_down_flag = False 
        self.joint_rot_flag = False 
        self.start_time = time.time()

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
    move = MoveArm()
    rclpy.spin(move)
    rclpy.shutdown ()

if __name__ == '__main__':
   main()
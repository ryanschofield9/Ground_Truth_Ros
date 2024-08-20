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

from std_msgs.msg import Float32, Int64, Bool
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
from groun_truth_msgs.srv import AngleCheck, MoveY



#from filterpy.kalman import KalmanFilter

class MoveArm(Node):
    def __init__(self):
        # TO DO: CHECK WHAT CAN BE DELETED

        super().__init__('center_cleaned')
        #Create publishers and subscripers (and timers as necessary )
        self.sub_tof1 = self.create_subscription(Int64, 'tof1', self.callback_tof1, 10) 
        self.sub_tof2 = self.create_subscription(Int64, 'tof2', self.callback_tof2, 10)
        self.sub_tof1 = self.create_subscription(Float32, 'tof1_filter', self.callback_tof1_filtered, 10)
        self.sub_tof2 = self.create_subscription(Float32, 'tof2_filter', self.callback_tof2_filtered, 10)
        self.sub_joints = self.create_subscription(JointState, 'joint_states',self.callback_joints, 10 )
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pub_step2 = self.create_publisher(Bool, 'step2', 10)
        #self.pub_timer = self.create_timer(1/10, self.main_control)
        self.tool_timer = self.create_timer(1/10, self.pub_tool_pose_y)

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
        #self.angle_check_client = self.create_client(AngleCheck, 'angle_check')
        self.move_y_client = self.create_client(MoveY, 'move_y_direction')
        while not self.move_y_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for service to start')

        #inital states
        self.servo_active = False #if the servos have been activated 
        self.tof_collected = False #if the tof data has been collected 
        self.calc_angle_done = False #if the angle of the branch has calculated 
        self.move_down = False #if the system has reached the center of the branch 
        self.done_step1= False #if the system has gotten parallel with the branch
        self.done_step2= False #if the system has checked that the system is parallel wiuth the brnach   

        #Wait three seconds to let everything get up and running (may not need)
        time.sleep(3)

        #constant variables 
        self.forward_cntr = 'forward_position_controller' #name of controller that uses velocity commands 
        self.joint_cntr = 'scaled_joint_trajectory_controller' # name of controller that uses joint commands 
        self.base_frame = 'base_link' #base frame that doesn't move 
        self.tool_frame = 'tool0' #frame that the end effector is attached to 
        self.move_up_collect = 8 #alloted time in seconds for moving up and collecting tof data  
        self.dis_sensors = 0.0508 # meters 
        self.branch_angle = 0 #initial angle of branch

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

        #start servoing and switch to forward position controller 
        self.start_servo()
        self.switch_controller(self.forward_cntr, self.joint_cntr)
        
        #set start_time 
        self.start_time = time.time() 
        self.main_control()

#TO DO, DON'T NEED TO DO MUCH WITH RAW DATA IN THIS ANYMORE 

    def main_control (self): 
        #TO DO: RESTRUCTURE FOR MORE ORGANIZATION 
        #TO DO: GET RID OF ALL THE PRINT STATEMENTS 
        #TO DO: DETERMINE NEED OF ALL THE COMMENTS 
        #this function is called every 0.1 seconds and holds the main control structure for getting parallel to the branch 
        collect_tof_request =MoveY.Request()
        collect_tof_request.time = self.move_up_collect 
        collect_tof_request.direction = "Up" 
        future = self.move_y_client.call_async(collect_tof_request)
        print("Done ")
        
    
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
        #collect and use filtered tof1 data (in mm)
        now = time.time()
        if (now - self.start_time ) < self.move_up_collect:
            #if it hasn't been the alloted time for moving up and collecting tof data
            self.tof1_filtered.append(msg.data) #add data reading to list of tof1 readings
            if 150 < msg.data < 500:
                #if the tof1 reading is between 150 mm and 500mm (~5.9in to 20in)
                if msg.data < self.lowest_tof1:
                    #if the tof1 reading is lower that the previous lowest tof1 reading
                    self.lowest_tof1 = msg.data #set the tof1 reading as the lowest tof1 reading
                    self.lowest_pos_tof1= self.tool_y #save the current tool position as the lowest tool position for tof1
    
    def callback_tof2_filtered(self, msg):
        #collect and use filtered tof2 data (in mm)
        now = time.time()
        if (now - self.start_time ) < self.move_up_collect:
            #if it hasn't been the alloted time for moving up and collecting tof data
            self.tof2_filtered.append(msg.data) #add data reading to list of tof2 readings
            if 150 < msg.data < 500:
                #if the tof2 reading is between 150 mm and 500mm (~5.9in to 20in)
                if msg.data < self.lowest_tof2:
                    #if the tof2 reading is lower that the previous lowest tof2 reading
                    self.lowest_tof2 = msg.data #set the tof2 reading as the lowest tof2 reading
                    self.lowest_pos_tof2= self.tool_y #save the current tool position as the lowest tool position for tof2


    def calculate_angle(self):
        #calculate the angle the branch is at based on the tof readings 
        print("tof1 readings: ", self.tof1_readings)
        print("tof2 readings: ", self.tof2_readings)
        print("tof1 filtered readings: ", self.tof1_filtered)
        print("tof2 filtered readings: ", self.tof2_filtered)
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
            self.control_switch = False # set move control switch flag to False to get it reset  
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
            
    def pub_tool_pose_y(self):
        #save the current tool pose y 
        #there is a timer calling this function every 0.1 seconds 
        self.tool_y = self.get_tool_pose_y()

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

    def callback_joints(self,msg ):
        #function that saves the current joint names and positions 
        self.joint_names = msg.name
        self.joints= msg.position

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
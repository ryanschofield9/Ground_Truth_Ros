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

from std_msgs.msg import Int64, Float32
from sensor_msgs.msg import JointState
import time 
import matplotlib.pyplot as plt

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
)
from rclpy.action import ActionClient
import scipy.optimize

from filterpy.kalman import KalmanFilter

class MoveArm(Node):
    def __init__(self):
        super().__init__('center_cleaned')
        #Create publishers and subscripers (and timers as necessary )
        self.sub_tof1 = self.create_subscription(Float32, 'tof1', self.callback_tof1, 10)
        self.sub_tof2 = self.create_subscription(Float32, 'tof2', self.callback_tof2, 10)
        self.sub_joints = self.create_subscription(JointState, 'joint_states',self.callback_joints, 10 )
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pub_timer = self.create_timer(1/10, self.main_control)

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
        self.servo_active = False
        self.tof_collected = False
        self.calc_angle_done = False
        self.move_down = False
        self.done = False

        #Wait three seconds to let everything get up and running (may not need)
        time.sleep(3)

        #constant variables 
        self.forward_cntr = 'forward_position_controller'
        self.joint_cntr = 'scaled_joint_trajectory_controller'
        self.base_frame = 'base_link'
        self.tool_frame = 'tool0'
        self.move_up_collect = 6 #seconds needed to move up and collect tof data 
        self.dis_sensors = 0.0508 # meters 
        self.branch_angle = 0

        #initialize variables 
        self.tof1_readings = []
        self.tof2_readings = []
        self.lowest_reading_tof1 = 500 #start with value that can not be saved 
        self.lowest_reading_tof2 = 500 #start with value that can not be saved 
        self.lowest_pos_tof1 = self.get_tool_pose_y()
        self.lowest_pos_tof2 = self.get_tool_pose_y()



        #start servoing and switch to forward position controller 
        self.start_servo()
        self.switch_controller(self.forward_cntr, self.joint_cntr)
        
        #set start_time 
        self.start_time = time.time() 


    def main_control (self): 
        if self.tof_collected == False: 
            # if tof data has not been collected 
            now = time.time()
            if (now - self.start_time ) < self.move_up_collect:
                self.publish_twist([0.0, -0.1, 0.0], [0.0, 0.0, 0.0]) #moving up at 0.1 m/s (negative y is up )
            else:
                self.tof_collected = True
                self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 

        elif self.done == False: 
            if self.calc_angle_done ==False:
                self.calculate_angle()
                y_pose_want = (self.lowest_pos_tof1 + self.lowest_pos_tof2)/2
                print ("y_pose_want: ", y_pose_want)
            elif self.move_down == False: 
                self.publish_twist([0.0, 0.05, 0.0], [0.0, 0.0, 0.0]) #move down slowly 
                y_pose_want = (self.lowest_pos_tof1 + self.lowest_pos_tof2)/2
                self.move_down_to_y(y_pose_want)
            elif self.move_down == True:  
                self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
                y_pose= self.get_tool_pose_y()
                print("final pose at", y_pose)
                print("calculated angle needed to rotate: ", self.branch_angle)
                self.switch_controller(self.joint_cntr, self.forward_cntr)
                self.rotate_to_w(self.branch_angle)
                self.plot_tof()
                self.done = True
        
    
    def publish_twist(self, linear_speed, rot_speed):
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
        now = time.time()
        if (now - self.start_time ) < self.move_up_collect:
             self.tof1_readings.append(msg.data)
             #Add here if between 150 and 400 save the lowest joint pos 
             if 150 < msg.data < 500:
                if msg.data < self.lowest_reading_tof1:
                    self.lowest_reading_tof1 = msg.data
                    self.lowest_pos_tof1 = self.get_tool_pose_y()

    
    def callback_tof2(self, msg):
        now = time.time()
        if (now - self.start_time ) < self.move_up_collect:
             self.tof2_readings.append(msg.data)
             #Add here if between 150 and 400 save the lowest joint pos 
             if 150 < msg.data < 500:
                if msg.data < self.lowest_reading_tof2:
                    self.lowest_reading_tof2 = msg.data
                    self.lowest_pos_tof2= self.get_tool_pose_y()
    
    def calculate_angle(self):
        print("tof1 readings: ", self.tof1_readings)
        print("tof2 readings: ", self.tof2_readings)
        print("lowest y pose for tof1: ", self.lowest_pos_tof1)
        print("lowest y pose for tof2: ", self.lowest_pos_tof2)
        print("actual pose y value: ", self.get_tool_pose_y())
        print("actual pose: ", self.get_tool_pose())
        distance_readings = self.lowest_pos_tof1 - self.lowest_pos_tof2
        self.branch_angle = np.arctan(distance_readings / self.dis_sensors)
        self.calc_angle_done = True


    
    def move_down_to_y(self, y_pose_want):
        y_pose= self.get_tool_pose_y()
        print("y_pose: ", y_pose, " y_pose_want", y_pose_want)
        if (y_pose - y_pose_want) < 0.016: #these seems to get closest to the middle consistently, assuming the get y_pose lags a little 
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
            self.move_down = True
        return 
    
    def rotate_to_w(self, angle):
        names = self.joint_names 
        pos = self.joints 

        for idx, vals in enumerate (names):
            if vals == "wrist_3_joint":
                pos[idx]= angle

        self.send_joint_pos(names, pos)
        return 
    

    def get_tool_pose_y(self, as_array=True):
            try:
                current_time = rclpy.time.Time()
                tf = self.tf_buffer.lookup_transform(
                    self.tool_frame, self.base_frame, current_time
                )
                
            except TransformException as ex:
                self.get_logger().warn("Received TF Exception: {}".format(ex))
                return
            pose = convert_tf_to_pose(tf)
            if as_array:
                p = pose.pose.position
                o = pose.pose.orientation
                return p.y
            else:
                print(pose)
                return -100000
    
    def plot_tof(self):
        t = []
        t2 = []
        found = False
        for idx, val in enumerate(self.tof1_readings): 
            t.append(idx)
            if val == self.lowest_reading_tof1 and found == False:
                lowest_x = idx
                found = True
        
        found = False
        for idx, val in enumerate(self.tof2_readings): 
            t2.append(idx)
            if val == self.lowest_reading_tof2 and found == False:
                lowest_x2 = idx
                found = True
        plt.plot(t,self.tof1_readings, label = 'TOF1 readings')
        plt.plot(t2, self.tof2_readings, label = 'TOF2 readings')
        plt.plot([lowest_x, lowest_x], [self.lowest_reading_tof1-100,self.lowest_reading_tof1+100, ])
        plt.plot([lowest_x2, lowest_x2], [self.lowest_reading_tof2-100,self.lowest_reading_tof2+100, ])
        plt.show()
        
    
    def get_tool_pose(self, time=None, as_array=True):
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.tool_frame, self.base_frame, time or rclpy.time.Time()
                    )
                except TransformException as ex:
                    self.get_logger().warn("Received TF Exception: {}".format(ex))
                    return
                pose = convert_tf_to_pose(tf)
                if as_array:
                    p = pose.pose.position
                    o = pose.pose.orientation
                    return np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w])
                else:
                    return pose
                
    def start_servo(self):
        print("in start")
        if self.servo_active:
            print ("Servo is already active")
        else:
            self.enable_servo.call_async(Trigger.Request())
            self.active = True
            print("Servo has been activated") 
        return
    
    def switch_controller(self, act, deact):
        switch_ctrl_req = SwitchController.Request(
            activate_controllers = [act], deactivate_controllers= [deact], strictness = 2
            )
        self.switch_ctrl.call_async(switch_ctrl_req)
        print("Controllers have been switched")
        print(f"Activated: {act}  Deactivated: {deact}")
            
        return
    
    def send_joint_pos(self, joint_names, joints):
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
        )
        goal_msg.planning_options = PlanningOptions(plan_only=False)

        self.moveit_planning_client.wait_for_server()
        future = self.moveit_planning_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_complete)

    def goal_complete(self, future):
            rez = future.result()
            if not rez.accepted:
                print("Planning failed!")
                return
            else:
                print("Plan succeeded!")

    def callback_joints(self,msg ):
        
        self.joint_names = msg.name
        self.joints= msg.position

def convert_tf_to_pose(tf: TransformStamped):
    pose = PoseStamped()
    pose.header = tf.header
    tl = tf.transform.translation
    pose.pose.position = Point(x=tl.x, y=tl.y, z=tl.z)
    pose.pose.orientation = tf.transform.rotation

    return pose

def parabola(x, a, b, c):
    return a*x**2 + b*x + c

def main(args=None):
    rclpy.init(args=args)
    move = MoveArm()
    rclpy.spin(move)
    rclpy.shutdown ()

if __name__ == '__main__':
   main()
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TransformStamped, Point, TwistStamped, Vector3
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
import numpy as np

from std_msgs.msg import Int64, Float32
import time 
import matplotlib.pyplot as plt

class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')
        # create publishers and subscripers (and timers as necessary )
        self.sub_tof1 = self.create_subscription(Int64, 'tof1', self.callback_tof1, 10)
        self.sub_tof2 = self.create_subscription(Int64, 'tof2', self.callback_tof2, 10)
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pub_timer = self.create_timer(1/10, self.publish_twist)
        #add a publisher that will be able to publish position (x,y,z,rotation) or joint 
        #self.pub_joint_commands= 
        #add a subscriber that will be able to get position location (proferably x,y,z, rotation tool endeffector)
        #self.sub_loc = 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link'
        self.tool_frame = 'tool0'
    
        self.service_handler_group = ReentrantCallbackGroup()
        
        self.enable_servo = self.create_client(
            Trigger, "/servo_node/start_servo", callback_group=self.service_handler_group
        )
        self.disable_servo = self.create_client(
            Trigger, "/servo_node/stop_servo", callback_group=self.service_handler_group
        )
        self.switch_ctrl = self.create_client(
            SwitchController, "/controller_manager/switch_controller", callback_group=self.service_handler_group
        )

        #inital states
        self.servo_active = False
        self.forward_cntr = 'forward_position_controller'
        self.joint_cntr = 'scaled_joint_trajectory_controller'

        #Creating parameters 
        self.declare_parameter('speed',0.1)
        self.declare_parameter('distance', 20)
        self.timer_2 = self.create_timer(1, self.callback_timer)
        

        self.tof_collected = False
        self.move_up_collect = 6 #seconds needed to move up and collect tof data 
        self.tof1_readings = []
        self.tof2_readings = []
        self.lowest_reading_tof1 = 500 #start with value that can not be saved 
        self.lowest_reading_tof2 = 500 #start with value that can not be saved 
        self.calc_angle_done = False
        self.move_down = False
        self.dis_sensors = 0.0508 # meters 
        self.branch_angle = 0
        time.sleep(3)
        self.start_servo()
        self.switch_controller(self.forward_cntr, self.joint_cntr)
        print("done waiting")
        #self.tf_buffer.wait(self.base_frame, self.tool_frame)
        self.lowest_pos_tof1 = self.get_tool_pose_y()
        self.lowest_pos_tof2 = self.get_tool_pose_y()
        self.done = False
        self.count = 0 
        self.start_time = time.time() 


    def publish_twist(self): 
        my_twist_linear = [0.0, 0.0, 0.0] 
        my_twist_angular = [0.0, 0.0, 0.0]
        cmd = TwistStamped()
        cmd.header.frame_id = 'tool0'
        cmd.twist.angular = Vector3(x=my_twist_angular[0], y=my_twist_angular[1], z=my_twist_angular[2])
        if self.tof_collected == False: 
            # if tof data has not been collected 
            now = time.time()
            if (now - self.start_time ) < self.move_up_collect:
                #if it has not been the seconds needed to move up and collect tof data, keep moving up at 0.1 m/s
                my_twist_linear[1]=  -0.1 #moving up at 0.1 m/s (negative y is up )
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.twist.linear = Vector3(x=my_twist_linear[0], y=my_twist_linear[1], z=my_twist_linear[2])
                self.pub_vel_commands.publish(cmd)
                self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")
    
            
            else:
                self.tof_collected = True
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.twist.linear = Vector3(x=my_twist_linear[0], y=my_twist_linear[1], z=my_twist_linear[2])
                self.pub_vel_commands.publish(cmd)
                self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")

        elif self.done == False: 
            if self.calc_angle_done ==False:
                self.calculate_angle()
                y_pose_want = (self.lowest_pos_tof1 + self.lowest_pos_tof2)/2
                print ("y_pose_want: ", y_pose_want)
            elif self.move_down == False: 
                cmd = TwistStamped()
                cmd.header.frame_id = 'tool0'
                cmd.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.twist.linear = Vector3(x=0.0, y=0.05, z=0.0)
                self.pub_vel_commands.publish(cmd)
                y_pose_want = (self.lowest_pos_tof1 + self.lowest_pos_tof2)/2
                self.move_down_to_y(y_pose_want)
            elif self.move_down == True:  
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
                self.pub_vel_commands.publish(cmd)
                self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")
                y_pose= self.get_tool_pose_y()
                print("final pose at", y_pose)
                print("calculated angle needed to rotate: ", self.branch_angle)
                self.plot_tof()
                self.done = True
        

        #cmd = TwistStamped()
        #cmd.header.frame_id = 'tool0'
        #cmd.header.stamp = self.get_clock().now().to_msg()
        #cmd.twist.linear = Vector3(x=my_twist_linear[0], y=my_twist_linear[1], z=my_twist_linear[2])
        #cmd.twist.angular = Vector3(x=my_twist_angular[0], y=my_twist_angular[1], z=my_twist_angular[2])
        #self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")

        self.pub_vel_commands.publish(cmd)
    
    
    def callback_timer(self):
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.distance = self.get_parameter('distance').get_parameter_value().integer_value
    
    def callback_tof1 (self, msg):
        now = time.time()
        if (now - self.start_time ) < self.move_up_collect:
             self.tof1_readings.append(msg.data)
             #Add here if between 150 and 400 save the lowest joint pos 
             if 150 < msg.data < 400:
                if msg.data < self.lowest_reading_tof1:
                    self.lowest_reading_tof1 = msg.data
                    self.lowest_pos_tof1 = self.get_tool_pose_y()

    
    def callback_tof2(self, msg):
        now = time.time()
        if (now - self.start_time ) < self.move_up_collect:
             self.tof2_readings.append(msg.data)
             #Add here if between 150 and 400 save the lowest joint pos 
             if 150 < msg.data < 400:
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
        distance_readings = self.lowest_pos_tof2 - self.lowest_pos_tof1
        self.branch_angle = np.arctan(distance_readings / self.dis_sensors)
        self.calc_angle_done = True


    
    def move_down_to_y(self, y_pose_want):
        #self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")
        print("Time TEST!!!: ", rclpy.time.Time())
        cmd = TwistStamped()
        cmd.header.frame_id = 'tool0'
        cmd.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear = Vector3(x=0.0, y=0.05, z=0.0)
        y_pose= self.get_tool_pose_y()
        print("y_pose: ", y_pose, " y_pose_want", y_pose_want)
        if abs(y_pose - y_pose_want) < 0.01:
            self.pub_vel_commands.publish(cmd)
            self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")
            self.move_down = True
            print("YAY")

            
        return 
    
    def rotate_to_w(self, y_pose_want):
        #self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")
        print("Time TEST!!!: ", rclpy.time.Time())
        cmd = TwistStamped()
        cmd.header.frame_id = 'tool0'
        cmd.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear = Vector3(x=0.0, y=0.05, z=0.0)
        y_pose= self.get_tool_pose_y()
        print("y_pose: ", y_pose, " y_pose_want", y_pose_want)
        if abs(y_pose - y_pose_want) < 0.01:
            self.pub_vel_commands.publish(cmd)
            self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")
            self.move_down = True
            print("YAY")

            
        return 

    def get_tool_pose_y(self, as_array=True):
            try:
                print("Time: ", rclpy.time.Time())
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
            if val == self.lowest_reading_tof1 and found == False:
                lowest_x2 = idx
                found = True
            
        plt.plot(t,self.tof1_readings)
        plt.plot(t2, self.tof2_readings)
        plt.plot([lowest_x, lowest_x], [self.lowest_reading_tof1-100,self.lowest_reading_tof1+100, ])
        plt.plot([lowest_x2, lowest_x2], [self.lowest_reading_tof2-100,self.lowest_reading_tof2+100, ])
        plt.show()
        
    
    def get_tool_pose(self, time=None, as_array=True):
                try:
                    print("Time: ", rclpy.time.Time())
                    current_time = rclpy.time.Time()
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
                    print(pose)
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
def convert_tf_to_pose(tf: TransformStamped):
    pose = PoseStamped()
    pose.header = tf.header
    #print("tf header: ",tf.header)
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
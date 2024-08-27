#TO DO CHECK IF WE NEED ALL THE IMPORTS 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import  TwistStamped, Vector3


from std_msgs.msg import Float32, Int64, Float32MultiArray

from groun_truth_msgs.srv import MoveY
from groun_truth_msgs.msg import ToolPose 

from rclpy.callback_groups import ReentrantCallbackGroup


import time 

class MoveYService(Node):
    def __init__(self):
        # TO DO: CHECK WHAT CAN BE DELETED

        super().__init__('move_y_direction_service')
        #Create service 
        self.service_moveup = self.create_service(MoveY, 'move_y_direction', self.moving)

        #Create Callback group
        self.service_handler_group = ReentrantCallbackGroup()

        #Create publishers and subscripers (and timers as necessary )
        self.sub_tof1 = self.create_subscription(Int64, 'tof1', self.callback_tof1, 10, callback_group=self.service_handler_group) 
        self.sub_tof2 = self.create_subscription(Int64, 'tof2', self.callback_tof2, 10, callback_group=self.service_handler_group)
        self.sub_tof1 = self.create_subscription(Float32, 'tof1_filter', self.callback_tof1_filtered, 10, callback_group=self.service_handler_group)
        self.sub_tof2 = self.create_subscription(Float32, 'tof2_filter', self.callback_tof2_filtered, 10, callback_group=self.service_handler_group)
        self.sub_tool_pose = self.create_subscription(ToolPose,'tool_pose', self.callback_tool_pose, 10, callback_group=self.service_handler_group)
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        #self.pub_timer = self.create_timer(1/10, self.wait)

        #initialize variables 
        self.tof1_readings = [] #holds raw tof data during tof collection period for tof1 
        self.tof2_readings = [] #holds raw tof data during tof collection period for tof2
        self.tof1_filtered = [] #holds tof data during tof collection period for tof1 
        self.tof2_filtered = [] #holds tof data during tof collection period for tof2
        #anytime tof is used, it is filtered
        #if the raw data is being used it will be mentioned 
        self.tof1_tool_pose = []
        self.tof2_tool_pose = []
        self.lowest_tof1 = 550 #start with value that can not be saved 
        self.lowest_tof2 = 550 #start with values that can not be saved
        self.lowest_pos_tof1 = None #y tool position at the lowest raw tof1 reading 
        self.lowest_pos_tof2 = None #y tool position at the lowest raw tof2 reading 
        self.tool_y = None #y position of the tool at the current moment 
        self.collecting = False 

#TO DO, DON'T NEED TO DO MUCH WITH RAW DATA IN THIS ANYMORE 
    
    def moving (self, request, response): 
        #TO DO: RESTRUCTURE FOR MORE ORGANIZATION 
        #TO DO: GET RID OF ALL THE PRINT STATEMENTS 
        #TO DO: DETERMINE NEED OF ALL THE COMMENTS 
        #this function is called every 0.1 seconds and holds the main control structure for getting parallel to the branch 
        self.time_collect = request.time 
        if request.direction == "Down":
            self.speed = 0.1

        if request.direction == "Up":
            self.speed = -0.1  
        self.start_time = time.time()
        self.collecting = True
        #self.wait_timer = self.create_timer(5, self.release_wait)
        #now = time.time()
        self.publish_twist([0.0, self.speed, 0.0], [0.0, 0.0, 0.0]) #move up at 0.1 m/s (negative y is up )
        #rate = Node.rate(0.5)
        #rate.sleep()
        while (now - self.start_time ) < self.time_collect:
            #if it hasn't been the alloted time for moving and collecting tof data 
            self.publish_twist([0.0, self.speed, 0.0], [0.0, 0.0, 0.0])
            now = time.time() 
        #if the alloted time for moving up and collected tof data has passed  
        self.collecting = False 
        self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving (move at 0 m/s) 
        response = MoveY.Response()
        
        print(f"tof1 readings = {self.tof1_readings}")
        print(f"tof2 readings = {self.tof2_readings}")
        print(f"tof1 filtered = {self.tof1_filtered}")
        print(f"tof2 filtered = {self.tof2_filtered}")
        print(f'tof1 tool pose = {self.tof1_tool_pose}')
        print(f'tof2 tool pose = {self.tof2_tool_pose}')
        print(f"tof1 lowest pose = {self.lowest_pos_tof1}")
        print(f"tof2 lowest pose = {self.lowest_pos_tof2}")
        
        response.tof1 = self.tof1_readings
        response.tof2 = self.tof2_readings
        response.tof1_filtered = self.tof1_filtered
        response.tof2_filtered = self.tof2_filtered
        response.tof1_tool_pose = self.tof1_tool_pose
        response.lowest_pose_tof1 = self.lowest_pos_tof1
        response.tof2_tool_pose = self.tof2_tool_pose
        response.lowest_pose_tof2 = self.lowest_pos_tof2
        print("returning")
        return response 


    
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
        if self.collecting:
             #if moving and collecting data 
             self.tof1_readings.append(msg.data) #add raw data reading to list of tof1 readings 

    
    def callback_tof2(self, msg):
        #collect raw tof2 data (in mm)
        if self.collecting:
            #if moving and collecting data 
            self.tof2_readings.append(msg.data) #add raw data reading to list of tof2 readings 
    
    def callback_tof1_filtered(self, msg):
        #collect and use filtered tof1 data (in mm)
        if self.collecting:
           #if moving and collecting data 
            self.tof1_filtered.append(msg.data) #add data reading to list of tof1 readings
            self.tof1_tool_pose.append(self.tool_y)
            if 150 < msg.data < 500:
                #if the tof1 reading is between 150 mm and 500mm (~5.9in to 20in)
                if msg.data < self.lowest_tof1:
                     #if the tof1 reading is lower that the previous lowest tof1 reading
                    if self.tool_y != None:
                        #if the tool y pose is not None 
                        self.lowest_tof1 = msg.data #set the tof1 reading as the lowest tof1 reading
                        self.lowest_pos_tof1= self.tool_y #save the current tool position as the lowest tool position for tof1
    
    def callback_tof2_filtered(self, msg):
        print(f"here in tof2 filtered and collecting is {self.collecting}")
        #collect and use filtered tof2 data (in mm)
        if self.collecting:
            #if moving and collecting data 
            self.tof2_filtered.append(msg.data) #add data reading to list of tof2 readings
            self.tof2_tool_pose.append(self.tool_y)
            if 150 < msg.data < 500:
                #if the tof2 reading is between 150 mm and 500mm (~5.9in to 20in)
                if msg.data < self.lowest_tof2:
                    #if the tof2 reading is lower that the previous lowest tof2 reading
                    if self.tool_y != None:
                        #if the tool y pose is not None 
                        self.lowest_tof2 = msg.data #set the tof2 reading as the lowest tof2 reading
                        self.lowest_pos_tof2= self.tool_y #save the current tool position as the lowest tool position for tof2

    def callback_tool_pose (self, msg):
        self.tool_y  = msg.py
        #print(self.tool_y)
        #save the y pose which is in the second spot of the array 

    def wait(self):
        if self.collecting == True: 
            now = time.time()
            if  (now - self.start_time ) < self.time_collect:
            #if it hasn't been the alloted time for moving and collecting tof data 
                self.publish_twist([0.0, self.speed, 0.0], [0.0, 0.0, 0.0])
                now = time.time() 
            else:
                self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) # stop moving 

    def release_wait(self):
        self.wait_done = True  

def main(args=None):
    rclpy.init(args=args)
    move = MoveYService()
    executor = MultiThreadedExecutor()
    rclpy.spin(move, executor)
    rclpy.shutdown ()

if __name__ == '__main__':
   main()
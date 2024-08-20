#TO DO CHECK IF WE NEED ALL THE IMPORTS 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import  TwistStamped, Vector3


from std_msgs.msg import Float32, Int64, Float32MultiArray

from groun_truth_msgs.srv import MoveY

import time 

class MoveUpService(Node):
    def __init__(self):
        # TO DO: CHECK WHAT CAN BE DELETED

        super().__init__('move_y_direction_service')
        #Create service 
        self.service_moveup = self.create_service(MoveY, 'move_y_direction', self.moving)

        #Create publishers and subscripers (and timers as necessary )
        self.sub_tof1 = self.create_subscription(Int64, 'tof1', self.callback_tof1, 10) 
        self.sub_tof2 = self.create_subscription(Int64, 'tof2', self.callback_tof2, 10)
        self.sub_tof1 = self.create_subscription(Float32, 'tof1_filter', self.callback_tof1_filtered, 10)
        self.sub_tof2 = self.create_subscription(Float32, 'tof2_filter', self.callback_tof2_filtered, 10)
        self.sub_tool_pose = self.create_subscription(Float32MultiArray,'tool_pose', self.call_back_tool_pose, 10)
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

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
        self.collecting = True
        self.start_time = time.time()
        now = time.time()
        self.publish_twist([0.0, self.speed, 0.0], [0.0, 0.0, 0.0]) #move up at 0.1 m/s (negative y is up )
        while (now - self.start_time ) < self.time_collect:
            #if it hasn't been the alloted time for moving and collecting tof data 
            now = time.time() 
        #if the alloted time for moving up and collected tof data has passed  
        self.collecting = False 
        self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving (move at 0 m/s) 
        response = MoveY.Response()
        response.tof1 = self.tof1_readings
        response.tof2 = self.tof2_readings
        response.tof1_filtered = self.tof1_filtered
        response.tof2_filtered = self.tof2_filtered
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
        self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")

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
            if 150 < msg.data < 500:
                #if the tof1 reading is between 150 mm and 500mm (~5.9in to 20in)
                if msg.data < self.lowest_tof1:
                     #if the tof1 reading is lower that the previous lowest tof1 reading
                    if self.tool_y != None:
                        #if the tool y pose is not None 
                        self.lowest_tof1 = msg.data #set the tof1 reading as the lowest tof1 reading
                        self.lowest_pos_tof1= self.tool_y #save the current tool position as the lowest tool position for tof1
    
    def callback_tof2_filtered(self, msg):
        #collect and use filtered tof2 data (in mm)
        if self.collecting:
            #if moving and collecting data 
            self.tof2_filtered.append(msg.data) #add data reading to list of tof2 readings
            if 150 < msg.data < 500:
                #if the tof2 reading is between 150 mm and 500mm (~5.9in to 20in)
                if msg.data < self.lowest_tof2:
                    #if the tof2 reading is lower that the previous lowest tof2 reading
                    if self.tool_y != None:
                        #if the tool y pose is not None 
                        self.lowest_tof2 = msg.data #set the tof2 reading as the lowest tof2 reading
                        self.lowest_pos_tof2= self.tool_y #save the current tool position as the lowest tool position for tof2

    def callback_tool_pose (self, msg):
        pose = msg.data
        self.tool_y = pose[1]
        #save the y pose which is in the second spot of the array 

def main(args=None):
    rclpy.init(args=args)
    move = MoveUpService()
    rclpy.spin(move)
    rclpy.shutdown ()

if __name__ == '__main__':
   main()
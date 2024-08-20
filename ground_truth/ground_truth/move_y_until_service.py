#TO DO CHECK IF WE NEED ALL THE IMPORTS 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import  TwistStamped, Vector3


from std_msgs.msg import Float32, Int64, Float32MultiArray

from groun_truth_msgs.srv import MoveYUntil
from groun_truth_msgs.msg import ToolPose 

from rclpy.callback_groups import ReentrantCallbackGroup


import time 

class MoveUpService(Node):
    def __init__(self):
        # TO DO: CHECK WHAT CAN BE DELETED

        super().__init__('move_y_direction_service')
        #Create service 
        self.service_moveup = self.create_service(MoveYUntil, 'move_y_until', self.moving)

        #Create Callback group
        self.service_handler_group = ReentrantCallbackGroup()

        #Create publishers and subscripers (and timers as necessary )
        #self.sub_tof1 = self.create_subscription(Int64, 'tof1', self.callback_tof1, 10, callback_group=self.service_handler_group) 
        #self.sub_tof2 = self.create_subscription(Int64, 'tof2', self.callback_tof2, 10, callback_group=self.service_handler_group)
        #self.sub_tof1 = self.create_subscription(Float32, 'tof1_filter', self.callback_tof1_filtered, 10, callback_group=self.service_handler_group)
        #self.sub_tof2 = self.create_subscription(Float32, 'tof2_filter', self.callback_tof2_filtered, 10, callback_group=self.service_handler_group)
        self.sub_tool_pose = self.create_subscription(ToolPose,'tool_pose', self.callback_tool_pose, 10, callback_group=self.service_handler_group)
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)


#TO DO, DON'T NEED TO DO MUCH WITH RAW DATA IN THIS ANYMORE 

    def moving (self, request, response): 
        #TO DO: RESTRUCTURE FOR MORE ORGANIZATION 
        #TO DO: GET RID OF ALL THE PRINT STATEMENTS 
        #TO DO: DETERMINE NEED OF ALL THE COMMENTS 
        #this function is called every 0.1 seconds and holds the main control structure for getting parallel to the branch 
        y_want = request.y_want
        if request.direction == "Down":
            self.speed = 0.1
        else:
            self.speed = -0.1
        y_pose = self.tool_y
        self.publish_twist([0.0, self.speed, 0.0], [0.0, 0.0, 0.0]) #move up at 0.1 m/s (negative y is up )
        if request.direction == "Down":
            while (y_pose - y_want) > 0.001: 
                #if it hasn't been the alloted time for moving and collecting tof data 
                self.publish_twist([0.0, self.speed, 0.0], [0.0, 0.0, 0.0])
                y_pose = self.tool_y
                print(f"y_pose = {y_pose}")
                print(f"y_want = {y_want}")
        else: 
            while (y_want - y_pose ) > 0.001: 
                #if it hasn't been the alloted time for moving and collecting tof data 
                self.publish_twist([0.0, self.speed, 0.0], [0.0, 0.0, 0.0])
                y_pose = self.tool_y
                print(f"y_pose = {self.y_pose}")
                print(f"y_want = {self.y_want}")

            #now = time.time() 
        #if the alloted time for moving up and collected tof data has passed  
        #self.collecting = False 
        self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving (move at 0 m/s) 
        response = MoveYUntil.Response()
        response.result = True 
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

    def callback_tool_pose (self, msg):
        self.tool_y  = msg.py
        #print(self.tool_y)
        #save the y pose which is in the second spot of the array 

def main(args=None):
    rclpy.init(args=args)
    move = MoveUpService()
    executor = MultiThreadedExecutor()
    rclpy.spin(move, executor)
    rclpy.shutdown ()

if __name__ == '__main__':
   main()
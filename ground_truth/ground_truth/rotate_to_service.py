#TO DO CHECK IF WE NEED ALL THE IMPORTS 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import  TwistStamped, Vector3


from std_msgs.msg import Float32, Int64, Float32MultiArray

from groun_truth_msgs.srv import RotateTo
from groun_truth_msgs.msg import ToolPose 

from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
)
from rclpy.action import ActionClient

import time 

class MoveUpService(Node):
    def __init__(self):
        # TO DO: CHECK WHAT CAN BE DELETED

        super().__init__('move_y_direction_service')
        #Create service 
        self.service_moveup = self.create_service(RotateTo, 'rotate_to', self.moving)

        #Create Callback group
        self.service_handler_group = ReentrantCallbackGroup()

        #Create publishers and subscripers (and timers as necessary )
        #self.sub_tool_pose = self.create_subscription(ToolPose,'tool_pose', self.callback_tool_pose, 10, callback_group=self.service_handler_group)
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.sub_joints = self.create_subscription(JointState, 'joint_states',self.callback_joints, 10 )

        #Create clients 
        self.moveit_planning_client = ActionClient(self, MoveGroup, "move_action")

#TO DO, DON'T NEED TO DO MUCH WITH RAW DATA IN THIS ANYMORE 

    def rotating (self, request, response): 
        #TO DO: RESTRUCTURE FOR MORE ORGANIZATION 
        #TO DO: GET RID OF ALL THE PRINT STATEMENTS 
        #TO DO: DETERMINE NEED OF ALL THE COMMENTS 
        #this function is called every 0.1 seconds and holds the main control structure for getting parallel to the branch 
        
        return response 

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

    def callback_joints(self,msg ):
        #function that saves the current joint names and positions 
        self.joint_names = msg.name
        self.joints= msg.position

def main(args=None):
    rclpy.init(args=args)
    move = MoveUpService()
    executor = MultiThreadedExecutor()
    rclpy.spin(move, executor)
    rclpy.shutdown ()

if __name__ == '__main__':
   main()
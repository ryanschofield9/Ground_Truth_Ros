import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TransformStamped, Point, TwistStamped, Vector3
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
import time 
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

#used Alex's code in follow the leader controller.py to get this code 
class JointTest(Node):
    def __init__(self):
        super().__init__("test_joint")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link'
        self.tool_frame = 'tool0'
        self.done = False
        self.timer = self.create_timer(1, self.run)
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.moveit_planning_client = ActionClient(self, MoveGroup, "move_action")
        #self.pub_timer = self.create_timer(1/10, self.publish_twist)
        self.sub_joints = self.create_subscription(JointState, 'joint_states',self.callback, 10 )
        '''
        self.joint_names = [
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
                "shoulder_pan_joint",
        ]
        self.joints = [-1.59, -1.59, 0.0, 0.0, 0.0, 0.0]
        '''
        #time.sleep(5)
        

    def get_tool_pose(self, time=None, as_array=True):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.tool_frame, time or rclpy.time.Time()
                )
            except TransformException as ex:
                self.get_logger().warn("Received TF Exception: {}".format(ex))
                return
            pose = convert_tf_to_pose(tf)
            if as_array:
                p = pose.pose.position
                o = pose.pose.orientation
                print(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
                return np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w])
            else:
                print(pose)
                return pose
    
    def publish_twist(self): 
        my_twist_linear = [0.0, 0.0, 0.0] 
        my_twist_angular = [0.0, 0.0, -0.5]
        cmd = TwistStamped()
        cmd.header.frame_id = 'tool0'
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear = Vector3(x=my_twist_linear[0], y=my_twist_linear[1], z=my_twist_linear[2])
        cmd.twist.angular = Vector3(x=my_twist_angular[0], y=my_twist_angular[1], z=my_twist_angular[2])
        self.pub_vel_commands.publish(cmd)
        self.get_tool_pose()
    
    def send_joint_pos(self, joint_names, joints):
        print(f"GOAL")
        for n, p in zip (joint_names, joints):
             print(f"{n}: {p}")
        print(f"NOW SENDING")
        joint_constraints = [JointConstraint(joint_name=n, position=p) for n, p in zip(joint_names, joints)]
        print(joint_constraints)
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
                self.new = False 
                self.wait_until()
                self.print_angles()

    def callback(self,msg ):
        
        self.joint_names = msg.name
        self.joints= msg.position
        self.new = True

    def print_angles(self):
        for n, p in zip(self.joint_names, self.joints):
            print(f"{n}: {p}")
    
    def run(self):
        names = [
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
                "shoulder_pan_joint",
        ]
        pos = [-1.59, -1.59, 0.0, 0.0, 0.0, 0.0]
    
        if self.done == False: 
            self.print_angles()
            self.send_joint_pos(names, pos)
            self.done = True

    def wait_until (self): 
        start = time.time()
        while(self.new == False):

            now = time.time()
            if (now-start) > 5: 
                print("timeout")
                return 
        return 
        
            

def convert_tf_to_pose(tf: TransformStamped):
    pose = PoseStamped()
    pose.header = tf.header
    tl = tf.transform.translation
    pose.pose.position = Point(x=tl.x, y=tl.y, z=tl.z)
    pose.pose.orientation = tf.transform.rotation

    return pose

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    joint = JointTest()
    rclpy.spin(joint, executor)
    return

if __name__ == "__main__":
    main()
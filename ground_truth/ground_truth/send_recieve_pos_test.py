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
import time 



#used Alex's code in follow the leader controller.py to get this code 
class PosTest(Node):
    def __init__(self):
        super().__init__("test_pos")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link'
        self.tool_frame = 'tool0'
        self.timer = self.create_timer(1, self.get_tool_pose)
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        #self.pub_timer = self.create_timer(1/10, self.publish_twist)
        self.send_joint_pos()

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
    pos = PosTest()
    rclpy.spin(pos, executor)
    return

if __name__ == "__main__":
    main()
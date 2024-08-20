#TO DO CHECK IF WE NEED ALL THE IMPORTS 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from tf2_ros import TransformException

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

from std_msgs.msg import Float32MultiArray, Float32
from groun_truth_msgs.msg import ToolPose 

class PubToolPose(Node):
    def __init__(self):
        # TO DO: CHECK WHAT CAN BE DELETED

        super().__init__('pub_tool_pose')
        #Create publishers and subscripers (and timers as necessary )
        self.publish_tool_pose = self.create_publisher(ToolPose, 'tool_pose', 10)
        self.tool_timer = self.create_timer(1/10, self.pub_tool_pose)

        #Create tf buffer and listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #constant variables 
        self.base_frame = 'base_link' #base frame that doesn't move 
        self.tool_frame = 'tool0' #frame that the end effector is attached to 

    def pub_tool_pose(self):
        #publish the current tool pose 
        #there is a timer calling this function every 0.1 seconds 
        self.tool = self.get_tool_pose()
        #print(f"tool = {self.tool}")
        msg= ToolPose()
        msg.px = self.tool[0]
        msg.py = self.tool[1]
        msg.pz = self.tool[2]
        msg.ox = self.tool[3]
        msg.oy = self.tool[4]
        msg.oz = self.tool[5]
        msg.ow = self.tool[6]
        #print(f"tool = {self.tool}  data= {msg}")
        self.publish_tool_pose.publish(msg)
    
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
    pub = PubToolPose()
    rclpy.spin(pub)
    rclpy.shutdown ()

if __name__ == '__main__':
   main()
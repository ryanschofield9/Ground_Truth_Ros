#!/usr/bin/env python3

#code sent to me from Abhinav

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
from visualization_msgs.msg import Marker
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class JoystickToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, "servo_node/delta_twist_cmds", 10)
        self.enable_servo = self.create_client(Trigger, "/servo_node/start_servo")
        self.disable_servo = self.create_client(Trigger, "/servo_node/stop_servo")
        self.marker_pub = self.create_publisher(Marker, "/vel_marker", 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.switch_ctrl = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        while not self.switch_ctrl.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ctrl service not available, waiting again...')


        while not self.enable_servo.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servo service not available, waiting again...')
        switch_ctrl_req = SwitchController.Request(
            activate_controllers=["forward_position_controller"], deactivate_controllers=["scaled_joint_trajectory_controller"], strictness = 2
        )
        self.send_request(self.switch_ctrl, switch_ctrl_req)
        
        self.send_request(self.enable_servo, Trigger.Request())
        

    def send_request(self, srv, req):
        self.get_logger().info("Calling {} {}".format(srv, req))
        self.future = srv.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    # Convert incoming joy commands to TwistStamped commands for servoing.
    # The TwistStamped component goes to servoing, while buttons 0 & 1 control
    # joints directly.
    def joy_callback(self, msg):
        # Cartesian servoing with the axes
        vel_scale = 0.1
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = msg.axes[0] * vel_scale
        twist.twist.linear.y = msg.axes[1] * vel_scale
        twist.twist.linear.z = msg.axes[4] * vel_scale

        # twist.twist.angular.x = msg.axes[3] * vel_scale
        # twist.twist.angular.y = msg.axes[4] * vel_scale
        # twist.twist.angular.z = msg.axes[5] * vel_scale
        self.publish_marker([msg.axes[0], msg.axes[1], msg.axes[4]])
        self.twist_pub.publish(twist)
        self.get_logger().info(f"Sending twist: linear {(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z)} ")

    def publish_marker(self, action):

        try:
            time = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform("base_link", "wrist_3_link", time)
            ee_translation = tf.transform.translation
        except:
            self.get_logger().info("TF failed")
            return
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        start = Point()
        start.x = ee_translation.x
        start.y = ee_translation.y
        start.z = ee_translation.z

        end = Point()

        end.x = ee_translation.x + action[0]*0.05
        end.y = ee_translation.y + action[1]*0.05
        end.z = ee_translation.z + action[2]*0.05
        marker.points = [start, end]
        #print(start, end)
        # add start

        # publish marker
        self.marker_pub.publish(marker)
def main():
    rclpy.init()
    joy_servo_node = JoystickToTwist()
    rclpy.spin(joy_servo_node)
    joy_servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
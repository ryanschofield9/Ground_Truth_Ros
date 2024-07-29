import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import numpy as np
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from threading import Event, Lock

#using alex's work in follow the leader controller.py to get this
class SwitchControllers(Node):
    def __init__(self):
        super().__init__("switch_controller")

        # ROS2-based utility setup
        self.service_handler_group = ReentrantCallbackGroup()
        


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
       # self.srv_start = self.create_service(
        #   Trigger, "servo_start", self.start, callback_group=self.service_handler_group
        #)
        #self.srv_stop = self.create_service(Trigger, "servo_stop", self.stop, callback_group=self.service_handler_group)

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
        self.activate_cntr = 'forward_position_controller'
        self.deactivate_cntr = 'scaled_joint_trajectory_controller'

        self.start()
    

    def start(self):
        print("in start")
        if self.servo_active:
            print ("Servo is already active")
        else:
            switch_ctrl_req = SwitchController.Request(
                activate_controllers = [self.activate_cntr], deactivate_controllers= [self.deactivate_cntr], strictness = 2
            )
            print("trying to enable servo call ")
            self.enable_servo.call_async(Trigger.Request())
            #self.call_service_synced(self.enable_servo, Trigger.Request())
            print("enable servo call done")
            print("trying to switch controller")
            #self.call_service_synced(self.switch_ctrl, switch_ctrl_req)
            self.switch_ctrl.call_async(switch_ctrl_req)
            print("controller switched")
            self.active = True
            print("Servo has been activated and controller switched")
            
        return


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    switch = SwitchControllers()
    rclpy.spin(switch, executor)
    return

if __name__ == "__main__":
    main()
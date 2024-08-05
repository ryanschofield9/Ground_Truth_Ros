#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter


from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Int64, Float32


#from kalman import Kalman

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

import numpy as np

"""
TODO: would be nice to be able to dynamically configure the number and placement of ToF sensors

"""


class ToFNode(Node):
    RANGING_ERR = -1
    def __init__(self) -> None:
        super().__init__(node_name="tof_node")

        self.sub_tof1 = self.create_subscription(Float32, 'tof1', self.callback_tof1, 10)
        self.sub_tof2 = self.create_subscription(Float32, 'tof2', self.callback_tof2, 10)
        # Publishers
        self.publisher_ = self.create_publisher(Float32,'tof1_filter',10)
        self.publisher_2 = self.create_publisher(Float32, 'tof2_filter', 10)

        
        """ Set up the ToF Kalman filters """
        # ToF model parameters
        tof_model_param = self.declare_parameter(name="tof_model", value=Parameter.Type.STRING)
        tof_fov_x_param = self.declare_parameter(name="tof_fov_x", value=Parameter.Type.DOUBLE)
        tof_fov_y_param = self.declare_parameter(name="tof_fov_y", value=Parameter.Type.DOUBLE)
        range_z_min_param = self.declare_parameter(name="tof_range_z_min", value=Parameter.Type.DOUBLE)
        range_z_max_param = self.declare_parameter(name="tof_range_z_max", value=Parameter.Type.DOUBLE)
        self.tof_model = tof_model_param.get_parameter_value().string_value
        self.sensor_fov = np.array([
            tof_fov_x_param.get_parameter_value().double_value,
            tof_fov_y_param.get_parameter_value().double_value
        ])
        self.sensor_z_range = np.array([
            range_z_min_param.get_parameter_value().double_value,
            range_z_max_param.get_parameter_value().double_value,
        ])


        # Values
        self.tof_vals = 0
        self.tof_filtered_vals = 0
        self.tof_kalmans = np.empty(shape=2, dtype=KalmanFilter)
        self.tof_cov1 = np.array([1.602574**2]) # measured values
        self.initial_err = 1000.0
        self.measurement_err = 15.0
        kalman = KalmanFilter(dim_x=2, dim_z=1)
        kalman.x = np.array([[0, 0]]).transpose()
        kalman.F = np.array([[1,1],[0,1]])
        kalman.H = np.array([[1, 0]])
        kalman.P = np.identity(kalman.dim_x) * self.initial_err
        kalman.R = self.measurement_err
        kalman.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=self.tof_cov1)
        self.tof_kalmans= kalman

        self.tof_vals2 = 0
        self.tof_filtered_vals2= 0
        self.tof_kalmans2 = np.empty(shape=2, dtype=KalmanFilter)
        self.tof_cov2 = np.array([1.717898**2]) # measured values
        self.initial_err2 = 1000.0
        self.measurement_err2 = 15.0
        kalman2 = KalmanFilter(dim_x=2, dim_z=1)
        kalman2.x = np.array([[0, 0]]).transpose()
        kalman2.F = np.array([[1,1],[0,1]])
        kalman2.H = np.array([[1, 0]])
        kalman2.P = np.identity(kalman2.dim_x) * self.initial_err2
        kalman2.R = self.measurement_err
        kalman2.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=self.tof_cov2)
        self.tof_kalmans2= kalman2
        return
    
    def callback_tof1(self, msg: Float32) -> None:
        """Callback method for ToF raw data subscriber
        TODO: Fix with message type so sensor data is iterable"""
        # self.tof0 = msg.tof0
        # self.tof1 = msg.tof1
        self.tof_vals = msg.data
        # if msg.tof0 != self.RANGING_ERR:    
        #     self.tof_kalmans[0].predict()
        #     self.tof_kalmans[0].update([self.tof_vals[0]])
        #     self.tof_filtered_vals[0] = self.tof_kalmans[0].x[0,0] # get position from state matrix
        # if msg.tof1 != self.RANGING_ERR:
        #     self.tof_kalmans[1].predict()
        #     self.tof_kalmans[1].update([self.tof_vals[1]])
        #     self.tof_filtered_vals[1] = self.tof_kalmans[1].x[0,0]

        try:
            dist = self.tof_vals
            if dist != self.RANGING_ERR:
                self.tof_kalmans.predict()
                self.tof_kalmans.update([dist])
                self.tof_filtered_vals = self.tof_kalmans.x[0,0]
            msg_tof_filtered = Float32()
            msg_tof_filtered.data = self.tof_filtered_vals
            self.publisher_.publish(msg=msg_tof_filtered)
            self.get_logger().info(f"Recieved for tof1: raw: {dist} Sending: filtered: {msg_tof_filtered.data}")
        except IndexError as e:
            self.get_logger().error(f"{e}: Please check ToF setup and confirm that the number of sensors connected to the microROS agent aligns with the specified configuration.")
        return
    
    def callback_tof2(self, msg: Float32) -> None:
        """Callback method for ToF raw data subscriber
        TODO: Fix with message type so sensor data is iterable"""
        # self.tof0 = msg.tof0
        # self.tof1 = msg.tof1
        self.tof_vals2 = msg.data
        # if msg.tof0 != self.RANGING_ERR:    
        #     self.tof_kalmans[0].predict()
        #     self.tof_kalmans[0].update([self.tof_vals[0]])
        #     self.tof_filtered_vals[0] = self.tof_kalmans[0].x[0,0] # get position from state matrix
        # if msg.tof1 != self.RANGING_ERR:
        #     self.tof_kalmans[1].predict()
        #     self.tof_kalmans[1].update([self.tof_vals[1]])
        #     self.tof_filtered_vals[1] = self.tof_kalmans[1].x[0,0]

        try:
            dist = self.tof_vals2
            if dist != self.RANGING_ERR:
                self.tof_kalmans2.predict()
                self.tof_kalmans2.update([dist])
                self.tof_filtered_vals2 = self.tof_kalmans2.x[0,0]
            msg_tof_filtered = Float32()
            msg_tof_filtered.data = self.tof_filtered_vals2
            self.publisher_2.publish(msg=msg_tof_filtered)
            self.get_logger().info(f"Recieved for tof2: raw: {dist} Sending: filtered: {msg_tof_filtered.data}")
        except IndexError as e:
            self.get_logger().error(f"{e}: Please check ToF setup and confirm that the number of sensors connected to the microROS agent aligns with the specified configuration.")
        return
    

 
def main():
    rclpy.init()
    tof_node = ToFNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node=tof_node, executor=executor)
    tof_node.destroy_node()
    rclpy.shutdown()
    return


if __name__ == "__main__":
    main()
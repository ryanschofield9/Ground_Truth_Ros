#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from std_msgs.msg import Float32


#from kalman import Kalman

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

import numpy as np



class Filter(Node):
    RANGING_ERR = 0.0
    def __init__(self) -> None:
        super().__init__("filter")

        # Subscribers
        self.sub_tof1 = self.create_subscription(Float32, 'tof1', self.callback_tof1, 10)
        self.sub_tof2 = self.create_subscription(Float32, 'tof2', self.callback_tof2, 10)
        # Publishers
        self.publisher_ = self.create_publisher(Float32,'tof1_filter',10)
        self.publisher_2 = self.create_publisher(Float32, 'tof2_filter', 10)

        # Values and constants for Kalman Filter for TOF1 
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

        # Values and constants for Kalman Filter for TOF2
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
    
    def callback_tof1(self, msg) -> None:
        #Function that is called everytime the raw data for tof1 is published
        #Uses kalman filter to find new predicted data reading for tof1 
        
        self.tof_vals = msg.data

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
    
    def callback_tof2(self, msg) -> None:
        #Function that is called everytime the raw data for tof1 is published
        #Uses kalman filter to find new predicted data reading for tof1 
       
        self.tof_vals2 = msg.data

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
    tof_node = Filter()
    executor = MultiThreadedExecutor()
    rclpy.spin(node=tof_node, executor=executor)
    tof_node.destroy_node()
    rclpy.shutdown()
    return


if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial

import time 

from collections import deque

serialPort = serial.Serial(port = '/dev/ttyACM0', baudrate=115200,
                            bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

class ArduinoTOFPublisher(Node):
    
    def __init__(self): 
        super().__init__(node_name = 'tof_pub')
        self.i = 0
        self.avg = deque([0.0])
        self.avg2 = deque([0.0])
        self.publisher_ = self.create_publisher(Float32,'tof1',10)
        self.publisher_2 = self.create_publisher(Float32, 'tof2', 10)
        time.sleep(3)

        # Timer
        timer_period=0.01
        self.timer = self.create_timer(timer_period, self.publish_data)
        
    

    def publish_data(self):
        msg = Float32()
        msg_2 = Float32()
        try:
            line = serialPort.readline() 
            string = line.decode()
            newstring = string.split(';')
            try:
                tof1val = float(newstring[1])
                tof2val = float(newstring[3])
            except: 
                tof1val = 0
                tof2val = 0
                
        except ValueError as e:
            print(f"{e}: Could not convert msg type to float.")

        msg.data = tof1val
        msg_2.data = tof2val
        self.publisher_.publish(msg)
        self.publisher_2.publish(msg_2)
        try: 
            self.get_logger().info(f"TOF reading: {newstring[1]}, {newstring[3]}, Publishing: {msg.data}, {msg_2.data}")
        except: 
            self.get_logger().info("No Reading")

        
    


def main(args=None):
    rclpy.init(args=args)

    arduino_publisher = ArduinoTOFPublisher()

    rclpy.spin(arduino_publisher)

    rclpy.shutdown()

    


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import serial

import time 

from collections import deque

#create the serial connection 
serialPort = serial.Serial(port = '/dev/ttyACM0', baudrate=115200,
                            bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

class ArduinoTOFPublisher(Node):
    
    def __init__(self): 
        super().__init__(node_name = 'tof_pub')
        
        #Create publishers  
        self.publisher_ = self.create_publisher(Float32,'tof1',10)
        self.publisher_2 = self.create_publisher(Float32, 'tof2', 10)
        self.publisher_bool = self.create_publisher(Bool, 'touching_tree_flag', 10)
        
        #sleep to allow time for the serial connection to be made 
        #time.sleep(3)

        # Create Timer
        timer_period=0.01
        self.timer = self.create_timer(timer_period, self.publish_data)
        
    

    def publish_data(self):
        #Function that is called every 0.01 seconds to publish the TOF data from the Serial Port 
        msg = Float32()
        msg_2 = Float32()
        msg_bool = Bool()
        try:
            line = serialPort.readline() 
            string = line.decode()
            try: 
                string_first = string[0]
            except: 
                string_first = 'N'
            
            if string_first == 'T':
                #if the system has touched the tree TOUCH Will be displayed 
                msg_bool.data = True
                self.get_logger().info("TOUCH!!")
                self.publisher_bool.publish(msg_bool)
            else:
                newstring = string.split(';')
                try:
                    tof1val = float(newstring[1])
                    tof2val = float(newstring[3])
                    msg.data = tof1val
                    msg_2.data = tof2val
                    self.publisher_.publish(msg)
                    self.publisher_2.publish(msg_2)
                except: 
                    tof1val = 0
                    tof2val = 0
                try: 
                    self.get_logger().info(f"TOF reading: {newstring[1]}, {newstring[3]}, Publishing: {msg.data}, {msg_2.data}")
                except: 
                    self.get_logger().info("No Reading")
                
        except:
            print(f"{e}: Error, couldn't get message No Reading ")



        
    


def main(args=None):
    rclpy.init(args=args)

    arduino_publisher = ArduinoTOFPublisher()

    rclpy.spin(arduino_publisher)

    rclpy.shutdown()

    


if __name__ == '__main__':
    main()
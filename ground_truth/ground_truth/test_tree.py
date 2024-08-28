
from groun_truth_msgs.srv import CalcDiameter

import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import time 





# TO DO: CREATE THE MESSAGE PACKAGE AND ADD THE SERVICE INTO IT SEE GITHUB FROM ROB599 HW3 FOR EXPLANATION
class TouchTree(Node):

    def __init__(self):
        super().__init__('tree_touch')

        #Create clients 
        self.calc_diameter_client = self.create_client(CalcDiameter, 'calc_diameter')
        while not self.calc_diameter_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for calc_diameter service to start')
        
        #create timers 
        self.control_timer = self.create_timer(1/10, self.main_control)
        self.future = None
        self.done = False
        self.request1 = False 
        self.request2 = False 
        self.request3 = False 
        
        self.start_time = time.time()

        


    def main_control (self):
        #This function is called every 0.1 seconds and holds the main control structure for touching the tree 
        #The arm will move forward until the tree is touched 
        if self.done == False:
            if self.request1 == False:
                if not self.future: # A call is not pending
                    self.get_logger().info("Timer, calling delay 3s")
                    request = CalcDiameter.Request()
                    request.video_dis = 7.0
                    self.future = self.calc_diameter_client.call_async(request)

                if self.future.done(): # a call just completed
                    self.get_logger().info("Done")
                    print(self.future.result())
                    self.request1 = True 
                    self.future = None
                else:
                        self.get_logger().info("Doing other stuff!")
            elif self.request2 == False:
                if not self.future: # A call is not pending
                    self.get_logger().info("Timer, calling delay 3s")
                    request = CalcDiameter.Request()
                    request.video_dis = 7.5
                    self.future = self.calc_diameter_client.call_async(request)

                if self.future.done(): # a call just completed
                    self.get_logger().info("Done")
                    print(self.future.result())
                    self.request2 = True 
                    self.future = None
                else:
                        self.get_logger().info("Doing other stuff!")

            elif self.request3 == False:
                
                if not self.future: # A call is not pending
                    self.get_logger().info("Timer, calling delay 3s")
                    request = CalcDiameter.Request()
                    request.video_dis = 8.0
                    self.future = self.calc_diameter_client.call_async(request)

                if self.future.done(): # a call just completed
                    self.get_logger().info("Done")
                    print(self.future.result())
                    self.request3 = True
                    self.future = None
                else:
                        self.get_logger().info("Doing other stuff!")
                
            else: 
                 self.done = True 
                    

            




def main(args=None):
    rclpy.init(args=args)

    ang_check = TouchTree()

    rclpy.spin(ang_check)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
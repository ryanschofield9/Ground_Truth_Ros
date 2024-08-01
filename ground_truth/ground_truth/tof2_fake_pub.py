import rclpy 
from rclpy.node import Node 

from std_msgs.msg import Int64, Float32

class TOFData(Node): 
    def __init__(self):
        super().__init__('tof2')
        self.pub = self.create_publisher(Int64, 'tof2',10)
        self.timer=self.create_timer(0.1, self.publish_tof)
        self.count = 700
        self.count_down = True
    
    def publish_tof(self):
        msg = Int64()
        if self.count_down:
            if self.count > 180:
                msg.data = self.count 
                self.get_logger().info(f"Sending tof2 data: {msg.data} ")
                self.pub.publish(msg)
                self.count = self.count - 10
            else: 
                msg.data = self.count 
                self.get_logger().info(f"Sending tof2 data: {msg.data} ")
                self.pub.publish(msg)
                self.count_down = False
        else:
            if self.count < 800:
                msg.data = self.count 
                self.get_logger().info(f"Sending tof2 data: {msg.data} ")
                self.pub.publish(msg)
                self.count = self.count + 10
            
            else: 
                msg.data = self.count 
                self.get_logger().info(f"Sending tof2 data: {msg.data} ")
                self.pub.publish(msg)
                self.count_down = True

        
def main(args=None):
   rclpy.init(args=args)
   tof = TOFData()
   rclpy.spin(tof)
   rclpy.shutdown()

if __name__ == '__main__':
   main()
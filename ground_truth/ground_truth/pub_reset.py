import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class ResetPub(Node):
    
    def __init__(self): 
        super().__init__('reset_pub')
        
        #Create publishers  
        self.publisher_bool = self.create_publisher(Bool, 'reset', 10)
        

        self.publish_data()
        
    

    def publish_data(self):
        #Function that is called every 0.01 seconds to publish the TOF data from the Serial Port 
        msg = Bool()
        msg.data = True
        for x in range (0,3):
            self.publisher_bool.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    reset_pub = ResetPub()

    rclpy.spin(reset_pub)

    rclpy.shutdown()

    


if __name__ == '__main__':
    main()
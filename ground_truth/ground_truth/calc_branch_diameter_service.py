from groun_truth_msgs.srv import PixelWidth, CalcDiameter

import rclpy
from rclpy.node import Node

import math 



# TO DO: CREATE THE MESSAGE PACKAGE AND ADD THE SERVICE INTO IT SEE GITHUB FROM ROB599 HW3 FOR EXPLANATION
class CalcDiameterService(Node):

    def __init__(self):
        super().__init__('angle_check_service')

        #Create Service 
        self.service_pixel = self.create_service(PixelWidth, 'pixel_width', self.pixel_service)
        self.service_calc = self.create_service(CalcDiameter, 'calc_diameter', self.calc_diameter)

        #initialize variables
        self.diameter_pix = 157 #Change this to start with 0
        self.img_width_pix = 640 # NEED TO CHECK THIS VALUE 
        self.angle = 24 #in degrees 
        
    def pixel_service (self, request, response):
        self.diameter_pix = request.diameter_pix 
        response = PixelWidth.Response()
        response.saved = True
        return response 
    

    def calc_diameter(self, request,response):
        print(f"GOT REQUEST: {request}")
        dis_video = request.dis_video 
        angle_in_rad = math.radians(self.angle)
        img_w_inch = 2*dis_video*math.tan(angle_in_rad)
        pix_w_inch = img_w_inch / self.img_width_pix
        diameter_inch = self.diameter_pix * pix_w_inch
        print(f"The diamter in inches is {diameter_inch}")
        response = CalcDiameter.Response()
        response.diameter = diameter_inch
        print(f"RETURNING RESPONSE: {response}")
        return response 
    


def main(args=None):
    rclpy.init(args=args)
    calc_diameter = CalcDiameterService()
    rclpy.spin(calc_diameter)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
from groun_truth_msgs.srv import PixelWidth, CalcDiameter

import rclpy
from rclpy.node import Node

import math 



# TO DO: CREATE THE MESSAGE PACKAGE AND ADD THE SERVICE INTO IT SEE GITHUB FROM ROB599 HW3 FOR EXPLANATION
class AngleCheckClass(Node):

    def __init__(self):
        super().__init__('angle_check_service')

        #Create Service 
        self.service_pixel = self.create_service(PixelWidth, 'pixel_width', self.pixel_service)
        self.service_calc = self.create_service(CalcDiameter, 'calc_diameter', self.calc_diameter)

        #initialize variables
        self.pixel_width = 0 
        self.img_width_pix = 640 # NEED TO CHECK THIS VALUE 
        
    def pixel_service (self, request, response):
        self.diameter_pix = request.diameter_pix 
        response = PixelWidth.Response()
        response.saved = True
        return response 
    

    def calc_diameter(self, request,response):
        dis_video = request.dis_video 
        img_w_inch = 2*dis_video*math.tan(24)
        pix_w_inch = img_w_inch / self.img_width_pix
        diameter_inch = self.diameter_pix * pix_w_inch
        response = CalcDiameter.Response()
        response.diameter = diameter_inch
        return response 
    


def main(args=None):
    rclpy.init(args=args)

    ang_check = AngleCheckClass()

    rclpy.spin(ang_check)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
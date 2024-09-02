from groun_truth_msgs.srv import PixelWidth, CalcDiameter

import rclpy
from rclpy.node import Node

import math 



# TO DO: CREATE THE MESSAGE PACKAGE AND ADD THE SERVICE INTO IT SEE GITHUB FROM ROB599 HW3 FOR EXPLANATION
class CalcDiameterService(Node):

    def __init__(self):
        super().__init__('calc_diameter_service')

        #Create Service 
        self.service_pixel = self.create_service(PixelWidth, 'pixel_width', self.pixel_service) 
        self.service_calc = self.create_service(CalcDiameter, 'calc_diameter', self.calc_diameters)

        #initialize variables
        self.diameter_pix_W1 = 0.0 
        self.diameter_pix_W2 = 0.0 
        self.diameter_pix_mean = 0.0 
        self.diameter_pix_median = 0.0 
        self.img_width_pix = 1280 
        self.angle = 35 #in degrees 
        
    def pixel_service (self, request, response):
        #Service function that saves the width of a branch in pixels when a request sent 
        self.diameter_pix_W1 = request.diameter_pix_w1
        self.diameter_pix_W2 = request.diameter_pix_w2
        self.diameter_pix_mean = request.diameter_pix_mean 
        self.diameter_pix_median = request.diameter_pix_median
        response = PixelWidth.Response()
        response.saved = True
        return response 
    

    def calc_diameters(self, request,response):
        #Service function that calculates the diameter of a branch in inches when a request is sent 
        print(f"GOT REQUEST: {request}")
        video_dis = request.video_dis 
        angle_in_rad = math.radians(self.angle) #have to turn the angle into radians to get correct tan value
        img_w_inch = 2*video_dis*math.tan(angle_in_rad) 
        pix_w_inch = img_w_inch / self.img_width_pix
        diameter_inch_W1 = self.diameter_pix_W1 * pix_w_inch
        diameter_inch_W2 = self.diameter_pix_W2 * pix_w_inch
        diameter_inch_mean = self.diameter_pix_mean * pix_w_inch
        diameter_inch_median = self.diameter_pix_median * pix_w_inch
        print(f"The diamter in inches when using W1B1 is {diameter_inch_W1}")
        print(f"The diamter in inches when using W2B1 is {diameter_inch_W2}")
        print(f"The diamter in inches when using mean is {diameter_inch_mean}")
        print(f"The diamter in inches when using median is {diameter_inch_median}")
        response = CalcDiameter.Response()
        response.diameter_w1 = diameter_inch_W1
        response.diameter_w2= diameter_inch_W2
        response.diameter_mean= diameter_inch_mean
        response.diameter_median= diameter_inch_median
        print(f"RETURNING RESPONSE: {response}")
        return response 



def main(args=None):
    rclpy.init(args=args)
    calc_diameter = CalcDiameterService()
    rclpy.spin(calc_diameter)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
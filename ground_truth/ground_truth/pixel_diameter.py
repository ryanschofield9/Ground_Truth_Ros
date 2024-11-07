
from groun_truth_msgs.srv import CalcDiameter, CameraRecord, PixelWidth

import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TransformStamped, Point, TwistStamped, Vector3
from tf2_ros import TransformException

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from controller_manager_msgs.srv import SwitchController
import numpy as np

from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState
import time 
import matplotlib.pyplot as plt

import torch
from torchvision.io import read_video
import matplotlib.pyplot as plt
import torchvision.transforms.functional as F
import numpy as np
from torchvision.models.optical_flow import Raft_Small_Weights
from torchvision.models.optical_flow import raft_small
from torchvision.utils import flow_to_image
import copy 
import cv2
import statistics

plt.rcParams["savefig.bbox"] = "tight"
weights = Raft_Small_Weights.DEFAULT
transforms = weights.transforms()

'''
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
)
from rclpy.action import ActionClient
'''

import time 



# TO DO: CREATE THE MESSAGE PACKAGE AND ADD THE SERVICE INTO IT SEE GITHUB FROM ROB599 HW3 FOR EXPLANATION
class PixelDiameter(Node):

    def __init__(self):
        super().__init__('pixel_diameter')
        
        #Create clients 
        self.pixel_width_client= self.create_client(PixelWidth, 'pixel_width')
        while not self.pixel_width_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for pixel_width service to start')
        

        self.camera_record_client = self.create_client(CameraRecord, 'camera_record')
        while not self.camera_record_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for camera_record service to start')
        

        #Create tf buffer and listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Create publisher and subscribers 
        self.sub_flag = self.create_subscription(Bool, 'step3',self.calback_step3_flag, 10 )
        self.pub_step5 = self.create_publisher(Bool, 'step5_popup', 10)
        self.sub_reset = self.create_subscription(Bool, 'reset',self.calback_reset, 10 )
        self.pub_vel_commands = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        #create timers 
        self.control_timer = self.create_timer(1/10, self.main_control)
        self.tool_timer_pos_y = self.create_timer(1/50, self.pub_tool_pose_y)

        #Flags initialize 
        self.step_3 = False
        self.step_4 = False
        self.back_to_original = False
        self.first = True
        self.first_subplot = True  

        #constant variables 
        self.base_frame = 'base_link' #base frame that doesn't move 
        self.tool_frame = 'tool0' #frame that the end effector is attached to 
        
        #set future 
        self.future = None


        
    def main_control (self):
        #This function is called every 0.1 seconds and holds the main control structure for touching the tree 
        #The arm will move forward until the tree is touched 
        
        if self.step_3: 
            if not self.future: # A call is not pending
                request = CameraRecord.Request()
                self.file_name = '/home/ryan/ros2_ws_groundtruth/src/Ground_Truth_Ros/ground_truth/videos/testing.avi'
                request.file_name = self.file_name
                self.future = self.camera_record_client.call_async(request)
                self.starting_y = self.tool_y 

            if self.future.done(): # a call just completed
                if self.first: 
                    self.get_logger().info("Done")
                    print(self.future.result())
                    self.first = False
                elif self.back_to_original == False:
                    self.publish_twist([0.0, -0.1, 0.0], [0.0, 0.0, 0.0])
                    self.move_up_to_y(self.starting_y) 
                elif self.back_to_original: 
                        self.step_3 = False 
                        self.step_4 = True 
                        self.future = None
            
        elif self.step_4: 
            ## HAVE TO ADD HERE TO DO OPTICAL FLOW 
            video_path = self.file_name
            frames, _, _ = read_video(str(video_path), output_format="TCHW")
            frame = int(len(frames)/2)
            flow_imgs = self.optical_flow(frames, frame)
            closing, flow_saved = self.filter_imgs(flow_imgs)
            final_img = copy.deepcopy(closing)
            submatrix = closing[300:500, 0:1279]
            '''
            if self.first_subplot: 
                self.create_img_subplot(500, 100, '132', '132', final_img, closing, flow_imgs, frame, frames)
                self.first_subplott = False
                self.get_logger().info(f"Closing: {closing} ")
            '''

            all_starts, all_ends = self.pixel_count (submatrix)
            all_middle = self.middle_count(all_starts, all_ends)
            diameters = self.diameter_options(all_starts, all_ends)
            self.get_logger().info(f"Diameters: {diameters}")
            self.get_logger().info(f"All middles: {all_middle}")
            middle_lines = self.calculate_middle_line_options(all_middle)
            self.get_logger().info(f"Middle_lines: {middle_lines}")
            diameter_mean = float(statistics.mean(diameters))
            middle_mean = statistics.mean(middle_lines)
            diameter_median = float(statistics.median(diameters))
            middle_median = statistics.median(middle_lines)
            diameterW2, middleW2, scoreW2 = self.score_options_W2_B1(diameters, middle_lines, submatrix)
            diameterW1, middleW1, scoreW1 = self.score_options_W1_B1(diameters, middle_lines, submatrix)
            print(f"For Frame {frame}")
            print("diameter W2: ", diameterW2, "middle W2: ", middleW2, "scoreW2: ", scoreW2)
            print("diameter W1: ", diameterW1, "middle W1: ", middleW1, "scoreW1: ", scoreW1)
            print("diameter Mean: ", diameter_mean, "middle mean: ", middle_mean)
            print("diameter Median: ", diameter_median, "middle median: ", middle_median)

            if not self.future: # A call is not pending
                request = PixelWidth.Request()
                request.diameter_pix_w1  = diameterW1
                request.diameter_pix_w2  = diameterW2
                request.diameter_pix_mean  = diameter_mean
                request.diameter_pix_median  = diameter_median
                self.future = self.pixel_width_client.call_async(request)

            if self.future.done(): # a call just completed
                print("Done")
                print(self.future.result())
                self.step_4 = False
                msg = Bool()
                msg.data = True 
                self.create_box (diameterW2/2, middleW2, final_img)
                self.create_img_subplot(middleW2, diameterW2, '132', '132', final_img, closing, flow_imgs, frame, frames)
                self.pub_step5.publish(msg)
                self.reset()
            
    def calback_step3_flag(self,msg):
        self.step_3= msg.data
    
    def move_up_to_y(self, y_pose_want):
        #Using the given y pose that we want to go back to, move down until we reach that location 
        y_pose= self.tool_y #get the current tool y pose 
        #print("y_pose: ", y_pose, " y_pose_want", y_pose_want)
        if (y_pose_want - y_pose) < 0.001: 
            #when within 1mm of wanted y pose 
            self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]) #stop moving 
            self.back_to_original= True
        return 

    def pub_tool_pose_y(self):
        #save the current tool pose y 
        #there is a timer calling this function every 0.1 seconds 
        self.tool_y = self.get_tool_pose_y()


    def get_tool_pose_y(self, as_array=True):
            #Get the y position of the tool pose in respect to the base in m 
            try:
                #try to get the transform of from the base frame to the tool frame 
                current_time = rclpy.time.Time()
                tf = self.tf_buffer.lookup_transform(
                    self.tool_frame, self.base_frame, current_time
                )
                
            except TransformException as ex:
                #if the tranform returns a TransformException error print the error
                self.get_logger().warn("Received TF Exception: {}".format(ex))
                return
            pose = convert_tf_to_pose(tf) 
            if as_array:
                #if the pose is given as an array save the position and orientation as p and o respectively 
                p = pose.pose.position
                o = pose.pose.orientation
                return p.y #return just the tool y position with respect to teh base in m 
            else:
                return pose
            
    def calback_reset(self,msg): 
        if msg.data == True: 
            self.reset()

    def publish_twist(self, linear_speed, rot_speed):
        #publish a velocity command with the specified linear speed and rotation speed 
        my_twist_linear = linear_speed 
        my_twist_angular = rot_speed
        cmd = TwistStamped()
        cmd.header.frame_id = 'tool0'
        cmd.twist.angular = Vector3(x=my_twist_angular[0], y=my_twist_angular[1], z=my_twist_angular[2])
        cmd.twist.linear = Vector3(x=my_twist_linear[0], y=my_twist_linear[1], z=my_twist_linear[2])
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_vel_commands.publish(cmd)
        self.get_logger().info(f"Sending: linear: {cmd.twist.linear} angular: {cmd.twist.angular}")
        
    def reset(self):
        #Flags initialize 
        self.step_3 = False
        self.step_4 = False
        self.back_to_original = False
        self.first = True  
        self.first_subplot = True  
        
        #set future 
        self.future = None

    #Optical Flow Functions Below 
    def preprocess(self,img1_batch, img2_batch):
        #img1_batch = F.resize(img1_batch, size=[520, 960], antialias=False)
        #img2_batch = F.resize(img2_batch, size=[520, 960], antialias=False)
        return transforms(img1_batch, img2_batch)
    
    def check_occurances(self,list):
        avail_nums = []
        occurances = []
        paired_occurances = {}
        for num in list: 
            if num not in avail_nums: 
                avail_nums.append(num)
        for avail in avail_nums:
            occurances.append(list.count(avail))
        for (rad, occur) in zip(avail_nums, occurances):
            paired_occurances[rad]=occur
        #https://www.geeksforgeeks.org/sort-python-dictionary-by-value/
        sorted_paired_occurances = dict(sorted(paired_occurances.items(), 
                            key=lambda item: item[1]))
        return sorted_paired_occurances
    
    def create_list (self,dict): 
        #turn a dictionary with the values (key) and the count of the value (value) into a list with the correct count
        estimates_sorted = []
        for key in dict: 
            times = dict[key]
            for x in range (times):
                estimates_sorted.append(key)
        return estimates_sorted

    def create_list_single (self,dict):
        estimates = []
        for key in dict:
            estimates.append(key) 
        return estimates 

    def create_box (self,radius, middle, img):
        #draw a box that replicates where the tree is thought to be 
        #calculate left side of the line 
        left_side = int(middle - radius )

        #calculate right side of the line 
        right_side = int (middle + radius) 

        print(f"radius: {radius}, middle: {middle}, left side: {left_side}, right side: {right_side}")

        #draw left side line 
        img = cv2.line(img ,(left_side, 300),(left_side, 500),(155,0,0),5)

        #draw top line 
        img = cv2.line(img ,(left_side, 500),(right_side, 500),(155,0,0),5)

        #draw right side line 
        img = cv2.line(img ,(right_side, 300),(right_side, 500),(155,0,0),5)

        #draw bottom line 
        img = cv2.line(img ,(right_side, 300),(left_side, 300),(155,0,0),5)

        #draw middle line 
        img = cv2.line(img,(int(middle), 200),(int(middle),600),(155,0,0),5)
    
    def optical_flow (self,frames, frame):
        #https://pytorch.org/vision/main/auto_examples/others/plot_optical_flow.html#sphx-glr-auto-examples-others-plot-optical-flow-py
        #optical flow for the frames in the range above
        batch1 = []
        batch2 = []
        frames_using = frame
        frame_after =int( (len(frames)/4)*3)
        batch1.append(frames[frame])
        batch2.append(frames[frame_after])
        img1_batch= torch.stack(batch1)
        img2_batch= torch.stack(batch2)
        img1_batch, img2_batch = self.preprocess(img1_batch, img2_batch)
        print(f"shape = {img1_batch.shape}, dtype = {img1_batch.dtype}")

        device = "cuda" if torch.cuda.is_available() else "cpu"

        model = raft_small(weights=Raft_Small_Weights.DEFAULT, progress=False).to(device)
        model = model.eval()

        list_of_flows = model(img1_batch.to(device), img2_batch.to(device))
        predicted_flows = list_of_flows[-1]
        flow_imgs = flow_to_image(predicted_flows)

        # The images have been mapped into [-1, 1] but for plotting we want them in [0, 1]
        img1_batch = [(img1 + 1) / 2 for img1 in img1_batch]

        grid = [[img1, flow_img] for (img1, flow_img) in zip(img1_batch, flow_imgs)] 

        #optical flow image is saved as flowimgs 
        return flow_imgs
    
    def filter_imgs (self,flow_imgs):
        #https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html
            
        #turn the image into a numpy 
        if torch.cuda.is_available():
            img2 = flow_imgs[0].cpu().numpy()
        else: 
            img2 = flow_imgs[0].numpy()

        # make a copy of the numpy array image 
        flow_saved = copy.deepcopy(img2)
        # save just the numpy array 
        flow_saved = flow_saved[0]
        #move the axis of the image tp be able to normalize 
        img2 = np.moveaxis(img2, 0, -1)

        #normalize image
        img2 = (img2 - min(img2.reshape(-1)))/(max(img2.reshape(-1))-min(img2.reshape(-1)))*255
        img2 = img2.astype(np.uint8)
        print(img2.shape, min(img2.reshape(-1)), max(img2.reshape(-1)))

        #turn the image from RGB to Gray 
        bwimg = cv2.cvtColor(img2, cv2.COLOR_RGB2GRAY)

        #Use Gaussian Blur to blur the image to get a better threshold 
        blur = cv2.GaussianBlur(bwimg,(5,5),0)

        #Use OTSU thresholding 
        ret, img=cv2.threshold(blur,100,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        #filter the image by opening and closing the image (getting rid of any pixels that are by themselves and filling in any small holes)
        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        return closing, flow_saved

    def pixel_count (self,submatrix):
        #set pixel counting variables 
        consecutive = 0 # how many black pixels have I seen in a row 
        total_black = 0 # at the end of a row how many total black pixels have I seen (excluding when there are less than the threshold value)
        black_pixels = [] # a list holding all the black pixel groups over the threshold value that I have seen 
        start = [] # a list of the value that I see the first black pixel at for one row 
        end = [] # a list of the values that I see the last black pixel at for one row 
        all_starts = [] # a list off all the starts for all the rows 
        all_ends = [] # a list of all teh ends for all the rows 
        threshold_val = 40 # the value that there must be at least that many consecutive pixels for it to be seen as tree and not a mistake 

        # measure the amount of black in each row 
        for i,x in enumerate(submatrix):
            # going across the row 
            for j,y in enumerate (x): 
                #going down the image
                if y == 255:
                    #means it is white 
                    if consecutive >= threshold_val: 
                        #the pixels have been added to the list 
                        end.append(j-1)
                        
                    consecutive = 0 
                else: 
                    #means it is black
                    consecutive += 1
                    if consecutive == threshold_val: 
                        #if there has been 30 black pixels in a row at that to the black_pixel count 
                        total_black += threshold_val
                        start.append(j-39)
                    elif consecutive > threshold_val: 
                        total_black += 1 
                    if j == (len(x)-1):
                        end.append(j)
            black_pixels.append(total_black)
            if start != []:
                all_starts.append(start)
            if end != []:
                all_ends.append(end)
            total_black = 0 # reset total black pixel count for the next row 
            consecutive = 0 # reset consecutive count for the next row 
            start = [] # reset the list of start values for the next row  
            end = [] # reset the list of end values for the next row 
        return all_starts, all_ends
    
    def middle_count (self,all_starts, all_ends):
        all_middle = [] #initializes list of all the middle points in each row 
        for i in range (len(all_starts)):
            #calculate the middle based on the start and values in each row 
            mid = round((all_ends[i][-1] - all_starts[i][0])/2 +all_starts[i][0])
            all_middle.append(mid) 
        return all_middle
    
    def diameter_estimate (self, all_starts, all_ends): 
        # calculate the diameter of each row  
        diameter_estimates = [] # imitialize diameter estimate list 
        for idx in range (len (all_starts)):
            diameter = all_ends[idx][0] - all_starts[idx][0]
            diameter_estimates.append(diameter)

        #reorganize list so the list is organized by least seen number to most seen numbers 
        diameter_estimates_occurances = self.check_occurances(diameter_estimates)
        diameter_estimates_sorted = self.create_list(diameter_estimates_occurances)
        #change list so only the top 40% most seen values are in the list 
        per = 0.6
        length_from_end = int(len(diameter_estimates_sorted) * (per))
        diameter_estimates_new = diameter_estimates_sorted[-length_from_end:-1]
        #Average the top 40% vaues 
        diameter_new = np.average(diameter_estimates_new)
        #average of all the values to compare 
        diameter = np.average(diameter_estimates)

        print(f"diameter: {diameter}  diameter_new: {diameter_new}")
        return diameter_new
    
    def diameter_options (self,all_starts, all_ends):
        # calculate the diameter of each row  
        diameter_estimates = [] # imitialize diameter estimate list 
        for idx in range (len (all_starts)):
            diameter = all_ends[idx][0] - all_starts[idx][0]
            diameter_estimates.append(diameter)

        #reorganize list so the list is organized by least seen number to most seen numbers 
        diameter_estimates_occurances = self.check_occurances(diameter_estimates)
        diameter_estimates_list = self.create_list_single(diameter_estimates_occurances)

        per = 0.6
        length_from_end = int(len(diameter_estimates_list) * (per))
        diameters = diameter_estimates_list[-length_from_end:-1]
        return diameters


    def calculate_middle_line (self,all_middle, middle_line) :
        #reorganize list so the list is organized by least seen number to most seen numbers
        middle_estimates_occurances = self.check_occurances(all_middle)
        middle_estimates_sorted = self.create_list(middle_estimates_occurances)
        #change list so only the top 40% most seen values are in the list 
        per = 0.6
        length_from_end = int(len(middle_estimates_sorted) * (per))
        middle_estimates_new = middle_estimates_sorted[-length_from_end:-1]
        #Average the top 40% vaues
        middle_new = np.average(middle_estimates_new)
        #average of all the values to compare 
        middle = np.average(all_middle)

        print(f"middle: {middle}  middle_new: {middle_new}, mode: {middle_line}")
        return middle_new

    def calculate_middle_line_options (self,all_middle):
        #reorganize list so the list is organized by least seen number to most seen numbers
        middle_estimates_occurances = self.check_occurances(all_middle)
        middle_estimates_list = self.create_list_single(middle_estimates_occurances)

        middle_available = []
        for mid in middle_estimates_list: 
            if 340< mid < 940:
                middle_available.append(mid)
  
        #change list so only the top 40% most seen values are in the list 
        per = 0.6
        length_from_end = int(len(middle_available) * (per))
        middle_options = middle_available[-length_from_end:-1]

        return middle_options 
    
    def calculate_col_scores(self,submatrix):
        #add up the sum off the pixels in each column (essentially counting white pixels becuase either 255 or 0)
        col_sum = submatrix.sum(axis = 0) 
        #account for the fact that white pixels are 255 so the number of white pixels is seen as 1 instead of 255 
        col_sum = col_sum /255.0
        #print(col_sum)
        #print(len(col_sum))
        return col_sum

    def score_options_W2_B1(self,diameters, middles, submatrix):
        rows = submatrix.shape[0]
        columns = submatrix.shape[1]
        col_sum = self.calculate_col_scores(submatrix)
        #print(col_sum)
        cumulative_sum = np.cumsum(col_sum)
        #print(cumulative_sum)
        min_score = cumulative_sum[-1]
        #print(cumulative_sum)
        diameter_save = 0 
        middle_save = 0
        for diameter in diameters: 
            for middle in middles:  
                left_most = int(middle - diameter/2)
                right_most = int(middle + diameter/2)
                #print(f"diameter: {diameter}  middle: {middle}  rightmost: {right_most}  leftmost: {left_most}")
                if 0<left_most >= 640 or 0< right_most >= 640:
                    score =  cumulative_sum[-1]
                else: 
                    #number of white pixels in the square 
                    if left_most == 0: 
                        white_val_inbox = cumulative_sum[right_most]-0
                    else: 
                        white_val_inbox = cumulative_sum[right_most]-cumulative_sum[left_most-1]
                    #print(f"cumsum right:{cumulative_sum[right_most]}  cumsum left: {cumulative_sum[left_most-1]} ")
                    #number of black pixels outside the squre 
                    black_val_outbox = ((columns-(right_most -left_most +1)) * rows) - (cumulative_sum[-1] - white_val_inbox)
                    score = 2*white_val_inbox  + black_val_outbox
                if min_score > score: 
                    min_score = score 
                    diameter_save = diameter 
                    middle_save = middle
                #print(f"diameter: {diameter}  middle: {middle}  rightmost: {right_most}  leftmost: {left_most} score: {score}")
                #print(f" black outside: {black_val_outbox}   white inside: {white_val_inbox}")
        return diameter_save, middle_save, min_score
    
    def score_options_W1_B1(self,diameters, middles, submatrix):
        rows = submatrix.shape[0]
        columns = submatrix.shape[1]
        col_sum = self.calculate_col_scores(submatrix)
        #print(col_sum)
        cumulative_sum = np.cumsum(col_sum)
        #print(cumulative_sum)
        min_score = cumulative_sum[-1]
        #print(cumulative_sum)
        diameter_save = 0 
        middle_save = 0
        for diameter in diameters: 
            for middle in middles:  
                left_most = int(middle - diameter/2)
                right_most = int(middle + diameter/2)
                #print(f"diameter: {diameter}  middle: {middle}  rightmost: {right_most}  leftmost: {left_most}")
                if 0<left_most >= 640 or 0< right_most >= 640:
                    score =  cumulative_sum[-1]
                else: 
                    #number of white pixels in the square 
                    if left_most == 0: 
                        white_val_inbox = cumulative_sum[right_most]-0
                    else: 
                        white_val_inbox = cumulative_sum[right_most]-cumulative_sum[left_most-1]
                    #print(f"cumsum right:{cumulative_sum[right_most]}  cumsum left: {cumulative_sum[left_most-1]} ")
                    #number of black pixels outside the squre 
                    black_val_outbox = ((columns-(right_most -left_most +1)) * rows) - (cumulative_sum[-1] - white_val_inbox)
                    score = white_val_inbox  + black_val_outbox
                if min_score > score: 
                    min_score = score 
                    diameter_save = diameter 
                    middle_save = middle
                #print(f"diameter: {diameter}  middle: {middle}  rightmost: {right_most}  leftmost: {left_most} score: {score}")
                #print(f" black outside: {black_val_outbox}   white inside: {white_val_inbox}")
        return diameter_save, middle_save, min_score
    
    def create_img_subplot (self, middle_new, diameter_new, tof1, tof2, final_img, closing, flow_imgs, frame, frames):
        
        #determine if the middle line is in the middle of the image 
        threshold = 5 # number of pixels you can be right or left to be considered in the middle 
        rows = final_img.shape[0]
        columns = final_img.shape[1]
        mult = np.zeros((rows*2, columns*2))
        if middle_new < (columns/2 -threshold):
            in_front = False 
            direction = "left"
        elif middle_new > (columns/2 +threshold):
            in_front = False 
            direction = "right"
        else:
            in_front = True 
            direction = None
                

        pic1 = frames[frame]
        pic1 = F.to_pil_image(pic1.to("cpu"))
        pic2 = flow_imgs[0]
        pic2 = F.to_pil_image(pic2.to("cpu"))
        pic3 = closing
        pic4 = final_img
        pic5 = cv2.cvtColor(final_img, cv2.COLOR_GRAY2RGB)
            #https://www.geeksforgeeks.org/python-opencv-cv2-puttext-method/
            
            # add TOF reading to image with green if in center and red if not
        if in_front: 
            color = (0,255,0)
        else: 
            color = (255, 0,0)
        '''
        image = cv2.putText(pic5, tof1, (int(middle_new - diameter_new - 50) ,240), cv2.FONT_HERSHEY_SIMPLEX ,  
                    1, color, 3,  cv2.LINE_AA)
        image = cv2.putText(pic5, tof2, (int(middle_new+ diameter_new + 50 ),240), cv2.FONT_HERSHEY_SIMPLEX ,  
                    1, color, 3,  cv2.LINE_AA)
        #if not in center, image will say if the line is to the left or right 
        image = cv2.putText(pic5, direction, (300,400), cv2.FONT_HERSHEY_SIMPLEX ,  
                    1, color, 3,  cv2.LINE_AA)
        '''
        #create subplots and image 
        fig=plt.figure(frame)
        fig.add_subplot(2,2,1)
        plt.imshow(pic1)
        fig.add_subplot(2,2,2)
        plt.imshow(pic2)
        fig.add_subplot(2,2,3)
        plt.imshow(pic3, cmap='gray')
        fig.add_subplot(2,2,4)
        #show image 
        plt.imshow(pic5)
        plt.show()
    
        #filename = 'video_images/W2B1/full run/'+ 'frame' + str(frame) + 'scoring_white2_black1.png'
        #plt.savefig(filename)
    

def convert_tf_to_pose(tf: TransformStamped):
    #take the tf transform and turn that into a position  
    pose = PoseStamped() #create a pose which is of type Pose stamped
    pose.header = tf.header
    tl = tf.transform.translation
    pose.pose.position = Point(x=tl.x, y=tl.y, z=tl.z)
    pose.pose.orientation = tf.transform.rotation

    return pose

def main(args=None):
    rclpy.init(args=args)

    pix_diameter= PixelDiameter()

    rclpy.spin(pix_diameter)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
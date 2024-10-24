import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Int64, Bool, Float32 
from groun_truth_msgs.msg import Diameters

from geometry_msgs.msg import TwistStamped, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
import time 

from groun_truth_msgs.srv import Start

from PyQt6.QtWidgets import *

from PyQt6 import QtCore, QtGui

from PyQt6.QtGui import *

from PyQt6.QtCore import *

from PIL import Image as IMG
from PIL.ImageQt import ImageQt

import sys
from sensor_msgs.msg import JointState

import time
import threading
import numpy as np 
from sensor_msgs.msg import Image 
import cv2  
import csv
import datetime

from sensor_msgs.msg import JointState
import time 
import matplotlib.pyplot as plt

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
)
from rclpy.action import ActionClient
from controller_manager_msgs.srv import SwitchController

class Run(Node):
    def __init__(self, shared_data):
        super().__init__('GUI')
        #self.pub = self.create_publisher(Bool, 'reset', 10)
        self.pub_step1 = self.create_publisher(Bool, 'step_1', 10)
        self.sub_diameters = self.create_subscription(Diameters, 'diameters', self.callback_diameters, 10)
        self.timer = self.create_timer(1/10, self.check_vals)
        self.shared_data = shared_data
        self.sub_joints = self.create_subscription(JointState, 'joint_states',self.callback_joints, 10 )
        self.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joints= [0, 1, 2, 3, 4, 5]
        self.diameter= ['W1', 'W2', 'Mean', 'Median']

        self.start_time = time.time()
        self.diameter_time = time.time()

        self.pub_step5 = self.create_publisher(Bool, 'step5', 10)
        self.sub_popup = self.create_subscription(Bool, 'step5_popup',self.callback_popup, 10 )

        #Create Callback group
        self.service_handler_group = ReentrantCallbackGroup()

        self.switch_ctrl = self.create_client(
            SwitchController, "/controller_manager/switch_controller", callback_group=self.service_handler_group
        )


        self.moveit_planning_client = ActionClient(self, MoveGroup, "move_action")
        #app = QApplication(sys.argv)
        #self.main_window = Window()
        #app.exec()
        
    
    def check_vals (self):
        if self.shared_data.get_start()== True:
            self.pub_start()
        if self.shared_data.get_rotate()[0] == True: 
            print("rotate to ",self.shared_data.get_rotate()[1] )
            self.rotate_to_w(self.shared_data.get_rotate()[1])
            self.shared_data.set_rotate(False, self.shared_data.get_rotate()[1] )
        if self.shared_data.get_controller()[0] == True: 
            print("Chaning Controller")
            self.switch_controller(self.shared_data.get_controller()[1], self.shared_data.get_controller()[2])
            self.shared_data.set_controller(False, self.shared_data.get_controller()[1])
        if self.shared_data.get_pub_5(): 
            msg = Bool()
            msg.data = True
            self.pub_step5.publish(msg)
            self.pub_step5.publish(msg)
            self.pub_step5.publish(msg)
            self.shared_data.set_pub_5(False)
        self.update_joints()

    def update_joints(self):
        for i, x in enumerate(self.joints):
            self.joints[i]= x+1 
        self.shared_data.set_joints(self.joint_names, self.joints )
    
    def callback_joints(self,msg ):
        
        self.joint_names = msg.name
        self.joints= msg.position
        self.shared_data.set_joints(self.joint_names, self.joints )

    def callback_diameters(self, msg):
        self.diameter = [msg.diameter_w1, msg.diameter_w2, msg.diameter_mean, msg.diameter_median]
                         
        self.shared_data.set_diameters(self.diameter)
        self.diameter_time = time.time()

        self.total_time = self.diameter_time - self.start_time
        self.shared_data.set_total_time(self.total_time)

    def callback_popup(self,msg):
        self.popup=msg.data
        self.shared_data.set_popup(self.popup)

    def pub_start (self):
        self.start_time = time.time()
        msg = Bool()
        msg.data = True 
        self.pub_step1.publish(msg)
        self.shared_data.set_start(False)
        print("Publishing Start")
        
    def rotate_to_w(self, angle):
        #rotate the tool to the given angle  
        names = self.joint_names 
        pos = self.joints 

        #for all the joints, use the current angle for all joints, but wrist 3. Set wrist 3 to the given angle  
        for idx, vals in enumerate (names):
            if vals == "wrist_3_joint":
                pos[idx]= angle

        self.send_joint_pos(names, pos)  

    def send_joint_pos(self, joint_names, joints):
        #make a service call to moveit to go to given joint values 
        for n, p in zip (joint_names, joints):
             print(f"{n}: {p}")
        print(f"NOW SENDING GOAL TO MOVE GROUP")
        joint_constraints = [JointConstraint(joint_name=n, position=p) for n, p in zip(joint_names, joints)]
        kwargs = {"joint_constraints": joint_constraints}
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest(
            group_name="ur_manipulator",
            goal_constraints=[Constraints(**kwargs)],
            allowed_planning_time=5.0,
        ) #create a service request of given joint values and an allowed planning time of 5 seconds 

        goal_msg.planning_options = PlanningOptions(plan_only=False)

        self.moveit_planning_client.wait_for_server() #wait for service to be active 
        future = self.moveit_planning_client.send_goal_async(goal_msg) #make service call 
        future.add_done_callback(self.goal_complete) #set done callback

    def goal_complete(self, future):
            #function that is called once a service call is made to moveit_planning 
            rez = future.result()
            if not rez.accepted:
                print("Planning failed!")
                return
            else:
                print("Plan succeeded!")

    def switch_controller(self, act, deact):
        #activate the act controllers given and deactivate the deact controllers given 
        switch_ctrl_req = SwitchController.Request(
            activate_controllers = [act], deactivate_controllers= [deact], strictness = 2
            ) #create request for activating and deactivating controllers with a strictness level of STRICT 
        self.switch_ctrl.call_async(switch_ctrl_req) #make a service call to switch controllers 
        print(f"Activated: {act}  Deactivated: {deact}")
            
        return
    

class Popup_Exit (QMainWindow):

    def __init__(self,parent, tests):

        super().__init__(parent)

        # setting title

        self.setWindowTitle("Exit")

        # setting geometry

        self.setGeometry(100, 100, 400, 300)

        #save test values

        self.test = tests

        # calling method

        self.UiComponents()


        # show all the widgets

        self.show()

        # method for widgets

    def UiComponents(self):

        # create layout

        #layout for text at the top

        text_layout = QHBoxLayout()

        text_layout.setContentsMargins(150, 0, 150, 0)

        #layout for save and exit buttons

        button_layout = QHBoxLayout()

        button_layout.setSpacing(60)

        button_layout.setContentsMargins(100, 0, 100, 0)

        #layout of full page

        full_layout = QVBoxLayout()

        full_layout.addLayout(text_layout)

        full_layout.addLayout(button_layout)
 
        #Add text  

        self.label = QLabel("Are you sure you want to exit?", self)

        text_layout.addWidget(self.label)

        #Add Buttons

        self.button_yes = self.create_button("Yes", self.yes)

        self.button_no = self.create_button("No", self.no)
 
        button_layout.addWidget(self.button_yes)

        button_layout.addWidget(self.button_no)


        #Set Layout

        widget = QWidget()

        widget.setLayout(full_layout)

        self.setCentralWidget(widget)

 
    def create_button(self, name,func):

        button = QPushButton(name, self)

        button.clicked.connect(func)

        return button


    def yes(self):

        print("Trees saved")

        for vals in self.test:

            if vals[0] == 0:

                vals[0] = 1

            if vals[1] == 0:

                vals[1] = 1

            if vals[2] == 0:

                vals[2] = 1

            print(f"Tree: {vals[0]}  Branch: {vals[1]}  Trial: {vals[2]}")

        #sys.exit("Trying to exit")
        app.quit()


    def no(self):

        self.hide()

 

    def exec(self):

        self.show()

class Popup_Attach (QMainWindow):

    def __init__(self,parent, shared_data):

        super().__init__(parent)

        # setting title

        self.setWindowTitle("Attach Contact Pole")

        # setting geometry

        self.setGeometry(100, 100, 400, 300)

        #attach shared data
        self.shared_data = shared_data

        # calling method

        self.UiComponents()


        # show all the widgets

        self.show()

        # method for widgets

    def UiComponents(self):

        # create layout

        #layout for text at the top

        text_layout = QHBoxLayout()

        text_layout.setContentsMargins(150, 0, 150, 0)

        #layout for save and exit buttons

        button_layout = QHBoxLayout()

        button_layout.setSpacing(60)

        button_layout.setContentsMargins(100, 0, 100, 0)

        #layout of full page

        full_layout = QVBoxLayout()

        full_layout.addLayout(text_layout)

        full_layout.addLayout(button_layout)
 
        #Add text  

        self.label = QLabel("Have you attached the contact pole? ", self)

        text_layout.addWidget(self.label)

        #Add Buttons

        self.button_yes = self.create_button("Yes", self.yes)

        self.button_no = self.create_button("No", self.no)
 
        button_layout.addWidget(self.button_yes)

        button_layout.addWidget(self.button_no)


        #Set Layout

        widget = QWidget()

        widget.setLayout(full_layout)

        self.setCentralWidget(widget)

 
    def create_button(self, name,func):

        button = QPushButton(name, self)

        button.clicked.connect(func)

        return button


    def yes(self):

        self.pub_step6 = True
        self.shared_data.set_pub_5(True)
        self.close()


    def no(self):

        print("PLEASE Attach Contact Pole")

 

    def exec(self):

        self.show()
    
class Window(QMainWindow):

    def __init__(self, shared_data):

        super().__init__()

        self.setWindowTitle("Ground Truth System")

        # setting geometry

        self.setGeometry(100, 100, 800, 400)

        #setting variables

        self.tree_val = 0

        self.branch_val = 0

        self.trial_val = 0

        self.tests = []

        self.trees_saved = []

        self.shared_data = shared_data

        self.timer = QtCore.QBasicTimer()

        self.at_0 = True 
        self.controller = "Joints"

        self.hit_pos = "N/a"
        self.distance_tree= "N/a"
        self.total_time = 0.0

        #QtGui.QImageReader.supportedImageFormats()

        # showing all the widgets

        self.UiComponents()

        
        #self.vid  = cv2.VideoCapture(0)

        #adding timer
        self.timer.start(100, self) 

        self.show()

        #self.refresh()

 


    def UiComponents(self):


        # create layout
        
        #layout for saving file 
        self.file_layout = QHBoxLayout()
        self.file_layout.setSpacing(30)
        self.file_layout.setContentsMargins(20, 0, 20, 0)

        #layout for naming test (Tree, Branch, Trial)
        self.test_layout = QHBoxLayout()

        self.test_layout.setSpacing(30)

        self.test_layout.setContentsMargins(20, 0, 20, 0)

        #layout for joint Names 
        self.joint_layout = QVBoxLayout()
        
        self.joint_layout.setSpacing(30)

        self.joint_layout.setContentsMargins(20,0,20, 0 )

        #layput for diameter 

        self.diameter_layout = QHBoxLayout()

        self.diameter_layout.setSpacing(30)

        self.diameter_layout.setContentsMargins(20, 0, 20, 0)

        #layout for hit position 
        self.hit_layout = QHBoxLayout()

        self.hit_layout.setSpacing(30)

        self.hit_layout.setContentsMargins(20, 0, 20, 0)


        #layout for rotate and change controller button 

        self.rot_change_layout = QHBoxLayout()
        self.rot_change_layout.setSpacing(30)
        self.rot_change_layout.setContentsMargins(20, 0, 20, 0)

        #layout for diameter and rot_change 

        self.dia_rot_layout = QVBoxLayout()
        self.dia_rot_layout.addLayout(self.diameter_layout)
        self.dia_rot_layout.addLayout(self.hit_layout)
        self.dia_rot_layout.addLayout(self.rot_change_layout)


        #set middle layout 

        self.middle_layout = QHBoxLayout()
        self.middle_layout.addLayout(self.joint_layout)
        self.middle_layout.addLayout(self.dia_rot_layout)

        # TO DO ADD FUNCTIONS TO START AND SWITCH TO scaled_joint_trajectory_controller'

        #layout for save and exit buttons

        self.button_layout = QHBoxLayout()

        self.button_layout.setSpacing(60)

        self.button_layout.setContentsMargins(100, 0, 100, 0)


        #layout of full page

        self.full_layout = QVBoxLayout()

        self.full_layout.addLayout(self.file_layout)
        self.full_layout.addLayout(self.test_layout)
        self.full_layout.addLayout(self.middle_layout)
        self.full_layout.addLayout(self.button_layout)

 
        self.file_name = QLineEdit()
        self.file_name.setMaxLength(30)
        self.file_name.setPlaceholderText("Enter your filename (no file type)")
        self.file_name.textChanged.connect(self.filename_changed)
        self.file_layout.addWidget(self.file_name)

        #create test counting boxes

        self.trial = self.create_QSpinBox("Trial", self.action_trial)

        self.trial.setRange(1,20)

        self.branch = self.create_QSpinBox("Branch", self.action_branch)

        self.branch.setRange(1,20)

        self.tree = self.create_QSpinBox("Tree", self.action_tree)

        self.tree.setRange(1,20)

        self.test_layout.addWidget(self.tree)

        self.test_layout.addWidget(self.branch)

        self.test_layout.addWidget(self.trial)

       #adding Joint names to 

        self.joint_names, self.joints = self.shared_data.get_joints()  

        self.label_1 = QLabel (f"{self.joint_names[0]}: {self.joints[0]}")  

        self.joint_layout.addWidget(self.label_1)

        self.label_2 = QLabel (f"{self.joint_names[1]}: {self.joints[1]}")  

        self.joint_layout.addWidget(self.label_2)

        self.label_3 = QLabel (f"{self.joint_names[2]}: {self.joints[2]}")  

        self.joint_layout.addWidget(self.label_3)

        self.label_4 = QLabel (f"{self.joint_names[3]}: {self.joints[3]}")  

        self.joint_layout.addWidget(self.label_4)

        self.label_5= QLabel (f"{self.joint_names[4]}: {self.joints[4]}")  

        self.joint_layout.addWidget(self.label_5)

        self.label_6= QLabel (f"{self.joint_names[5]}: {self.joints[5]}")  

        self.joint_layout.addWidget(self.label_6)


        #add diameter 

        self.label_diameter_found_w1 = QLabel ("W1: None")
        self.label_diameter_found_w2 = QLabel ("W2: None")
        self.label_diameter_found_mean = QLabel ("Mean: None")
        self.label_diameter_found_median = QLabel ("Median: None")
        self.label_diameter_real = QLineEdit()
        self.label_diameter_real.setMaxLength(30)
        self.label_diameter_real.setPlaceholderText("Please enter the calipered measured diameter")
        self.label_diameter_real.textChanged.connect(self.real_diameter_changed)
        self.diameter_layout.addWidget(self.label_diameter_found_w1)
        self.diameter_layout.addWidget(self.label_diameter_found_w2)
        self.diameter_layout.addWidget(self.label_diameter_found_mean)
        self.diameter_layout.addWidget(self.label_diameter_found_median)
        self.diameter_layout.addWidget(self.label_diameter_real)


        #add hit position text 
        self.hit_text = QLineEdit()
        self.hit_text.setMaxLength(15)
        self.hit_text.setPlaceholderText("Enter Contact Postion")
        self.hit_text.textChanged.connect(self.hit_position_func)
        self.hit_layout.addWidget(self.hit_text)

        self.dis_text = QLineEdit()
        self.dis_text.setMaxLength(10) 
        self.dis_text.setPlaceholderText("Distance From Tree")
        self.dis_text.textChanged.connect(self.distance_func)
        self.hit_layout.addWidget(self.dis_text)

        self.time_text = QLabel (f"Time: {round(self.total_time,2)} ")
        #self.time_text.setMaxLength(10)
        #self.time_text.setPlaceholderText("Time to measure (s)")
        #self.time_text.textChanged.connect(self.time_func)
        self.hit_layout.addWidget(self.time_text)

        
        # add rotate and change controller buttons 

        self.button_rotate = self.create_button("Rotate", 400, 300, 120, 60, self.rotate)
        self.button_change = self.create_button("Change Controller", 400, 300, 120, 60, self.change)
        self.button_start = self.create_button("Start", 200, 300, 120, 60, self.start)
        self.rot_change_layout.addWidget(self.button_rotate)
        self.rot_change_layout.addWidget(self.button_change)
        self.rot_change_layout.addWidget(self.button_start)

        #create save, reset, and exit buttons

        self.button_save = self.create_button("Save", 400, 300, 120, 60, self.save)

        self.button_exit = self.create_button("Exit", 200, 300, 120, 60, self.exit)
        
        self.button_start = self.create_button("Start", 200, 300, 120, 60, self.start)

 

        self.button_layout.addWidget(self.button_save)

        self.button_layout.addWidget(self.button_exit)


 

        #Set Layout

        widget = QWidget()

        widget.setLayout(self.full_layout)

        self.setCentralWidget(widget)

 

    def filename_changed (self,file):
        self.file = file 

    def real_diameter_changed (self,diameter):
        self.real_diameter = diameter 
        
    def create_QSpinBox (self,name, func):

        box= QSpinBox(self)

        box.setPrefix(name+": ")

        box.valueChanged.connect(func)

        return box

 

    def create_button(self, name, x, y, w, l, func):

        button = QPushButton(name, self)

        button.clicked.connect(func)

        return button

 

    def action_tree(self):

        # method called after editing finished

        # getting current value of spin box

        self.tree_val = self.tree.value()

        self.branch.setValue(1)

        self.trial.setValue(1)

         

    def action_branch(self):

        # method called after editing finished

        # getting current value of spin box

        self.branch_val = self.branch.value()

        self.trial.setValue(1)

   

    def action_trial(self):

        # getting current value of spin box

        self.trial_val = self.trial.value()

    def save(self):
        self.joints_in_order = []
        if self.tree_val== 0:
            self.tree_val = 1
        if self.branch_val == 0:
            self.branch_val = 1
        if self.trial_val == 0:
            self.trial_val = 1
        for val in self.trees_saved: 
            print(self.trees_saved)
            if val == [self.tree_val, self.branch_val, self.trial_val]:
                print("Already Saved Not adding anything to the saved values")
                #popup = Popup_Label(self,"This trial will not be ", "" )
                return 
        # add the joints in order 
        for idx, joint in enumerate(self.joint_names):
            if joint == 'elbow_joint':
                self.joints_in_order.append(self.joints[idx])
        for idx, joint in enumerate(self.joint_names):
            if joint == 'shoulder_lift_joint':
                self.joints_in_order.append(self.joints[idx])
        for idx, joint in enumerate(self.joint_names):
            if joint == 'shoulder_pan_joint':
                self.joints_in_order.append(self.joints[idx])
        for idx, joint in enumerate(self.joint_names):
            if joint == 'wrist_1_joint':
                self.joints_in_order.append(self.joints[idx])
        for idx, joint in enumerate(self.joint_names):
            if joint == 'wrist_2_joint':
                self.joints_in_order.append(self.joints[idx])
        for idx, joint in enumerate(self.joint_names):
            if joint == 'wrist_3_joint':
                self.joints_in_order.append(self.joints[idx])
        

        self.trees_saved.append([self.tree_val, self.branch_val, self.trial_val])
        self.tests.append([self.tree_val, self.branch_val, self.trial_val, self.joints_in_order[0],self.joints_in_order[1],self.joints_in_order[2],self.joints_in_order[3], self.joints_in_order[4], self.joints_in_order[5],self.real_diameter, self.diameters[0], self.diameters[1], self.diameters[2], self.diameters[3], self.hit_pos, self.distance_tree, self.total_time])
       
    
    def exit(self):
        time = datetime.datetime.now()
        filename = 'src/GroundTruth/ground_truth/csv_files/tes.csv'+self.file + str(time)+ '.csv'
        fields = ['Tree', 'Branch', 'Trial', 'elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint', 'measured diameter', "W1 Diameter", "W2 Diameter", "Mean Diameter", "Median Diameter", "Contact", "Distance from Tree", "Time (s)"]
        with open(filename, 'w', newline='') as file:
            csvwriter = csv.writer(file)   
            # writing the fields   
            csvwriter.writerow(fields)   
            # writing the data rows   
            csvwriter.writerows(self.tests) 
    
        popup = Popup_Exit(self, self.tests)
        popup.exec()

    def reset(self):
        self.shared_data.set_reset(True)

    def start(self):
        self.shared_data.set_start(True)

    def hit_position_func(self, pos):
        self.hit_pos= pos

    def distance_func(self, dis):
        self.distance_tree = dis
    
    
    def joint_update(self, joint_layout):
        self.joint_names, self.joints = self.shared_data.get_joints()  

        self.label_1.setText(f"{self.joint_names[0]}: {round(self.joints[0],2)}")  


        self.label_2.setText(f"{self.joint_names[1]}: {round(self.joints[1],2)}")  


        self.label_3.setText(f"{self.joint_names[2]}: {round(self.joints[2],2)}")  


        self.label_4.setText(f"{self.joint_names[3]}: {round(self.joints[3],2)}")  


        self.label_5.setText(f"{self.joint_names[4]}: {round(self.joints[4],2)}")  


        self.label_6.setText(f"{self.joint_names[5]}: {round(self.joints[5],2)}")  

        return joint_layout 

    def diameters_update(self):
        self.diameters = self.shared_data.get_diameters()
        self.label_diameter_found_w1.setText(f"W1: {round(self.diameters[0],2)}")
        self.label_diameter_found_w2.setText(f"W2: {round(self.diameters[1],2)}")
        self.label_diameter_found_mean.setText(f"Mean: {round(self.diameters[2],2)}")
        self.label_diameter_found_median.setText(f"Median: {round(self.diameters[3],2)}")

    def rotate(self):
        if self.at_0 == True: 
            self.shared_data.set_rotate(True, 0.0)
            self.at_0 = False
        else:
            self.shared_data.set_rotate(True, 1.5708) #radians is used, so 1.5708 rads is 90 degrees 
            self.at_0 = True

    def change(self):
        if self.controller == "Joints": 
            self.shared_data.set_controller(True, "Forward")
            self.controller = "Forward"
        else:
            self.shared_data.set_controller(True, "Joints")
            self.controller = "Joints"

    def attach_popup(self):
        if self.shared_data.get_popup():
            popup = Popup_Attach(self, self.shared_data)
            popup.exec()
            self.shared_data.set_popup(False)

    def total_time_update(self):
        self.total_time = round(self.shared_data.get_total_time(),2)
        self.time_text.setText(f"Time: {self.total_time} ")

    def timerEvent(self, e):
        self.joint_update(self.joint_layout)
        self.diameters_update()
        self.attach_popup()
        self.total_time_update()

        

class SharedData:
    def __init__(self):
        self.reset = False 
        self.start = False
        self.rotate = False 
        self.angle = 0 
        self.joint_names = [None, None, None, None, None, None]
        self.joints= [0,0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.frame = None
        self.diameter = [0.0, 0.0, 0.0, 0.0]
        self.forward_cntr = 'forward_position_controller' #name of controller that uses velocity commands 
        self.joint_cntr = 'scaled_joint_trajectory_controller' # name of controller that uses joint commands 
        self.switch = False 
        self.controller = 'Forward'
        self.popup = False 
        self.pub_5 = False
        self.total_time = 0 
    
    def get_reset(self):
        return self.reset
    
    def set_reset (self, val):
        self.reset = val 

    def get_start(self):
        return self.start
    
    def set_start (self, val):
        self.start = val 
    
    def set_joints(self, joint_names, joints ):
        self.joint_names = joint_names 
        self.joints = joints
    
    def get_joints(self):
        return (self.joint_names, self.joints)
    
    def set_frame(self, frame):
        self.frame = frame
        #print(frame)
    
    def get_frame(self):
        return self.frame 
    
    def set_diameters(self, diameter):
        #print(diameter)
        self.diameter = diameter

    def get_diameters(self):
        return self.diameter
    
    def set_rotate(self, rotate, angle):
        self.rotate = rotate
        self.angle = angle 

    def get_rotate(self):
        return [self.rotate, self.angle]
    
    def set_controller(self, switch, controller):
        self.switch = switch 
        self.controller = controller 

    def get_controller(self):
        if self.controller == "Forward":
            return [self.switch, self.forward_cntr, self.joint_cntr]
        else:
            return [self.switch, self.joint_cntr, self.forward_cntr]
        
    def set_popup(self, popup):
        self.popup = popup
    
    def get_popup(self):
        return self.popup
    
    def set_pub_5(self, pub_5):
        self.pub_5= pub_5
    
    def get_pub_5(self):
        return self.pub_5
    
    def set_total_time(self, time):
        self.total_time = time

    def get_total_time(self):
        return self.total_time
    

class ROS2NodeWrapper(threading.Thread):
    def __init__(self, node_class, *args, **kwargs):
        super().__init__()
        self.node_class = node_class
        self.args = args
        self.kwargs = kwargs

    def run(self):
        rclpy.init()
        node = self.node_class(*self.args, **self.kwargs)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

app = QApplication(sys.argv) 

def main(args=None):
    '''
    rclpy.init(args=args)
    run = Run()
    rclpy.spin(run)
    

    app = QApplication(sys.argv)
    window = Window()
    app.exec()
    '''

    shared_data = SharedData()
    
    main_window = Window(shared_data)
    main_window.show()
    

   
    node_thread = ROS2NodeWrapper(Run, shared_data)
    node_thread.start()

    sys.exit(app.exec())


    
    
    


if __name__ == '__main__':
   main()


import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Int64, Bool
from geometry_msgs.msg import TwistStamped, Vector3
import time 

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

class Run(Node):
    def __init__(self, shared_data):
        super().__init__('GUI')
        self.pub = self.create_publisher(Bool, 'reset', 10)
        self.timer = self.create_timer(1/10, self.reset)
        self.shared_data = shared_data
        self.sub_joints = self.create_subscription(JointState, 'joint_states',self.callback_joints, 10 )
        self.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joints= [0, 1, 2, 3, 4, 5]
        self.sub_tof1 = self.create_subscription(Image, 'camera_image', self.callback_image, 10)
        #app = QApplication(sys.argv)
        #self.main_window = Window()
        #app.exec()
        
    
    def reset(self):
        if self.shared_data.get_reset()== True:
            msg = Bool()
            msg.data = True
            self.pub.publish(msg)
            self.shared_data.set_reset(False)
        for i, x in enumerate(self.joints):
            self.joints[i]= x+1 
        self.shared_data.set_joints(self.joint_names, self.joints )

    def callback_joints(self,msg ):
        
        self.joint_names = msg.name
        self.joints= msg.position
        self.shared_data.set_joints(self.joint_names, self.joints )

    def callback_image (self, msg):
        frame = msg
        #print ("in callback", frame)
        self.shared_data.set_frame(frame)

        
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

        self.shared_data = shared_data

        self.timer = QtCore.QBasicTimer()

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

        #set middle layout 

        self.middle_layout = QHBoxLayout()
        self.middle_layout.addLayout(self.joint_layout)
        self.middle_layout.addLayout(self.diameter_layout)

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

        self.label_diameter_found = QLabel ("TEST")
        self.label_diameter_real = QLineEdit()
        self.label_diameter_real.setMaxLength(30)
        self.label_diameter_real.setPlaceholderText("Please enter the calipered measured diameter")
        self.label_diameter_real.textChanged.connect(self.real_diameter_changed)
        self.diameter_layout.addWidget(self.label_diameter_found)
        self.diameter_layout.addWidget(self.label_diameter_real)


        #create save, reset, and exit buttons

        self.button_save = self.create_button("Save", 400, 300, 120, 60, self.save)

        self.button_exit = self.create_button("Exit", 200, 300, 120, 60, self.exit)
        
        self.button_reset = self.create_button("Reset", 200, 300, 120, 60, self.reset)

 

        self.button_layout.addWidget(self.button_save)

        self.button_layout.addWidget(self.button_exit)

        self.button_layout.addWidget(self.button_reset)

 

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
        for val in self.tests: 
            if val == [self.tree_val, self.branch_val, self.trial_val]:
                print("Already Saved ")
                popup = Popup_Label(self,"This trial will not be ", "" )
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

        self.tests.append([self.tree_val, self.branch_val, self.trial_val, self.joints_in_order[0],self.joints_in_order[1],self.joints_in_order[2], self.joints_in_order[3], self.joints_in_order[4], self.joints_in_order[5],self.real_diameter])
       
    
    def exit(self):
        filename = 'src/Ground_Truth_Ros/ground_truth/csv_files/'+self.file + '.csv'
        fields = ['Tree', 'Branch', 'Trial', 'elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'measured diameter']
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

    def joint_update(self, joint_layout):
        self.joint_names, self.joints = self.shared_data.get_joints()  

        self.label_1.setText(f"{self.joint_names[0]}: {self.joints[0]}")  


        self.label_2.setText(f"{self.joint_names[1]}: {self.joints[1]}")  


        self.label_3.setText(f"{self.joint_names[2]}: {self.joints[2]}")  


        self.label_4.setText(f"{self.joint_names[3]}: {self.joints[3]}")  


        self.label_5.setText(f"{self.joint_names[4]}: {self.joints[4]}")  


        self.label_6.setText(f"{self.joint_names[5]}: {self.joints[5]}")  

        return joint_layout 

        
    def timerEvent(self, e):
        self.joint_update(self.joint_layout)

        

class SharedData:
    def __init__(self):
        self.reset = False 
        self.joint_names = [None, None, None, None, None, None]
        self.joints= [None, None, None, None, None, None]
        self.frame = None
    
    def get_reset(self):
        return self.reset
    
    def set_reset (self, val):
        self.reset = val 
    
    def set_joints(self, joint_names, joints ):
        self.joint_names = joint_names 
        self.joints = joints
    
    def get_joints(self):
        return (self.joint_names, self.joints)
    
    def set_frame(self, frame):
        self.frame = frame
        #print(frame)
    
    def get_frame(self):
        print("in get frame")
        return self.frame 
    


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


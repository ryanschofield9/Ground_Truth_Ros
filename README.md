# Ground Truth Measurement System 
Measure the diameter of a branch that the UR5 arm is placed in front of. ROS2 humble is used. 

# How to run 

1. Plug arduino into computer 
  
2. Use the following commanda to allow ROS2 to see what the arduino is publishing 
```
sudo chmod 777 /dev/ttyACM0
```
3. Start connection to the UR5 
Run the following command if you are running a simulation 
```
ros2 launch ur_robot_driver ur5e.launch.py robot_ip:=10.10.10.10 use_fake_hardware:=true
```
Run the following command if you are running on a real arm 
```
ros2 launch ur_robot_driver ur5e.launch.py robot_ip:=169. 254.174.50 use_fake_hardware:=false
```
4. Start Moveit 
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e
```  

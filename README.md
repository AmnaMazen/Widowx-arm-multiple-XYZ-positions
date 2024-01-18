# Widowx-arm-multiple-XYZ-positions

Author name: Amna Mazen Ali & Ziyuan Zhao


This repository provides a step-by-step guide on picking and placing multiple items given their XYZ positions using the Widowx arm.

I uploaded a video about a step-by-step tutorial on YouTube:

We will use the package "move_group_python_interface" that we created before in this repository "https://github.com/AmnaMazen/Widowx-arm-Move-Group-Python-Interface".

Please download the attached code in this repository.

It contains two Python files, "GetPose.py" and "ThreeBox_pose.py," and you need to save them inside the source folder "src" of the "move_group_python_interface" package. 

* "GetPose.py" is a Python file used to get the XYZ position and RPY orientation values for the specified pose.

* "ThreeBox_pose.py" is a Python file to pick three items from three predefined poses and place them in a specific position.

After saving the Python files, you will need to make them as executables and source the "widowx_arm" workspace using the following commands:

$ sudo chmod +x GetPose.py

$ sudo chmod +x ThreeBox_pose.py

$ cd ~/widowx_arm

$ catkin_make

## Let's launch the arm and run the code:

$ source ./devel/setup.bash

$ roslaunch widowx_arm_bringup arm_moveit.launch sim:=false sr300:=false


In another terminal:

$ cd ~/widowx_arm

$ source ./devel/setup.bash


$ rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link base_footprint 100

$ rosrun move_group_python_interface ThreeBox_pose.py


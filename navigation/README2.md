#setup apriltag follow

#follow directions to download pip for python 3, I think other python versions work (just be consistent)
sudo apt install python3-pip
#download apriltag for python 3 with pip
python3 -m pip install apriltag
#python wrapper for realsense, if on laptop can pip install, pi must build from sorce
https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python#installation

#make workspace
cd catkin_ws
catkin_make

#Run Code
 *laptop $ roscore

 *pi3 (optional) $ roslaunch turtlebot3_bringup turtlebot3_robot.launch

#Apriltag follow with Turtlebot, Camera and Depth
 *pi4 or laptop $ python filename.py


#--------------------------------------------------------------------------------
#setup xy_pid
see file



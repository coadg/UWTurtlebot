#5/3 PID x and Y 
'''
#General
-Navigate to x and y point using PID. adjust. Change total_distance_x and y variables to change goal
-Integral control not functioning (i cancel it in eq)
-adusting path angle based on 2 pi not functioning, not sure why i need it yet but example did

Install:
Should have everything already

#Running
-Roscore
-bringup on pi3
-Run code on pi3 (or laptop, since everything is through ROS)
-make sure to close bringup after each trial. this will reset the odometer.

#code based on: https://www.youtube.com/watch?v=znWkFWk76XQ
#move5.py on my computer
'''

import rospy
import time
import math

import tf
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry #odometry
from geometry_msgs.msg import Twist  #moving bot
from math import radians, copysign, sqrt, pow, pi, atan2
#count loops to only set goal once
count=0
#
previous_distance=0
previous_angle=0
last_rotation=0

def callback(msg):
        #distance PID gain
        kp_distance = 1
        ki_distance =.01
        kd_distance = .5
        #angle PID Gain
        kp_angle=1
        ki_angle=.03
        kd_angle=.05

        #current position
        posx=msg.pose.pose.position.x
        posy=msg.pose.pose.position.y
        posz=msg.pose.pose.position.z

        #current rotation
        x=msg.pose.pose.orientation.x
        y=msg.pose.pose.orientation.y
        z=msg.pose.pose.orientation.z
        w=msg.pose.pose.orientation.w
        rot=(x,y,z,w)
        rotation_array=euler_from_quaternion(rot)
        #rotation=rotation_array[2]-rotation_start
        

        #set goal and total distance in first loop
        global count

        global goalx
        global goaly
        global goalz

        global x_start
        global y_start

        global total_distance_x
        global total_distance_y
        global total_distance
        global total_angle
        global previous_angle
        global last_rotation
        global rotation_start

        if count == 0:
                total_distance_x=2.5
                total_distance_y=1.75
                goalx=posx+total_distance_x
                goaly=posy+total_distance_y
                #goalz=posz+1 #Would make spin after reaching the destination i think
                total_distance=sqrt(pow(goalx,2)+pow(goaly,2))
                total_angle=0
                x_start=posx
                y_start=posy
		rotation_start=rotation_array[2] #getting .05ish initial rotation when should be 0

        rotation=rotation_array[2]-rotation_start*0
	print("rotation:", rotation)
        #determine angle
        path_angle = atan2(goaly - posy, goalx - posx)
        '''
        if path_angle < -pi/4 or path_angle > pi/4:
                if goaly<0 and posy<goaly:
                        path_angle = -2*pi+path_angle
                elif goaly >=0 and posy >goaly:
                        path_angle = 2*pi + path_angle
        '''
        if last_rotation >pi-.1 and roation <=0:
                rotation =2*pi + rotation
        elif last_rotation < -pi+.1 and roation >0:
                rotation = -2*pi + rotation
        
        #distance remaining each loop and difference from last loop
        #print (goalx, goaly, posx, posy)
        distance = sqrt(pow(goalx - posx,2) +pow(goaly - posy,2))
        global previous_distance
        diff_distance=distance-previous_distance
        diff_angle=path_angle-previous_angle

        #move to goal
        #if posx < goalx or posy<goaly:
        print("distance", distance, "angle", path_angle)
        if distance >.02:
                controlx = kp_distance*distance +ki_distance*total_distance*0+kd_distance*diff_distance#took out integral
                controlz = kp_angle*path_angle + ki_angle*total_angle*0 + kd_distance*diff_angle #took out integral
                move.linear.x=min(controlx,.1)
                if controlz >0:
                        move.angular.z=min(controlz-rotation,.5)
                else:
                        move.angular.z=max(controlz-rotation,-.5)

                print("posx: ",posx, "controlx: ", controlx)
                print("posy: ",posy, "controlz: ",controlz)
                print("distance", distance, "angle", path_angle, "angle deg", math.degrees(path_angle))
                #print("distance", distance, "total distance", total_distance, "diff distance",  diff_distance)

        else:
                move.linear.x = 0
                move.angular.z= 0
        
        pub.publish(move)
        count +=1

        last_rotation=rotation
        previous_distance=distance
        total_distance=total_distance+distance #now working, canceled in controlx eq
        previous_angle=path_angle
        total_angle=total_angle+path_angle #not working, cancled in controlz eq
#moving
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
move = Twist()

#odometry
rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)

#repeat
rospy.spin()



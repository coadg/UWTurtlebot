#5/4/21:
'''
#General
- Moves robot based on x,y,z location of 1 apriltag form family "tag36h11". My tag was 3 inches.
- Use angle calculation twice, then average the distancne of upper and lower points to get angle
- Commented out, commented block how to turn with angle.
- Commented out, fps: starts timer at beginning, breaks loop at 100 frames and tells the time it took.

#Errors:
-if corner leaves frame it will crash the program. could be worked around later.
-2nd tag depth is not accurate. (program not setup to show 2nd tag anyways)

#What you need to install
python realsense wrapper
numpy
opencv
apriltag

#Running
roscore
bring up on turtlebot 3
run this code on turtlebot 4

#Follow3 on my pi
'''
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import apriltag #apriltag
import argparse #apriltag
import math #need sqrt for tag width
import struct #bytes -> numbers
import rospy #python library for ROS
from geometry_msgs.msg import Twist  #moving bot
import time #fps calculation

#from pupil_apriltags import Detector

#function to make program stop correctly
import sys, signal
def signal_handler(signal, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

#fps counter
count=0
start = time.time()


#Get tag width from width and focal point
def get_size(tag_width):
        focalLength=625
        KNOWN_DISTANCE=10.6
        KNOWN_WIDTH=3.0

        #finds focal length from known measured width and distance, and width found in image
        #focalLength = (tag_width * KNOWN_DISTANCE) / KNOWN_WIDTH
        #print (focalLength)
        zsize = (KNOWN_WIDTH * focalLength) / tag_width * 25.4
        return zsize
#Get angle from 2 corners of tag
def get_angle(tag_width,ZL,ZR):
        depth_difference=(abs(ZL)-abs(ZR))
        #depth_difference=abs(abs(ZL)-abs(ZR))
        print(depth_difference)
        print(tag_width)
        if abs(depth_difference)<tag_width:
                print("dd",depth_difference,"tw",tag_width)
                angle=math.degrees(math.acos(abs(depth_difference)/tag_width))
                #angle=math.degrees(math.acos(depth_difference/tag_width))
                print(angle)
        else:
                print("angle cannot be found")
                angle=0
        if depth_difference<0:
                angle=-angle
        return (angle)

#main code
try:
        while True:

                ###-----------------------------
                ###Get Camera Data###
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                        continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                        images = np.hstack((resized_color_image, depth_colormap))
                else:
                        images = np.hstack((color_image, depth_colormap))
                frame = color_image
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                ###-----------------------------
                ### april tags###
                # define the AprilTags detector options and then detect the AprilTags in the input image
                #print("[INFO] detecting AprilTags...")
                options = apriltag.DetectorOptions(families="tag36h11")
                detector = apriltag.Detector(options)
                results = detector.detect(gray)

                #detector = Detector(families='tag36h11')
                #results  = detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

                print("[INFO] {} total AprilTags detected".format(len(results)))
                global amount_tags
                amount_tags=(len(results))

                ### Tag Name,Corners and Center Drawing###
                # loop over the AprilTag detection results
                for r in results:
                        ###-----------Get Corners, Draw Bounding Box, Get Center, Get Distance with Size
                        # extract the bounding box (x, y)-coordinates for the AprilTag
                        # and convert each of the (x, y)-coordinate pairs to integers
                        (ptA, ptB, ptC, ptD) = r.corners        #get integer points for cv2.line
                        ptBi = (int(ptB[0]), int(ptB[1]))
                        ptCi = (int(ptC[0]), int(ptC[1]))
                        ptDi = (int(ptD[0]), int(ptD[1]))
                        ptAi = (int(ptA[0]), int(ptA[1]))
                        # draw the bounding box of the AprilTag detection
                        cv2.line(frame, ptAi, ptBi, (0, 255, 0), 2)
                        cv2.line(frame, ptBi, ptCi, (0, 255, 0), 2)
                        cv2.line(frame, ptCi, ptDi, (0, 255, 0), 2)
                        cv2.line(frame, ptDi, ptAi, (0, 255, 0), 2)
                        # draw the center (x, y)-coordinates of the AprilTag
                        (X, Y) = (int(r.center[0]), int(r.center[1]))
                        #cv2.circle(frame, (ptAi), 5, (0, 0, 255), -1)



                        # draw the tag family on the image
                        tagFamily = r.tag_family.decode("utf-8")
                        cv2.putText(frame, tagFamily, (ptAi[0], ptAi[1] - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        #print("[INFO] tag family: {}".format(tagFamily))

                        ###distance from size###
                        tagx = (abs(abs(ptA[0])-abs(ptB[0]))) #compare x's
                        tagy = (abs(abs(ptA[1])-abs(ptB[1]))) #compare y's
                        tag_width= math.sqrt(tagx**2+tagy**2) #pythagorean for width

                        #test if different enough to be worth using floats
                        tagx2 = (abs(abs(ptA[0])-abs(ptB[0]))) #compare x's
                        tagy2 = (abs(abs(ptA[1])-abs(ptB[1]))) #compare y's
                        tag_width2= math.sqrt(tagx2**2+tagy2**2) #pythagorean for width

                        #check if x exsists to avoid error when no tag

                        #if 0 < ptAi[0] < xmax and .... and 0< ptAi[1]] <ymax
                        if 'X' in locals():
                                Z=depth_frame.get_distance(X,Y)*1000
                                #gives error when tag leaves cam
                                ZA=depth_frame.get_distance(ptAi[0],ptAi[1])*1000
                                ZB=depth_frame.get_distance(ptBi[0],ptBi[1])*1000
                                ZC=depth_frame.get_distance(ptCi[0],ptCi[1])*1000
                                ZD=depth_frame.get_distance(ptDi[0],ptDi[1])*1000
                                #get x,y,z in meters
                                depth_scale = .001 #meters for d435
                                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, r.center, Z*depth_scale)
                                #replace image xyz with depth xyz
                                X=depth_point[0]
                                Y=depth_point[1]
                                Z=depth_point[2]
                        else:
                                X=0
                                Y=0
                                Z=350 #to not move

                        #get tag depth from size
                        zsize=get_size(tag_width)
                        zsize2=get_size(tag_width2)
                        #get angle of tag
                        #if Ax-Bx > Ay-By, tag is horizontal, use a and b, if not the tag is flipped 90, use A and C
                        if abs(abs(ptAi[0])-abs(ptBi[0])) > abs(abs(ptAi[1])-abs(ptBi[1])):
                                angle1=get_angle(tag_width,ZA,ZB)
                                angle2=get_angle(tag_width,ZD,ZC)
                        else:
                                angle1=get_angle(tag_width,ZA,ZC)
                                angle2=get_angle(tag_width,ZD,ZC)

                        if angle1 !=0 and angle2 !=0 or angle1==angle2:
                                angle=(angle1+angle2)/2 #average for better result, or return 0 if they both are
                        elif angle1 ==0 and angle2 !=0:
                                angle=angle2            #ignore one if it is zero
                        elif angle1 !=0 and angle2 ==0:
                                angle=angle1

                        #cv2.putText(frame, "%.2f Zsize" % (zsize),(frame.shape[1]-200, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
                        cv2.putText(frame, "%.2f angle" % (angle),(frame.shape[1]-200, frame.shape[0] - 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
                        #cv2.putText(frame, "%.2f angle2" % (angle2),(frame.shape[1]-200, frame.shape[0] - 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
                        #cv2.putText(frame, "%.2f angle1" % (angle1),(frame.shape[1]-200, frame.shape[0] - 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)          
                        cv2.putText(frame, "%.2f X" % (X),(20, frame.shape[0] - 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
                        cv2.putText(frame, "%.2f Y" % (Y),(20, frame.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
                        cv2.putText(frame, "%.2f Z" % (Z),(20, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

                #Ros command to motors
                WAFFLE_MAX_LIN_VEL = 0.26
                WAFFLE_MAX_ANG_VEL = 1.82

                # Tells rospy the name of the node, Anonymous=True gives node unique name.
                rospy.init_node('test6.0',anonymous=True)
                # publish move command
                pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
                move = Twist()

                #control by location
                if amount_tags == 1:
                        #forward/back
                        if Z > .35:
                                move.linear.x = 0.075
                        elif Z < .3:
                                move.linear.x = -0.075
                        else:
                                move.linear.x=0
                        #left/right, frame = 620x480, right angle turn +, left angle turn -
                        if X > .36:
                               move.angular.z = -0.2 #right
                        elif X > .32:
                               move.angular.z=-.1
                        elif X > .28:
                               move.angular.z=-.05
                        elif X < .22:
                               move.angular.z=.05
                        elif X < .18:
                               move.angular.z=.01
                        elif X < .14:
                                move.angular.z = 0.2 #left
                        else:
                                move.angular.z=0
                        print("X:",round(X,2),'Y:',round(Y,2),'Z:',round(Z,2))
                        '''
                        #right/left angle
                        if angle < 87 and angle > 0:
                                move.angular.z = -0.25
                        elif angle > -87 and angle < 0:
                                move.angular.z = 0.25
                        else:
                                move.angular.z=0
                        '''
                else:
                        move.linear.x=0
                        move.angular.z = 0
                print("Move Angular:", move.angular.z)
                print('=============================================')
                #end angle turn

                pub.publish(move)
                #cv2.imshow('Apriltag',frame)
                cv2.waitKey(1)

                '''
                #fps
                count=count+1
                if count == 100:
                        stop = time.time()
                        fps=count/(stop-start)
                        print("fps over 100 frames",fps)
                        #(laptop 23 fps with everything on)
	                        #pi 3.5fps everything on
                        #Ros pi .19 fps
                        break
                '''
finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()


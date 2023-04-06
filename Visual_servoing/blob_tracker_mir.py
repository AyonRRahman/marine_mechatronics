#!/usr/bin/env python
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken
import rospy
import tf
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist


###---- Visual Tracking and Servoing----
from sensor_msgs.msg import CompressedImage
import cv2
import copy
import time
import sys
import argparse

# ---------- Global Variables ---------------
global nb_points_vs
global reset_desired_points
global desired_points
global previous_points

nb_points_vs = 5
reset_desired_points = False
desired_points = []
flag_alert = False


global keypoints

keypoints = 'none'
# reset_desired_points = False

def find_cg(points):
    n = len(points)

    # Compute the sum of x and y coordinates
    sum_x = sum([p[0] for p in points])
    sum_y = sum([p[1] for p in points])

    # Compute the center of gravity (CG)
    CG_x = sum_x / n
    CG_y = sum_y / n
    points.append((CG_x, CG_y))
    return points


def match_point(new_keypoints, prev_points):
    new_ordered_point = []
    for p in prev_points:
        distances = [math.sqrt((new[0] - p[0])**2 + (new[1] - p[1])**2) for new in new_keypoints]
        
        new_ordered_point.append(new_keypoints[distances.index(min(distances))])
        
      

    # print(new_ordered_point)
    return new_ordered_point


def cameracallback(image_data):
    global nb_points_vs
    global reset_desired_points
    global desired_points
    global previous_points
    ##
    # get image data
    np_arr = np.fromstring(image_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    position = (10,30)
    
    global keypoints
    #blob detect
    if reset_desired_points:
        
        detector = cv2.SimpleBlobDetector_create() 
        
        keypoints = detector.detect(image_np)
        print(f"key_points {type(keypoints[0])}")
        # print(f"key_point 1 {type(keypoints[0].pt)}")
        desired_points = []
        for points in keypoints:
            desired_points.append(points.pt)
        reset_desired_points = False
        
        # desired_points = find_cg(desired_points)

        previous_points = desired_points
        

    elif len(desired_points)!=0:
        
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 100
        detector = cv2.SimpleBlobDetector_create(params)     
        
        new_keypoints = detector.detect(image_np)

        stored_keypoints_cv2 = [cv2.KeyPoint(x, y, 1) for (x, y) in previous_points]
        current_points = []
        
        
        for points in new_keypoints:
            current_points.append(points.pt)
            
        

        current_points = match_point(current_points, previous_points)
        
        previous_points = current_points
        new_keypoints = [cv2.KeyPoint(x, y, 10) for x, y in current_points]

        for i, kp in enumerate(new_keypoints):
            # Draw a circle around the keypoint
            cv2.circle(image_np, (int(kp.pt[0]), int(kp.pt[1])), int(kp.size/2), (0, 255, 0), 2)
            # Add text label with the keypoint number
            cv2.putText(image_np, str(i+1), (int(kp.pt[0]-kp.size), int(kp.pt[1]-kp.size)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        
        

    text = "Click on the image : reset desired point."
    cv2.putText(image_np,text,position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255, 255),1)

    cv2.namedWindow("image")
    
    # cv2 mouse
    if keypoints!='none':

        # image_np = cv2.drawKeypoints(image_np, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        for i, kp in enumerate(keypoints):
         # Draw a circle around the keypoint
            cv2.circle(image_np, (int(kp.pt[0]), int(kp.pt[1])), int(kp.size/2), (0, 0, 255), 2)
            # Add text label with the keypoint number
            cv2.putText(image_np, str(i+1), (int(kp.pt[0]-kp.size), int(kp.pt[1]-kp.size)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)



    cv2.setMouseCallback("image", click_detect)   
    cv2.imshow("image", image_np)
    
    cv2.waitKey(2)

def click_detect(event,x, y, flags, param):
    global reset_desired_points
    if event == cv2.EVENT_LBUTTONDOWN:
        reset_desired_points = True
        print ("desired_points to update")
   


def subscriber():
    #camera
    rospy.Subscriber("usb_cam/image_raw/compressed", CompressedImage, cameracallback,  queue_size = 1)
    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':
    
    rospy.init_node('blob_tracker_mir', anonymous=False)  
    
    print('tracker launched')
    
    if rospy.has_param('~points'):
        nb_points_vs = rospy.get_param('~points')
        print ("target with", nb_points_vs, " points.")
    else:
        rospy.logwarn('no parameter given; using the default value %d' %nb_points_vs)
    
    pub_tracked_point = rospy.Publisher("tracked_points",Float64MultiArray,queue_size=1,tcp_nodelay = True)
    pub_desired_point = rospy.Publisher("desired_points",Float64MultiArray,queue_size=1,tcp_nodelay = True)
    subscriber()



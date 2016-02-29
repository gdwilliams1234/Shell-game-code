#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2, cv_bridge
import numpy as np 
import cv

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window",1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

  def image_callback(self, msg):
     frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding= 'bgr8')

     # Convert BGR to HSV
     blur = cv2.blur(frame,(20,20))
     hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
     lower_blue = np.array([167,20,150])
     upper_blue = np.array([180,177,220])

    # Threshold the HSV image to get only blue colors
     mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
     res = cv2.bitwise_and(frame,frame, mask= mask)
    

     cv2.imshow('frame',frame)
     cv2.imshow('mask',mask)
     cv2.imshow('res',res)


     key = cv2.waitKey(20)
      

rospy.init_node('follower')
follower = Follower()
rospy.spin()

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
    self.previous = None
  def image_callback(self, msg):
     frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding= 'bgr8')


     # Convert BGR to HSV
     blur = cv2.blur(frame,(20,20),0)
     hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
     lower_blue = np.array([145, 40, 150])
     upper_blue = np.array([175, 255, 255])

    # Threshold the HSV image to get only blue colors
     mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
     res = cv2.bitwise_and(frame,frame, mask= mask)

     if self.previous is None:
       self.previous = mask

     frameDelta = cv2.absdiff(self.previous, mask)
     thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

     thresh = cv2.dilate(thresh, None, iterations=2)
     (cnts, _)= cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

     for c in cnts:
       if cv2.contourArea(c) < 5:
         continue

         (x, y, w, h) = cv2.boundingRect(c)
         cv2.rectangle (frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
         text = "Motion"

     cv2.imshow('frame',frame)
     cv2.imshow('mask',mask)
     cv2.imshow('res',res)
     cv2.imshow('thresh', thresh)
     cv2.imshow('delta', frameDelta)


     key = cv2.waitKey(20)
     self.previous = mask

rospy.init_node('follower')
follower = Follower()
rospy.spin()

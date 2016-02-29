#!/usr/bin/env python

# import the necessary packages
import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2, cv_bridge
from collections import deque
import numpy as np
import argparse
import math
import time
import os
from std_msgs.msg import String



class Movement:

    #the range of colors that will be processed
    color_lower = (150, 45, 150)
    color_upper = (170, 250, 255)

    buffer_size = 2048

    minimum_contour_radius = 10

    blur_strength = 11

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window",1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image,
            self.image_callback)

        self.pts = deque(maxlen=self.buffer_size)
        #adds global variable to save the last frame allowing for compaison
        self.last_circle = None


    def convert_and_preprocess(self, image_msg):
        """
        Convert a ROS image messge to OpenCV format and HSV color.
        """
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding= 'bgr8')
        #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return frame, hsv

    def mask_and_despeckle(self, hsv_image):
        """
        construct a mask for the color "green", then perform
        a median blur to get rid of small speckles
        """
        mask = cv2.inRange(hsv_image, self.color_lower, self.color_upper)
        mask = cv2.medianBlur(mask, self.blur_strength)
        return mask

    #returns a list of circles
    #filters out small areas
    def filter_by_radius(self, contours):
        return [contour for contour in contours if contour[1] > self.minimum_contour_radius]

    def location(self, place_pixels):
        #takes a integer representing the number of pixels
        #returns place on screen by placement of x pixels
        if place_pixels < 200:
            #return place_pixels
            return "The correct shell is on the right."
        if 200 < place_pixels < 420:
            #return place_pixels
            return "The correct shell is in the middle."
        if place_pixels > 420:
            #return place_pixels
            return "The correct shell is on the left"

    def decompose_circle(self, circle):
        #takes a circle
        #returns a tuple of x, y, radius
        return circle[0][0], circle[0][1], circle[1]

    def Distance_Between(self, circle1, circle2):
        #finds the distance between two circles
        #returns the distance
        cir1 = self.decompose_circle(circle1)
        cir2 = self.decompose_circle(circle2)
        squared = (cir1[0] - cir2[0])**2 + (cir1[1] - cir2[1])**2
        return squared**.5


    def match_circles(self, last, new_circles):
        #finds the closest circle off of the last frame captured
        ((last_x, last_y), last_radius) = last

        def distance_to(new_circle):
            ((new_x, new_y), new_radius) = new_circle
            return (new_x - last_x)**2 + (new_y - last_y)**2

        new_circles.sort(key=distance_to)
        return new_circles[0]


    def image_callback(self, msg):
        img, hsv = self.convert_and_preprocess(msg)
        mask = self.mask_and_despeckle(hsv)
        cv2.imshow("mask", mask)

        # find contours in the mask
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]

        new_circles = [cv2.minEnclosingCircle(contour) for contour in contours]

        new_circles = self.filter_by_radius(new_circles)

        if self.last_circle == None:
            if new_circles:
                self.last_circle = new_circles[0]
            else:
                return
        else:
            if new_circles:
                potential = self.match_circles(self.last_circle, new_circles)
                dist = self.Distance_Between(potential, self.last_circle)
                if dist < 100:
                    self.last_circle = potential

        ((x,y), radius) = self.last_circle
        center = int(x), int(y)
        cv2.circle(img, center, int(radius),
            (0, 255, 255), 2)
        cv2.circle(img, center, 5, (0, 0, 255), -1)

        cv2.imshow("circles", img)
        key = cv2.waitKey(1)



rospy.init_node('movement')
movement = Movement()
rospy.spin()

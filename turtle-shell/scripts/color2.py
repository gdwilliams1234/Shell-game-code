#!/usr/bin/env python


import rospy

import cv2
import numpy as np
import sys

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "You must give an argument to open a video stream."
        print "  It can be a number as video device, e.g.: 0 would be /dev/video0"
        print "  It can be a url of a stream,        e.g.: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov"
        print "  It can be a video file,             e.g.: myvideo.mkv"
        exit(0)

    resource = sys.argv[1]
    # If we are given just a number, interpret it as a video device
    if len(resource) < 3:
        resource_name = "/dev/video" + resource
        resource = int(resource)
    else:
        resource_name = resource
    print "Trying to open resource: " + resource_name
    cap = cv2.VideoCapture(resource)
    if not cap.isOpened():
        print "Error opening resource: " + str(resource)
        print "Maybe opencv VideoCapture can't open it"
        exit(0)

    print "Correctly opened resource, starting to show feed."
    rval, frame = cap.read()
    while rval:
        cv2.imshow("Stream: " + resource_name, frame)
        rval, frame = cap.read()
 # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

    # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)

        cv2.imshow('frame',frame)
        cv2.imshow('mask',mask)
        cv2.imshow('res',res)

        key = cv2.waitKey(20)
        # print "key pressed: " + str(key)
        # exit on ESC, you may want to uncomment the print to know which key is ESC for you
        if key == 27 or key == 1048603:
            break


cv2.destroyAllWindows()

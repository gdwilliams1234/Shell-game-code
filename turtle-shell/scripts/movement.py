#!/usr/bin/env python

# import the necessary packages
import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2, cv_bridge
from collections import deque
import numpy as np
import argparse
import math 

class Movement:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window",1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        
        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video",
                        help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=32,
                        help="max buffer size")
        args = vars(ap.parse_args())

        self.pts = deque(maxlen=args["buffer"])
        self.PX = 1
        self.PY = 1
        

    def image_callback(self, msg):

        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video",
                        help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=32,
                        help="max buffer size")
        args = vars(ap.parse_args())
        

        # define the lower and upper boundaries of the "green"
        # ball in the HSV color space
        greenLower = (150, 40, 150)
        greenUpper = (170, 190, 240)

        # initialize the list of tracked points, the frame counter,
        # and the coordinate deltas
        counter = 0
        (dX, dY) = (0, 0)
        direction = ""
		

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding= 'bgr8')
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid		
         # 
	  
	    distp=90000
            S=1
            Ha = len(cnts)
	    for i in range (1, Ha): 
                ((x, y), radius) = cv2.minEnclosingCircle(cnts[i])
                M = cv2.moments(i)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  
                sql= ((int(x)-int(self.PX))*(int(x)-int(self.PX)))
                sqt = (int(y)-self.PY)*(int(y)-self.PY) 
                dist = math.sqrt(int (sql) + int(sqt))
                if dist < distp:
                    distp = dist
                    S=i


            ((x, y), radius) = cv2.minEnclosingCircle(cnts[S])
            M = cv2.moments(S)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))         
            
      


		# only proceed if the radius meets a minimum size
            if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			self.pts.appendleft(center)
			self.PX = int(x)
            		self.PY = int(y)

	# loop over the set of tracked points
	for i in np.arange(1, len(self.pts)):
		# if either of the tracked points are None, ignore
		# them
		if self.pts[i - 1] is None or self.pts[i] is None:
                    continue

		# check to see if enough points have been accumulated in
		# the buffer
		if counter >= 10 and i == 1 and self.pts[-10] is not None:
			# compute the difference between the x and y
			# coordinates and re-initialize the direction
			# text variables
			dX = self.pts[-10][0] - self.pts[i][0]
			dY = self.pts[-10][1] - self.pts[i][1]
			(dirX, dirY) = ("", "")

			# ensure there is significant movement in the
			# x-direction
			if np.abs(dX) > 20:
				dirX = "East" if np.sign(dX) == 1 else "West"

			# ensure there is significant movement in the
			# y-direction
			if np.abs(dY) > 20:
				dirY = "North" if np.sign(dY) == 1 else "South"

			# handle when both directions are non-empty
			if dirX != "" and dirY != "":
				direction = "{}-{}".format(dirY, dirX)

			# otherwise, only one direction is non-empty
			else:
				direction = dirX if dirX != "" else dirY

                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

	# show the movement deltas and the direction of movement on
	# the frame
	cv2.putText(frame, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
		0.65, (0, 0, 255), 3)
	cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY),
		(10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
		0.35, (0, 0, 255), 1)

	# show the frame to our screen and increment the frame counter
	cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
	key = cv2.waitKey(1) & 0xFF
	counter += 1

rospy.init_node('movement')
movement = Movement()
rospy.spin()

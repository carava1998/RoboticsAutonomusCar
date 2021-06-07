#!/usr/bin/env python3
from numpy.core.numeric import False_
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
from checkerboard import detect_checkerboard
import glob
import os

bridge = CvBridge()
pub = rospy.Publisher('carsim/cmd_vel', Twist, queue_size = 1)


def findStopSign(cv_image):
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    dilate = cv2.dilate(gray,np.ones((3, 3), np.uint8))
    erode = cv2.erode(dilate,np.ones((7, 7), np.uint8))
    found = False
    """ cv2.imshow('img1',gray) """
    ret,thresh1 = cv2.threshold(erode, 0, 255,cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    """ cv2.imshow('thresh1', thresh1) """
    contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.04*cv2.arcLength(cnt,True),True)
        if len(approx) == 8:
            found = True
            cv2.drawContours(cv_image, [cnt], 0, (0, 255, 0), 6)
            cv2.imshow('img1',erode)
            cv2.imshow('thresh1', thresh1)
            cv2.imshow('sign', cv_image)
            cv2.waitKey(0)
    if found:
        print("found")
        twist.linear.x=0; twist.linear.y=0; twist.linear.z=0;
        twist.angular.x=0; twist.angular.y=0; twist.angular.z=0;
        pub.publish(twist)
    else:
        print("Not found")
    """ cv2.imshow('sign', cv_image)  """



def findCrossWalk(cv_image):
    nrows = 4
    ncol = 4
    size = (ncol, nrows) # size of checkerboard
    image = cv_image # obtain checkerboard
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, score = detect_checkerboard(gray, size)
    print("Score:",score)
    print("Corners:",corners)

def image_callback(ros_image):
    global bridge
    #Convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

    except CvBridgeError as e:
        print(e)
    # Work in opencv-compatible
    findStopSign(cv_image)

def main(args):
    rospy.init_node('image_converter', anonymous=False)
    image_sub = rospy.Subscriber("/carsim/camera1/image_raw1",Image,image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

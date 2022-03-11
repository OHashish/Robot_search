#!/usr/bin/env python
from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class Robot():
    def __init__(self, rate):
        self.bridge = CvBridge()
        self.redflag = 0
        self.greenflag = 0
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        self.rate = rate
        rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self.sensitivity = 10
    def callback():
        desired_velocity = Twist()
        cv = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.namedWindow('camera_Feed')
        hsv_green_lower = np.array([60-self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60+self.sensitivity, 255, 255])
        hsv_red_lower = np.array([10-self.sensitivity, 100, 100])
        hsv_red_upper = np.array([10+self.sensitivity, 255, 255])
        hsv_image = cv2.cvtColor(cv, cv2.COLOR_BGR2HSV)
        mskg = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        mskr = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
        gr = cv2.bitwise_and(cv, cv, mask=mskg)
        rd = cv2.bitwise_and(cv, cv, mask=mskr)
        graygr = cv2.cvtColor(gr, cv2.COLOR_BGR2GRAY)
        grayrd = cv2.cvtColor(rd, cv2.COLOR_BGR2GRAY)
        combmsk = cv2.bitwise_or(mskg, mskr)
        com = cv2.bitwise_and(cv, cv, mask=combmsk)
        greencontours, ghierarcy = cv2.findContours(graygr, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        redcontours, rhierarcy = cv2.findContours(grayrd, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(greencontours)>0:
            cg = max(greencontours, key=cv2.contourArea)
            if cv2.contourArea(cg) > 300:
                (x, y), radius = cv2.minEnclosingCircle(cg)
                circil = cv2.circle(res,(int(x),int(y)),int(radius),(255,0,255),1)
                self.greenflag = 1
        else:
            self.greenflag = 0
        if len(redcontours)>0:
            cr = max(redcontours, key=cv2.contourArea)
            if cv2.contourArea(cr) > 300:
                (x, y), radius = cv2.minEnclosingCircle(cr)

                circil = cv2.circle(res,(int(x),int(y)),int(radius),(0,255,255),1)
                self.redflag = 1
        else:
            self.redflag = 0
        if self.redflag == 1:
            pass
        if self.greenflag == 1:
            if cv2.contourArea(cg) > 15000:
                pass
            elif cv2.contourArea(cg) < 15000:
                pass
            elif cv2.contourArea(cg) == 15000:
                pass
        self.pub.publish(desired_velocity)
        self.rate.sleep()
        cv2.imshow('camera_Feed', res)
        cv2.waitKey(3)

def main(argv):
    rospy.init_node('camera_Feed', anonymous=True)
    robot = Robot(rospy.Rate(10))
    rospy.spin()

if __name__ == '__main__':
    # Your code should go here. You can break your code into several files and
    # include them in this file. You just need to make sure that your solution 
    # can run by just running rosrun group_project main.py
    main(sys.argv)

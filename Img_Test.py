#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

#Generator node
def callback(data):
    pub = rospy.Publisher('webcam_gray', Image, queue_size=1)
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    rospy.loginfo(ids)
    img_msg = bridge.cv2_to_imgmsg(gray, "mono8")
    pub.publish(img_msg)

def Img_Test():

    #Initialize node, publisher, and subscriber
    rospy.init_node('Img_Test', anonymous=True)
    rospy.Subscriber('webcam_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        Img_Test()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

#Generator node
def webcam_pub():

    #Initialize node, publisher, and subscriber
    rospy.init_node('webcam_pub', anonymous=True)
    pub = rospy.Publisher('webcam_raw', Image, queue_size=1)
    bridge = CvBridge()
    gst_pipeline = (
        'v4l2src device=/dev/video0 ! '
        'image/jpeg, width=640, height=480, framerate=30/1 ! '
        'jpegdec ! videoconvert ! appsink'
    )
    capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    if not capture.isOpened():
        rospy.logerr("Camera didn't open")
        return

    if capture.isOpened():
        rospy.loginfo("Camera opened")

    for _ in range(5):
        capture.read()
        rospy.sleep(0.1)

    #Set rate to 10Hz
    rate = rospy.Rate(10)

    #Loop until the program is stopped
    while not rospy.is_shutdown():

        ret, img = capture.read()
        if not ret:
            rospy.logerr("Frame not found")
            break
        try:
            img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
            pub.publish(img_msg)
        except CvBridgeError as error:
            print(error)

        #Sleep for rate
        #rate.sleep()

if __name__ == '__main__':
    try:
        webcam_pub()
    except rospy.ROSInterruptException:
        pass
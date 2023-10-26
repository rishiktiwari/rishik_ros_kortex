#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2 as cv                    # opencv version 3.4.0.14

subscriberNode = 'rishik_opencv'
# topicName = '/my_gen3/image_raw'
topicName = '/my_gen3/camera_image/compressed'

def imageCallback(msg):
    # print("rcv frame")
    bridgeObj = CvBridge()
    convertedFrame = bridgeObj.compressed_imgmsg_to_cv2(msg, desired_encoding='rgb8')
    # print(convertedFrame)
    cv.imshow("preview", convertedFrame)
    cv.waitKey(1)

print("starting...")
rospy.init_node(subscriberNode, anonymous=True)
rospy.Subscriber(topicName, CompressedImage, imageCallback)
rospy.spin()
cv.destroyAllWindows()

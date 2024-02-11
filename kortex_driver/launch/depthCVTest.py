#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv    # opencv version 3.4.0.14
import numpy as np


# DEPTH_CAMERA_TOPIC = '/my_gen3/camera/depth/image_raw'    # For Sim Arm - depth
DEPTH_CAMERA_TOPIC = '/my_gen3/camera/depth/image_raw'
# DEPTH_CAMERA_TOPIC = '/camera/depth/image_raw'          # For Real Arm

def imageCallback(msg):
    try:
        bridgeObj = CvBridge()
        convertedFrame = bridgeObj.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        convertedFrame = cv.normalize(convertedFrame, None, 0, 1, cv.NORM_MINMAX, dtype=cv.CV_32F)
        convertedFrame *= 255
        convertedFrame = convertedFrame.astype(np.uint8)
        # (_, convertedFrame) = cv.imencode(".jpg", convertedFrame, [int(cv.IMWRITE_JPEG_QUALITY), 50])
        # print(convertedFrame)

    except (CvBridgeError, Exception, KeyboardInterrupt) as e:
        print(e)
        rospy.signal_shutdown('')

    cv.imshow("preview", convertedFrame)
    if cv.waitKey(30) == 27:
        rospy.signal_shutdown('')
    

rospy.init_node('rishik_depthCVTest', anonymous=True)
rospy.Subscriber(DEPTH_CAMERA_TOPIC, Image, imageCallback)
rospy.spin()
cv.destroyAllWindows()
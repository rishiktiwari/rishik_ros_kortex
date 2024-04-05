#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2 as cv                    # opencv version 3.4.0.14

subscriberNode = 'rishik_opencv'
topicName = '/camera/color/image_raw'
# topicName = '/my_gen3/camera_image/compressed'

def imageCallback(msg):
    global cvVw
    # print("rcv frame")
    bridgeObj = CvBridge()
    convertedFrame = bridgeObj.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # convertedFrame = cv.cvtColor(convertedFrame, cv.COLOR_BGR2GRAY)
    # print(convertedFrame.shape)
    cv.imshow("preview", convertedFrame)
    cvVw.write(convertedFrame)

    if cv.waitKey(2) == 27: #ESC
        rospy.signal_shutdown('')


if __name__ == '__main__':
    global cvVw
    videoExportDirectory = '/media/rishik/Installs/Kinova Recordings/'
    videoFileName = 'pickPlaceTask_banana_2--rgb'

    print("starting...")
    videoExportPath = videoExportDirectory + videoFileName + '.mp4'
    fourcc = cv.VideoWriter_fourcc(*'mp4v')
    cvVw = cv.VideoWriter(videoExportPath, fourcc, 30.0, (640,480))

    rospy.init_node(subscriberNode, anonymous=True)
    rospy.Subscriber(topicName, Image, imageCallback)
    rospy.spin()

print('quitting...')
cvVw.release()
cv.destroyAllWindows()

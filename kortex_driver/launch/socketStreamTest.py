#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 as cv                    # opencv version 3.4.0.14
import pickle
import socket
from time import time

subscriberNode = 'rishik_opencv'
# topicName = '/my_gen3/image_raw'
topicName = '/my_gen3/camera_image/compressed'

# client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, int(10e6))

host_ip = '192.168.1.104' # MBP addr
port = 9999
client_socket.connect((host_ip,port))

def imageCallback(msg):
    bridgeObj = CvBridge()
    convertedFrame = bridgeObj.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    cv.putText(convertedFrame, str(int(time())), (530,465), cv.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, cv.LINE_AA)
    cv.imshow("preview", convertedFrame)
    (_, buf) = cv.imencode(".jpg", convertedFrame, [int(cv.IMWRITE_JPEG_QUALITY), 80])
    imBytes = pickle.dumps(buf)
    # print(len(imBytes))

    try:
        client_socket.sendto((imBytes), (host_ip, port))
    except:
        print("--NO_CONN--")

    cv.waitKey(1)

print("starting...")
rospy.init_node(subscriberNode, anonymous=True)
rospy.Subscriber(topicName, CompressedImage, imageCallback)
rospy.spin()
client_socket.close()
cv.destroyAllWindows()
print("\nSocket closed\n")

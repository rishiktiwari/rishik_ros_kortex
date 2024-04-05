#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, JointState
from cv_bridge import CvBridge
import cv2 as cv    # opencv version 3.4.0.14
import pickle, socket, struct, threading
from time import time

subscriberNode = 'rishik_opencv'

# topicName = '/my_gen3/camera_republished/compressed' # may not be required for sim
# topicName = '/my_gen3/camera_image/compressed'
topicName = '/my_gen3/camera/color/image_raw/compressed'    # For Sim Arm - color
# topicName = '/camera/color/image_raw/compressed'          # For Real Arm

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, int(10e6))

host_ip = '192.168.1.104' # MBP addr
# host_ip = '192.168.1.100' # MBP addr
port = 9999
client_socket.connect((host_ip,port))


def imageCallback(msg):
    bridgeObj = CvBridge()
    convertedFrame = bridgeObj.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # convertedFrame = convertedFrame[120:600, 320:960] # crop and resize 720x1280 to 480x640, yes it is HxW
    convertedFrame = cv.resize(convertedFrame, (640,480))
    cv.putText(convertedFrame, str(int(time())), (530,465), cv.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, cv.LINE_AA)
    cv.imshow("preview", convertedFrame)
    (_, buf) = cv.imencode(".jpg", convertedFrame, [int(cv.IMWRITE_JPEG_QUALITY), 50])
    imBytes = pickle.dumps(buf)
    # print(len(imBytes))

    try:
        client_socket.sendto((imBytes), (host_ip, port))
    except:
        print("--NO_CONN--")

    cv.waitKey(1)


def initDataLink():
    print('TODO: move from file')


print("starting...")
# initDataLink()
# rospy.init_node(subscriberNode, anonymous=True)
# rospy.Subscriber(topicName, CompressedImage, imageCallback)
# rospy.spin()



# ---- ONLY FOR TESTING ----
# cam = cv.VideoCapture(0)
# cam.set(cv.CAP_PROP_FRAME_WIDTH, 640)
# cam.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

# while True:
#     (ret, frame) = cam.read()
#     if not ret:
#         print("Cannot open camera, exiting...")
#         break

#     # frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
#     (_, buf) = cv.imencode(".jpg", frame, [int(cv.IMWRITE_JPEG_QUALITY), 50])
#     imBytes = pickle.dumps(buf)

#     try:
#         client_socket.sendto((imBytes), (host_ip, port))
#     except:
#         print("--NO_CONN--")
    
#     # cv.imshow('Camera Feed', frame)
#     if cv.waitKey(1) == ord('q'):
#         break


# client_socket.close()
# cv.destroyAllWindows()
# print("\nSocket closed\n")

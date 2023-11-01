#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, JointState
from cv_bridge import CvBridge
import cv2 as cv    # opencv version 3.4.0.14
import pickle, socket, struct, threading
from time import time, sleep



HOST_ADDR = ('0.0.0.0', 9999)
rospy.init_node('rishik_socketstreamer', anonymous=True)

JOINT_STATE_TOPIC = '/my_gen3/joint_states'
# COLOR_CAMERA_TOPIC = '/my_gen3/camera_republished/compressed' # may not be required for sim
# COLOR_CAMERA_TOPIC = '/my_gen3/camera_image/compressed'
COLOR_CAMERA_TOPIC = '/my_gen3/camera/color/image_raw/compressed'    # For Sim Arm - color
# COLOR_CAMERA_TOPIC = '/camera/color/image_raw/compressed'          # For Real Arm

video_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, int(10e6))

client_addr = None          # for video stream
data_client_socket = None   # for data stream



def sendVideoFrame(msg):
    bridgeObj = CvBridge()
    convertedFrame = bridgeObj.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # convertedFrame = convertedFrame[120:600, 320:960] # crop and resize 720x1280 to 480x640, yes it is HxW
    convertedFrame = cv.resize(convertedFrame, (640,480))
    cv.putText(convertedFrame, str(int(time())), (530,465), cv.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, cv.LINE_AA)
    # cv.imshow("preview", convertedFrame)
    (_, buf) = cv.imencode(".jpg", convertedFrame, [int(cv.IMWRITE_JPEG_QUALITY), 50])
    imBytes = pickle.dumps(buf)

    try:
        video_client_socket.sendto((imBytes), client_addr)
    except Exception as e:
        print(e)
        print("Video send fail")
        rospy.signal_shutdown('')



def sendStateData(msg):
    global data_client_socket
    data = {
        "name": msg.name,
        "pos": msg.position,
        "vel": msg.velocity
    }
    # print(data)
    try:
        check = data_client_socket.sendall(pickle.dumps(data))
        if(check != None):
            print("Data send incomplete/failed")

    except Exception as e:
        print(e)
        print('Client unavailable')
        rospy.signal_shutdown('')



connectedSocketCount = 0
def spinRos():
    # only spin ros when both video and data sockets are connected
    global connectedSocketCount
    connectedSocketCount += 1
    
    if(connectedSocketCount == 2):
        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, sendStateData)
        rospy.Subscriber(COLOR_CAMERA_TOPIC, CompressedImage, sendVideoFrame)
        rospy.spin()



def initVideoLink():
    global client_addr
    print('\nInit video link...')

    video_client_socket.bind(HOST_ADDR)
    print('Video link awaiting client at', HOST_ADDR)

    (msg, addr) = video_client_socket.recvfrom(1024)
    if(msg.decode('utf-8') == 'namaste'):
        client_addr = addr
        print('Client connected', client_addr)
        spinRos()



def initDataLink():
    global data_client_socket
    print('\nInit data link...')
    addr = ('0.0.0.0', 9998)
    rcvmsg_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #TCP
    rcvmsg_socket.bind(addr)
    rcvmsg_socket.listen(5)
    print('Data link listening at', addr)

    (data_client_socket, addr) = rcvmsg_socket.accept()
    if data_client_socket:
        print('Client connected ' + addr[0])
        spinRos()



print("Launching...")
vidLinkThread = threading.Thread(target=initVideoLink, args=())
dataLinkThread = threading.Thread(target=initDataLink, args=())

vidLinkThread.start()
dataLinkThread.start()

# while True:
#     sleep(1)
#     try:
#         if (not vidLinkThread.is_alive() and not dataLinkThread.is_alive()):
#             break
#     except KeyboardInterrupt:
#         break

# rospy.signal_shutdown('')

# if video_client_socket:
#     video_client_socket.close()

# if data_client_socket:
#     data_client_socket.close()

# cv.destroyAllWindows()
# print("\Data stream closed\n")

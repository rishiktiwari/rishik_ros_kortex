#!/usr/bin/env python

'''
This script starts the TCP server to stream RGB, Depth image and joint states to client.
'''

import rospy, message_filters, json
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2 as cv    # opencv version 3.4.0.14
import numpy as np
import pickle, socket, threading, sys, math
from time import time, sleep



RGB_HOST_ADDR = ('0.0.0.0', 9999)
DEPTH_HOST_ADDR = ('0.0.0.0', 9998)
DATA_HOST_ADDR = ('0.0.0.0', 9997)
NUM_OF_SOCKETS = 1
# MAX_BUF_SIZE = 8192


rospy.init_node('rishik_socketstreamer', anonymous=True)

JOINT_STATE_TOPIC = '/my_gen3/joint_states'

# COLOR_CAMERA_TOPIC = '/my_gen3/camera/color/image_raw'    # For Sim Arm - color
COLOR_CAMERA_TOPIC = '/camera/color/image_raw'              # For Real Arm

# DEPTH_CAMERA_TOPIC = '/my_gen3/camera/depth/image_raw'    # For Sim Arm - depth
DEPTH_CAMERA_TOPIC = '/camera/depth/image'                  # For Real Arm

video_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# video_client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, int(10e6))

depth_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# depth_client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, int(10e6))

client_addr_rgb = None      # for RGB stream
client_addr_depth = None    # for Depth stream
data_client_socket = None   # for data stream
isRunning = True



def sendVideoFrame(msg):
    bridgeObj = CvBridge()
    convertedFrame = bridgeObj.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    convertedFrame = cv.resize(convertedFrame, (320, 240))
    # cv.putText(convertedFrame, str(int(time())), (530,465), cv.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, cv.LINE_AA)
    # cv.imshow("preview", convertedFrame)
    (_, buf) = cv.imencode(".jpg", convertedFrame, [int(cv.IMWRITE_JPEG_QUALITY), 35])
    imBytes = pickle.dumps(buf)
    # print(sys.getsizeof(imBytes))

    try:
        video_client_socket.sendto((imBytes), client_addr_rgb)
    except Exception as e:
        print(e)
        print("Video send fail")
        video_client_socket.close()
        rospy.signal_shutdown('')



def sendDepthFrame(msg):
    bridgeObj = CvBridge()
    convertedFrame = bridgeObj.imgmsg_to_cv2(msg, desired_encoding='32FC1')
    convertedFrame = np.array(convertedFrame, dtype=np.float32)
    cv.normalize(convertedFrame, convertedFrame, 0, 1, cv.NORM_MINMAX)
    convertedFrame = (convertedFrame * 255).round().astype(np.uint8)

    (_, convertedFrame) = cv.imencode(".jpg", convertedFrame, [int(cv.IMWRITE_JPEG_QUALITY), 50])
    # print(convertedFrame)
    depthImgBytes = pickle.dumps(convertedFrame)

    try:
        depth_client_socket.sendto((depthImgBytes), client_addr_depth)
    except Exception as e:
        print(e)
        print("Depth video send fail")
        depth_client_socket.close()
        rospy.signal_shutdown('')

    """ 
    # (_, buf) = cv.imencode(".jpg", convertedFrame, [int(cv.IMWRITE_JPEG_QUALITY), 50])
    buf = convertedFrame.tobytes()
    buf_size = len(buf)
    num_of_packs = 1
    print("len:", buf_size)

    if buf_size > MAX_BUF_SIZE:
        num_of_packs = int(math.ceil(buf_size/MAX_BUF_SIZE))
    
    frame_info = {"packs": num_of_packs}
    print(frame_info)

    try:
        depth_client_socket.sendto(pickle.dumps(frame_info), client_addr_depth)
        left = 0
        right = MAX_BUF_SIZE

        for i in range(0, num_of_packs):
            # print("left:", left)
            # print("right:", right)

            # truncate data to send
            data = buf[left:right]
            left = right
            right += MAX_BUF_SIZE

            # send the frames accordingly
            depth_client_socket.sendto(data, client_addr_depth)
    """



def sendStateData(msg):
    global data_client_socket
    data = {
        "name": msg.name,
        "pos": msg.position,
        "vel": msg.velocity
    }
    # print(data)
    try:
        data_pickelised = json.dumps(data)
        # data_pickelised = pickle.dumps(data)
        print(sys.getsizeof(data_pickelised))
        check = data_client_socket.send(data_pickelised)
        if(check != None):
            print("Data send incomplete/failed")

    except Exception as e:
        print(e)
        print('Client unavailable')
        data_client_socket.close()
        rospy.signal_shutdown('')



connectedSocketCount = 0
def spinRos():
    # only spin ros when all sockets are connected
    global connectedSocketCount

    connectedSocketCount += 1
    print('\n%d/%d links connected' % (connectedSocketCount, NUM_OF_SOCKETS))

    if(connectedSocketCount == NUM_OF_SOCKETS):
        print('---ALL LINKS CONNECTED---')
        rospy.spin()



def initVideoLink():
    global client_addr_rgb
    print('\nInit video link...')

    video_client_socket.bind(RGB_HOST_ADDR)
    print('Video link awaiting client at %s:%d' % (RGB_HOST_ADDR[0], RGB_HOST_ADDR[1]))

    (msg, addr) = video_client_socket.recvfrom(1024)
    if(msg.decode('utf-8') == 'namaste'):
        client_addr_rgb = addr
        print('Video link client connected %s' % client_addr_rgb[0])
        rospy.Subscriber(COLOR_CAMERA_TOPIC, Image, sendVideoFrame)
        spinRos()



def initDepthVideoLink():
    global client_addr_depth
    print('\nInit depth video link...')

    depth_client_socket.bind(DEPTH_HOST_ADDR)
    print('Depth link awaiting client at %s:%d' % (DEPTH_HOST_ADDR[0], DEPTH_HOST_ADDR[1]))

    (msg, addr) = depth_client_socket.recvfrom(1024)
    if(msg.decode('utf-8') == 'namaste'):
        client_addr_depth = addr
        print('Depth link client connected %s' % client_addr_depth[0])
        rospy.Subscriber(DEPTH_CAMERA_TOPIC, Image, sendDepthFrame)
        spinRos()



def initDataLink():
    global data_client_socket
    print('\nInit data link...')
    rcvmsg_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #TCP

    try:
        rcvmsg_socket.bind(DATA_HOST_ADDR)
        rcvmsg_socket.listen(5)
        print('Data link listening at %s:%d' % (DATA_HOST_ADDR[0], DATA_HOST_ADDR[1]))

        (data_client_socket, addr) = rcvmsg_socket.accept()
        if data_client_socket:
            print('Data link client connected %s' % addr[0])
            rospy.Subscriber(JOINT_STATE_TOPIC, JointState, sendStateData)
            spinRos()
            
    except Exception as e:
        print(e)
        print('Data link init failed!')
        


def cleanQuit():
    global isRunning
    print('Quitting...')
    video_client_socket.close()
    depth_client_socket.close()
    data_client_socket.close()
    rospy.signal_shutdown('')
    isRunning = False
    return None



try:
    print("Launching...")
    vidLinkThread = threading.Thread(target=initVideoLink, args=())
    depthLinkThread = threading.Thread(target=initDepthVideoLink, args=())
    dataLinkThread = threading.Thread(target=initDataLink, args=())

    # vidLinkThread.start()
    depthLinkThread.start()
    # dataLinkThread.start()
    
except KeyboardInterrupt:
    cleanQuit()


# initVideoLink()
# initDepthVideoLink()
# initDataLink()

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

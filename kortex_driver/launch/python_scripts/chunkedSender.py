#!/usr/bin/env python

'''
This script starts the TCP server to stream RGB, Depth image and joint states to client.
'''

import rospy, message_filters, json, pickle, socket, sys
# import threading

from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2 as cv    # opencv version 3.4.0.14
import numpy as np



DATA_HOST_ADDR = ('0.0.0.0', 9998)
CHUNK_SIZE = 1024
NUM_OF_SKIPS = 40 # 1Hz
ENCODING_FORMAT = 'utf-8'
PACK_FLAG = '[PACKET]'
CV_BRIDGE = CvBridge()

JOINT_STATE_TOPIC = '/my_gen3/joint_states'

COLOR_CAMERA_TOPIC = '/my_gen3/camera/color/image_raw'    # For Sim Arm - color
#COLOR_CAMERA_TOPIC = '/camera/color/image_raw'              # For Real Arm

DEPTH_CAMERA_TOPIC = '/my_gen3/camera/depth/image_raw'    # For Sim Arm - depth
#DEPTH_CAMERA_TOPIC = '/camera/depth/image'                  # For Real Arm

rcvmsg_socket = None
client_addr_rgb = None      # for RGB stream
client_addr_depth = None    # for Depth stream
data_client_socket = None   # for data stream
skipCounter = 0
isRunning = True


def cleanQuit():
    global isRunning
    print('Quitting...')
    isRunning = False
    data_client_socket.shutdown(socket.SHUT_RDWR)
    data_client_socket.close()
    rcvmsg_socket.shutdown(socket.SHUT_RDWR)
    rcvmsg_socket.close()
    rospy.signal_shutdown('')
    sys.exit()
    
    
def mergeImages(rgbImage, depthImage):
    # combines both to create (320, 240, 4) image
    rgbImage = CV_BRIDGE.imgmsg_to_cv2(rgbImage, desired_encoding='bgr8')
    rgbImage =  cv.resize(rgbImage, (320, 240))
    
    # normalise depth image - may lose accuracy
    depthImage = CV_BRIDGE.imgmsg_to_cv2(depthImage, desired_encoding='32FC1')
    depthImage = np.array(depthImage, dtype=np.float32)
    #cv.normalize(depthImage, depthImage, 0, 1, cv.NORM_MINMAX)
    #depthImage = (depthImage * 255).round().astype(np.uint8)
    depthImage = cv.resize(depthImage, (426, 240)) # maintains 16:9 ratio, but match height with rgb
    
    # crop and align with rgb image
    x_offset = 0
    y_offset = 0
    crop_margin = int((depthImage.shape[1] - rgbImage.shape[1]) / 2) # extracts width diff and calculates center
    depthImage = depthImage[:, (crop_margin + x_offset) : (depthImage.shape[1] - crop_margin + x_offset)]
    
    depth_aligned = np.zeros(shape=(depthImage.shape[0], depthImage.shape[1]), dtype=np.float32)
    depth_aligned[0 : depth_aligned.shape[0] - y_offset, :] = depthImage[y_offset : depth_aligned.shape[0], :]

    #rgbd = np.zeros(shape=(depthImage.shape[0], depthImage.shape[1], 4), dtype=np.uint8)
    #rgbd[:, : , 0:3] = rgbImage
    #rgbd[0 : rgbd.shape[0] - y_offset, :, 3] = depthImage[y_offset : rgbd.shape[0], :]
    
    return (rgbImage, depth_aligned) # rgb as uint8, depth as float32


def sendStateData(jointState, rgbImage, depthImage):
    rgbdImage = mergeImages(rgbImage, depthImage)
    
    data = {
        "name": jointState.name,
        "pos": jointState.position,
        "vel": jointState.velocity,
        'rgbd': pickle.dumps(rgbdImage)
    }
    # print(data)
    try:
        data_packed = json.dumps(data).encode(encoding=ENCODING_FORMAT)
        # data_packed = pickle.dumps(data)
        data_size = len(data_packed)
        print('Data size: %d' % data_size)
        
        #send data size msg
        length_msg = (PACK_FLAG + str(data_size)).encode(encoding=ENCODING_FORMAT)
        length_msg += b'.' * (CHUNK_SIZE - len(length_msg)) # add padding
        print('Length discriptor size: %d' % len(length_msg))
        sent = data_client_socket.sendall(length_msg)
        if(sent == 0):
            raise Exception("connection broken")
        # print('Sent length msg of size: %d' % len(length_msg))
        
        #send data chunks
        print('Sending %d chunks...' % int(data_size/CHUNK_SIZE))
        bytes_sent = 0
        while (bytes_sent < data_size):
            chunk = data_packed[bytes_sent : bytes_sent+CHUNK_SIZE]
            sent = data_client_socket.sendall(chunk)
            # print(chunk)
            # print('Chunks sent %d / %d' % (bytes_sent, data_size))
            if (sent != None):
                raise Exception("connection broken")
            bytes_sent += CHUNK_SIZE
            
        print('-- Data sent --')
        
    except Exception as e:
        print(e)
        print('Client unavailable')
        cleanQuit()
        
    except KeyboardInterrupt:
        print('Interrupted')
        cleanQuit()



def cbHandler(*args):
    global skipCounter
    skipCounter += 1
    if(skipCounter == NUM_OF_SKIPS):
        print('--- Subsampled Synced Frame Ready ---')
        skipCounter = 0
        try:
            sendStateData(*args)
        except Exception or KeyboardInterrupt as e:
            print(e)
            cleanQuit()
        
    return None


""" 
def listenForSrvCmd():
    global isRunning, data_client_socket
    print('Listening srv cmds...')
    while isRunning:
        msg = data_client_socket.recv(1024)
        if(str(msg).strip() == 'quit'):
            print('> QUIT RCVD <')
            break

    print('Deafened srv cmnds!')
    cleanQuit()
 """


def initDataLink():
    global data_client_socket, rcvmsg_socket
    print('\nInit data link...')
    rcvmsg_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #TCP
    rcvmsg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        rcvmsg_socket.bind(DATA_HOST_ADDR)
        rcvmsg_socket.listen(5)
        print('Data link listening at %s:%d' % (DATA_HOST_ADDR[0], DATA_HOST_ADDR[1]))

        (data_client_socket, addr) = rcvmsg_socket.accept()
        if data_client_socket:
            print('Data link client connected: %s' % addr[0])
            # threading.Thread(target=listenForSrvCmd).start()
            # rospy.Subscriber(JOINT_STATE_TOPIC, JointState, cbHandler)
            image_sub = message_filters.Subscriber(COLOR_CAMERA_TOPIC, Image)
            depth_sub = message_filters.Subscriber(DEPTH_CAMERA_TOPIC, Image)
            jointstate_sub = message_filters.Subscriber(JOINT_STATE_TOPIC, JointState)
            rosTimeSync = message_filters.ApproximateTimeSynchronizer([jointstate_sub, image_sub, depth_sub], queue_size=10, slop=1)
            rosTimeSync.registerCallback(cbHandler)
            rospy.spin()
            
    except Exception or KeyboardInterrupt as e:
        print(e)
        print('Data link init failed!')
        
    except:
        print('Some error occurred while init!')
    
    finally:
        cleanQuit()


        
if __name__ == '__main__':
    print("Launching...")
    print("Using python: " + sys.version)
    rospy.init_node('rishik_socketstreamer', anonymous=True)
    initDataLink()

#!/usr/bin/env python

import rospy, message_filters, sys
import numpy as np
import cv2 as cv

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge, CvBridgeError

class syncTest:
    
    JOINT_STATE_TOPIC = '/my_gen3/joint_states'
    COLOR_CAMINFO_TOPIC = '/camera/color/camera_info'
    COLOR_CAMERA_TOPIC = '/camera/color/image_raw'
    DEPTH_CAMINFO_TOPIC = '/camera/depth/camera_info'
    DEPTH_CAMERA_TOPIC = '/camera/depth/image'
    
    SUBSAMPLE_SKIP_FRAMES = 5
    subsample_counter = 0
    
    def __init__(self):
        self.bridge = CvBridge()

        # self.color_info_sub = message_filters.Subscriber(self.COLOR_CAMINFO_TOPIC, CameraInfo)
        # self.depth_info_sub = message_filters.Subscriber(self.DEPTH_CAMINFO_TOPIC, CameraInfo)
                
        self.jointstate_sub = message_filters.Subscriber(self.JOINT_STATE_TOPIC, JointState)
        self.image_sub = message_filters.Subscriber(self.COLOR_CAMERA_TOPIC, Image)
        self.depth_sub = message_filters.Subscriber(self.DEPTH_CAMERA_TOPIC, Image)
        
        cv.namedWindow('rgb', cv.WINDOW_AUTOSIZE)
        cv.namedWindow('depth', cv.WINDOW_AUTOSIZE)
            
        self.ts = message_filters.ApproximateTimeSynchronizer([self.jointstate_sub, self.image_sub, self.depth_sub], queue_size=10, slop=1)
        self.ts.registerCallback(self.cbHandler)
        
        
    def cbHandler(self, joint_state, rgb_data, depth_data):
        self.subsample_counter += 1
        # print('synced frame')
        
        if(self.subsample_counter == self.SUBSAMPLE_SKIP_FRAMES):
            self.subsample_counter = 0
            try:
                rgb_data = self.bridge.imgmsg_to_cv2(rgb_data, desired_encoding='bgr8')
                rgb_data = cv.resize(rgb_data, (320, 240))
                
                depth_data = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='32FC1')
                depth_data = np.array(depth_data, dtype=np.float32)
                cv.normalize(depth_data, depth_data, 0, 1, cv.NORM_MINMAX)
                depth_data = (depth_data * 255).round().astype(np.uint8)
                depth_data = cv.resize(depth_data, (426, 240)) # maintains 16:9 ratio, but match height with rgb
                
                # crop to match rgb resolution
                x_offset = 20
                y_offset = 30
                crop_margin = int((depth_data.shape[1] - rgb_data.shape[1]) / 2) # extracts width diff and calculates center
                depth_data = depth_data[:, (crop_margin + x_offset) : (depth_data.shape[1] - crop_margin + x_offset)]
                
                depth_aligned = np.zeros(shape=depth_data.shape, dtype=np.uint8)
                depth_aligned[0 : depth_aligned.shape[0] - y_offset, : ] = depth_data[y_offset : depth_aligned.shape[0], :] 
                
                #overlaying depth on rgb for visualisation purpose only
                depth_data = cv.bitwise_not(depth_aligned) #invert image
                
                depth_overlayed = 0.25 * rgb_data.copy()
                depth_overlayed[:,:,0] = depth_overlayed[:,:,0] + (10 * depth_data)
                depth_overlayed[:,:,2] = depth_overlayed[:,:,2] + (10 * depth_data)
                depth_overlayed = depth_overlayed.round().astype(np.uint8)
                # cv.normalize(depth_overlayed, depth_overlayed, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8UC1)
                # print(depth_overlayed)
                
                # depth_3ch_img = np.zeros(shape=(depth_data.shape[0], depth_data.shape[1], 3), dtype=np.uint8)
                # depth_3ch_img[:,:,0] = depth_data
                # depth_3ch_img[:,:,1] = depth_data
                # depth_3ch_img[:,:,2] = depth_data
                # print(depth_3ch_img.shape)

                
                # print(joint_state)
                cv.imshow('rgb', rgb_data)
                cv.imshow('depth', depth_overlayed)
                
                if cv.waitKey(30) == 27: #ESC
                    rospy.signal_shutdown('')
                    print("Terminating...")
            
            except CvBridgeError as e:
                print(e)



def main():
    print("Launching...")
    rospy.init_node('rishik_synchroniser', anonymous=True)
    st = syncTest()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Terminating...')


        
if __name__ == '__main__':
    main()

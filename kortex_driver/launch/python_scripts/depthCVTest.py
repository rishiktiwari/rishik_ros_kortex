#!/usr/bin/env python

# THIS PROGRAM USES PYTHON 2
import rospy, sys, pickle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv    # opencv version 3.4.0.14
import numpy as np

print("Using python: " + sys.version)

# DEPTH_CAMERA_TOPIC = '/my_gen3/camera/depth/image_raw'    # For Sim Arm - depth
DEPTH_CAMERA_TOPIC = '/camera/depth/image'                  # For Real Arm

def imageCallback(msg):
    global cvVw
    try:
        bridgeObj = CvBridge()
        convertedFrame = bridgeObj.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        
        depth_array = np.array(convertedFrame, dtype=np.float32)
        print(depth_array[135][240])

        # distance extraction, not very accurate, should probably use point cloud info
        # print('Raw: %.4f, Scaled: %.1f cm' % (depth_array[120][160], depth_array[120][160]*63.492))
        
        cv.normalize(depth_array, depth_array, 0, 255, cv.NORM_MINMAX)
        depth_data = depth_array.round().astype(np.uint8)

        # crop and align to match rgb image
        depth_data = cv.resize(depth_data, (426, 240))
        x_offset = 20
        y_offset = 30
        crop_margin = int((depth_data.shape[1] - 320) / 2) # extracts width diff and calculates center
        depth_data = depth_data[:, (crop_margin + x_offset) : (depth_data.shape[1] - crop_margin + x_offset)]
        
        depth_aligned = np.zeros(shape=depth_data.shape, dtype=np.uint8)
        depth_aligned[0 : depth_aligned.shape[0] - y_offset, : ] = depth_data[y_offset : depth_aligned.shape[0], :]
        
        # print(depth_aligned.shape)
        #cvVw.write(depth_aligned)
        
        # not conversion to 3-channel required
        # cv_depth = np.zeros(shape=(depth_data.shape[0], depth_data.shape[1], 3), dtype=np.uint8)
        # cv_depth[:,:,0] = depth_data
        # cv_depth[:,:,1] = depth_data
        # cv_depth[:,:,2] = depth_data
        # print(cv_depth[0])

        # (_, depth_jpg) = cv.imencode(".jpg", depth_data, [int(cv.IMWRITE_JPEG_QUALITY), 50])
        """ 
        # enable the following to imshow the JPG.
        img_string = depth_jpg.tostring()
        npimg = np.fromstring(img_string, dtype=np.uint8)
        img = cv.imdecode(npimg, 1)
        """
        # print(sys.getsizeof(depth_data))
        # print(sys.getsizeof(pickle.dumps(depth_jpg)))

    except (CvBridgeError, Exception, KeyboardInterrupt) as e:
        print(e)
        rospy.signal_shutdown('')

    cv.imshow("preview", depth_aligned)
    if cv.waitKey(2) == 27:
        rospy.signal_shutdown('')
    

if __name__ == '__main__':
    global cvVw
    videoExportDirectory = '/media/rishik/Installs/Kinova Recordings/'
    videoFileName = 'pickPlaceTask_ball_zdiff--depth'

    print("starting...")
    #videoExportPath = videoExportDirectory + videoFileName + '.mp4'
    #fourcc = cv.VideoWriter_fourcc(*'mp4v')
    #cvVw = cv.VideoWriter(videoExportPath, fourcc, 30.0, (320, 240), False)

    rospy.init_node('rishik_depthCVTest', anonymous=True)
    rospy.Subscriber(DEPTH_CAMERA_TOPIC, Image, imageCallback)
    rospy.spin()

print('quitting...')
#cvVw.release()
cv.destroyAllWindows()

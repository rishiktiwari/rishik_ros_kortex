#!/usr/bin/env python

# THIS PROGRAM USES PYTHON 2
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv    # opencv version 3.4.0.14
import numpy as np
import matplotlib.pyplot as plt

print("Using python: " + sys.version)

# DEPTH_CAMERA_TOPIC = '/my_gen3/camera/depth/image_raw'    # For Sim Arm - depth
DEPTH_CAMERA_TOPIC = '/camera/depth/image'                  # For Real Arm

SAMPLE_SIZE = 30
TEST_FRAME_OFFSET = np.random.randint(1, 60)
frameSum = np.zeros((270,480), dtype=np.float32)
frameAvg = np.zeros((270,480), dtype=np.float32)
diffFrame = np.zeros((270,480), dtype=np.float32)
frameCount = 0


def imageCallback(msg):
    global frameSum, frameAvg, diffFrame, frameCount
    try:
        bridgeObj = CvBridge()
        depthImage = bridgeObj.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        depthImage = np.array(depthImage, dtype=np.float32)
        depthImage += np.nan_to_num(depthImage) * 1000 # replace nans and convert from m to mm

        frameSum += depthImage
        frameCount += 1

        if(frameCount == SAMPLE_SIZE):
            print("SAMPLING DONE of %d frames" % SAMPLE_SIZE)
            frameAvg = frameSum / float(SAMPLE_SIZE)
            frameSum = np.zeros((270,480), dtype=np.float32)

        if(frameCount == SAMPLE_SIZE + TEST_FRAME_OFFSET):
            print("DIFFING")
            diffFrame = np.nan_to_num(frameAvg - depthImage)
            # print(diffFrame)
            cv.imshow("diff", diffFrame)
            rospy.signal_shutdown('')

    except (CvBridgeError, Exception, KeyboardInterrupt) as e:
        print(e)
        rospy.signal_shutdown('')

    cv.imshow("preview", depthImage)
    if cv.waitKey(2) == 27:
        rospy.signal_shutdown('')
    


print("starting...")
print("TEST_FRAME_OFFSET: %d" % TEST_FRAME_OFFSET)
rospy.init_node('rishik_depthNoiseTest', anonymous=True)
rospy.Subscriber(DEPTH_CAMERA_TOPIC, Image, imageCallback)
rospy.spin()

fig, ax = plt.subplots(figsize=(8,4))
c = ax.pcolormesh(diffFrame, cmap='viridis', vmin=-5.0, vmax=5.0)
ax.set_title('Difference Map. min: %.2f, max: %.2fmm' % (np.nanmin(diffFrame), np.nanmax(diffFrame)))
# set the limits of the plot to the limits of the data
ax.axis([0, diffFrame.shape[1], 0, diffFrame.shape[0]])
cb = fig.colorbar(c, ax=ax)
cb.set_label('Difference in mm')
plt.xlabel('Width px')
plt.ylabel('Height px')
plt.show()

fig, ax = plt.subplots(figsize=(8,4))
c = ax.pcolormesh(diffFrame, cmap='viridis', vmin=-250.0, vmax=250.0)
ax.set_title('Difference Map. min: %.2f, max: %.2fmm' % (np.nanmin(diffFrame), np.nanmax(diffFrame)))
# set the limits of the plot to the limits of the data
ax.axis([0, diffFrame.shape[1], 0, diffFrame.shape[0]])
cb = fig.colorbar(c, ax=ax)
cb.set_label('Difference in mm')
plt.xlabel('Width px')
plt.ylabel('Height px')
plt.show()

try: inp = input('press enter to exit')
except: pass
print('quitting...')
cv.destroyAllWindows()
exit(1)

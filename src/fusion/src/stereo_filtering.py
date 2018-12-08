#! /usr/bin/env python

import sys
import cv2
import rospy
import message_filters
import numpy as np
import seaborn as sns; sns.set()
import matplotlib.pyplot as plt
import scipy.ndimage.filters as filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from skimage.transform import rescale

def depth_filter(depth_data, lidar_scan): # lidar_scan
    rospy.loginfo("IN PROCESSING LOOP")
    intensities = lidar_scan.intensities
    ranges = lidar_scan.ranges

    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(depth_data, depth_data.encoding
                ).astype(np.float32)
    except CvBridgeError as e:
        rospy.logerr(e)
    rospy.loginfo(depth_image)

    black = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), np.uint8)
    ret, thresh = cv.threshold(depth_image, 140, 255, cv.THRESH_BINARY)
    _, contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL,
                                    cv.CHAIN_APPROX_NONE)
    cv.drawContours(black, contours, -1, (255, 255, 255), thickness= -1)
    #c = max(contours, key = cv.contourArea)
    self.mask = cv.blur(black, (50, 50))
    #plt.imshow(self.mask)
    #plt.show()


    #cv2.imwrite("matching_image.png", depth_image)

def listen():
    rospy.init_node('calibration')
    rospy.loginfo("Listening...")

    image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
    lidar_sub = message_filters.Subscriber('/lidar/scan', LaserScan)

    time_sync = message_filters.ApproximateTimeSynchronizer([image_sub, lidar_sub], 5, 0.01)
    time_sync.registerCallback(depth_filter)
    
    rospy.spin()

if __name__ == '__main__':
    listen()
    plt.show()

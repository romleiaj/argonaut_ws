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

def calibrate(depth_data, lidar_scan): # lidar_scan
    global avg_heatmap, i
    rospy.loginfo("IN PROCESSING LOOP")
    intensities = lidar_scan.intensities
    ranges = lidar_scan.ranges

    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(depth_data, depth_data.encoding
                ).astype(np.float32)
    except CvBridgeError as e:
        rospy.logerr(e)

    #cv2.imwrite("matching_image.png", depth_image)

    li_in_px = 2520
    #depth_image = cv2.imread(str(depth_image))[:,:,0].astype(np.float32)
    sns.heatmap(depth_image)
    plt.show()
    scan_width = 91

    imY, imX = depth_image.shape[:2] # image resolution
    filtered_image = filters.gaussian_filter(depth_image, 2, 0)
    #sns.heatmap(filtered_image)
    #plt.show()
    
    for m in range(imY):
        filtered_image[m, :] = np.divide(filtered_image[m, :], 
                np.nanmax(filtered_image[m, :]) + np.finfo(float).eps)
    #sns.heatmap(filtered_image, cmap="jet")
    #plt.show()

    #lidar_scan = lidar_scan.reshape(1, imX)
    #avg_lidar_scan = []
    #for l in range(scan_width):
        #avg_lidar_scan.append(np.mean(lidar_scan[0][(l*7):(l*7+7)]))

    #np.savetxt("ranges.txt", lidar_scan.ranges, delimiter=",")
    #np.savetxt("intensities.txt", lidar_scan.intensities, delimiter=",")
    franges = filters.gaussian_filter(ranges, 2, 0)
    fintensities = filters.gaussian_filter(intensities, 2, 0)
    scaled_ranges = franges * fintensities
    intenWrap = np.append(scaled_ranges, scaled_ranges)
    print(intenWrap.shape)
    scaled_lidar = rescale(np.array(scaled_ranges), (imX)/(scan_width), 
            multichannel=False, anti_aliasing=True, mode='constant')
    for n in range(li_in_px - len(scaled_lidar)):
       scaled_lidar = np.append(scaled_lidar, [0.0])
    scaled_lidar = scaled_lidar.reshape(1,li_in_px)
    scaled_lidar = filters.gaussian_filter(scaled_lidar, 2, 0)

    #full_scan = np.concatenate((np.random.rand(1,imX),
        #normalized_lidar, np.random.rand(1,imX)), axis=1)

    result = np.zeros((scaled_lidar.shape[1] - imX, imY))
    for j in range(scaled_lidar.shape[1] - imX):
        for k in range(imY):
            narrow_scan = scaled_lidar[0][j:imX+j].astype(np.float32)
            normalized_scan = narrow_scan/np.max(narrow_scan)
            filtered_img_row = filtered_image[k, :].astype(np.float32)
            result[j][k] = np.nansum((normalized_scan - filtered_img_row)**2, axis=0)

    avg_heatmap = (avg_heatmap + result) / 2.0
    i += 1
    rospy.loginfo("Iteration %s finished." % i)

def listen():
    rospy.init_node('calibration')
    rospy.loginfo("Listening...")

    image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
    lidar_sub = message_filters.Subscriber('/lidar/scan', LaserScan)

    time_sync = message_filters.ApproximateTimeSynchronizer([image_sub, lidar_sub], 5, 0.01)
    time_sync.registerCallback(calibrate)
    
    rospy.spin()

def find_min_idx(x):
    w = np.unravel_index(np.argmin(x, axis=None), x.shape)
    return w

if __name__ == '__main__':
    i = 0
    avg_heatmap = np.zeros((1880, 480))
    #image = sys.argv[1]
    #for l in range(5):
        #calibrate(image)
        #i += 1
    listen()

    x, y = find_min_idx(avg_heatmap)
    x1 = int(x/360. - 91/2.)
    x2= int(x/360. + 91/2.)
    if x1 < 0:
        x1 += 360

    rospy.loginfo("Laser scan matches from index %s to %s." % (x1, x2))
    rospy.loginfo("Pixel row of matching laser scan: %s." % y)
    sns.heatmap(avg_heatmap, cmap="jet")
    plt.show()

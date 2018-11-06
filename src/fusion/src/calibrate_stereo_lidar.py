#! /usr/bin/env python

import sys
import cv2
import rospy
import message_filters
import numpy as np
import seaborn as sns; sns.set()
import matplotlib.pyplot as plt
import scipy.ndimage.filters as filters
from skimage.transform import rescale

def calibrate(depth_image): # lidar_scan
    global avg_heatmap, i
    rospy.loginfo("Reading image.")
    depth_image = cv2.imread(str(depth_image))[:,:,0].astype(np.float32)
    #sns.heatmap(depth_image)
    #plt.show()
    scan_width = 91

    imY, imX = depth_image.shape[:2] # image resolution
    filtered_image = filters.gaussian_filter(depth_image, 2, 0)
    #sns.heatmap(filtered_image)
    #plt.show()
    
    for m in range(imY):
        filtered_image[m, :] = np.divide(filtered_image[m, :], 
                np.nanmax(filtered_image[m, :]) + np.finfo(float).eps)

    lidar_scan = filtered_image[400, :].reshape(1, imX)
    avg_lidar_scan = []
    for l in range(scan_width):
        avg_lidar_scan.append(np.mean(lidar_scan[0][(l*7):(l*7+7)]))

    scaled_lidar = rescale(np.array(avg_lidar_scan), (imX)/(scan_width), 
            multichannel=False, anti_aliasing=True, mode='constant')
    normalized_lidar = scaled_lidar/np.max(scaled_lidar)
    for n in range(imX - len(normalized_lidar)):
       normalized_lidar = np.append(normalized_lidar, [0.0])
    normalized_lidar = normalized_lidar.reshape(1,imX)

    full_scan = np.concatenate((np.random.rand(1,imX),
        normalized_lidar, np.random.rand(1,imX)), axis=1)

    result = np.zeros((2*imX, imY))
    for j in range(full_scan.shape[1] - imX):
        for k in range(imY):
            narrow_scan = full_scan[0][j:imX+j].astype(np.float32)
            filtered_img_row = filtered_image[k, :].astype(np.float32)
            result[j][k] = np.nansum((narrow_scan - filtered_img_row)**2, axis=0)

    avg_heatmap = (avg_heatmap + result) / 2.0
    print("Iteration %s finished." % (i + 1))

def listen():
    rospy.init_node('calibration')

    image_sub = message_filter.Subscriber('topic', topic_type)
    lidar_sub = message_filter.Subscriber('topic', topic_type)

    time_sync = message_filters.TimeSynchronizer([image_sub, lidar_sub], 10)
    time_sync.registerCallback(calibrate)
    
    rospy.spin()

def find_min_idx(x):
    w = np.unravel_index(np.argmin(x, axis=None), x.shape)
    return w

if __name__ == '__main__':
    i = 0
    avg_heatmap = np.zeros((2*640, 480))
    image = sys.argv[1]
    for l in range(5):
        calibrate(image)
        i += 1

    x, y = find_min_idx(avg_heatmap)
    x1 = int(x/360. - 91/2.)
    x2= int(x/360. + 91/2.)
    if x1 < 0:
        x1 += 360

    print("Laser scan matches from index %s to %s." % (x1, x2))
    print("Pixel row of matching laser scan: %s." % y)
    sns.heatmap(avg_heatmap, cmap="jet")
    plt.show()

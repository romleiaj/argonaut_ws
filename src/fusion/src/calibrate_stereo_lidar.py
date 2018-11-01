#! /usr/bin/env python

import sys
import numpy as np
#import matplotlib.pyplot as plt
import scipy.ndimage.filters as filters
from scipy.interpolate import interp1d
import scipy.misc.imresize as imresize
import rospy


def get_lidar_scan():
    pass

def get_depth_image():
    pass

def calibrate(depth_image): # lidar_scan
    scan_width = 91

    imX, imY = depth_image.size # image resolution
    filtered_image = filters.gaussian_filter(depth_image, 2, 0)
    
    for i in range(imX):
        filtered_image[i, :] = filtered_image[i, :]/(max(filtered_image[i, :]))

    lidar_scan = filtered_image[360, :]
    avg_lidar_scan = []
    for i in range(scan_width):
        avg_lidar_scan.append(np.mean(lidar_scan[(i*7):(i*7+7)]))

    scaled_lidar = interp1d(range(scan_width), lidar_scan)
    scaled_lidar = imresize(scaled_lidar, (1, imX))
    scaled_lidar = np.array(scaled_lidar/(max(scaled_lidar)))

    full_scan = np.concatenate(np.zeros(imgX-1), scaled_lidar, np.zeros(imgX-1))

    result = [[]]
    for i, pt in enumerate(full_scan-imX):
        for j in range(imY):
            narrow_scan = full_scan[i:imX+i]
            filtered_image_row = float(filtered_image[j, :])
            result[j,i] = sum((narrow_scan - filtered_img_row)**2)

    print(result)
    #plt.plot(result)
    #plt.show()

        



def listen():

    rospy.init_node('fusion', anonymous=True)

    rospy.Subscriber('topic', topic_type, callback)
    rospy.Subscriber('topic', topic_type, callback)

    rospy.spin()

if __name__ == '__main__':
    image = sys.argv[1]
    calibrate(image)

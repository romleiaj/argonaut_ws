#! /usr/bin/env python

import Queue
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import LaserScan

class Motion():
    def __init__(self):
        self.scan_q = Queue.Queue(maxsize=20)
 
    def motion_detection(self, lidar_scan):
        self.scan_q.put(lidar_scan)
        if self.scan_q.qsize() > 2:
            first_scan = self.scan_q.get()
            normalized_first_scan = scan.intensities * scan.ranges
            second_scan = self.scan_q.get()
            normalized_second_scan = scan.ranges * scan.intensities
            ss_diff = (normalized_second_scan - normalized_first_scan)**2
            plt.plot(ss_diff)
            plt.show
        
    def listen(self):
        rospy.init_node('calibration')
        rospy.loginfo("Listening...")

        lidar_sub = rospy.Subscriber('/lidar/scan', LaserScan, self.motion_detection)
    
        rospy.spin()

if __name__ == '__main__':
    motion = Motion()
    motion.listen()

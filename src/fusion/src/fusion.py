#! /usr/bin/env python

import rospy

def listen():

    rospy.init_node('fusion', anonymous=True)

    rospy.Subscriber('topic', topic_type, callback)

    rospy.spin()

if __name__ == '__main__':
    listen()

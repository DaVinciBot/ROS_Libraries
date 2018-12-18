#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Publish a video as ROS messages.
"""

import argparse

import numpy as np

import cv2

import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def main():
    rospy.init_node("video_publisher", anonymous=True)
    img_pub = rospy.Publisher("/camera/image_raw", Image,
                              queue_size=10)

    video = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        try:
            # Publish image.
            img_msg = bridge.cv2_to_imgmsg(video, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera"
            img_pub.publish(img_msg)
        except CvBridgeError as err:
            print err

            rate.sleep()





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
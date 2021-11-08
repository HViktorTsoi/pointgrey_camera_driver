#!/usr/bin/env python2
# coding=utf-8

from __future__ import absolute_import, print_function

import random
import sys
import time

import cv2
import matplotlib.pyplot as plt
import message_filters
import numpy as np
import ros_numpy
import rospy
import transforms3d
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, CompressedImage, Image, PointField


def cb_stereo(left_msg, right_msg):
    left = bridge.imgmsg_to_cv2(left_msg, "passthrough")
    right = bridge.imgmsg_to_cv2(right_msg, "passthrough")

    cv2.putText(left, 'Stamp: {}.{:09d}'.format(left_msg.header.stamp.secs, left_msg.header.stamp.nsecs),
                (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(right, 'Stamp: {}.{:09d}'.format(right_msg.header.stamp.secs, right_msg.header.stamp.nsecs),
                (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    img = np.concatenate([left, right], axis=1)
    print(img.shape)
    if random.random() < 0.03:
        cv2.imwrite('/tmp/sync_validator/{}.jpg'.format(time.time()), img)
    # cv2.imshow('stereo', img)
    # cv2.waitKey(1)


def cb_image(img_msg, name):
    print(name)
    # img = cv2.imdecode(np.fromstring(img.data, np.uint8), cv2.IMREAD_COLOR)
    img = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    # img = cv2.resize(img, dsize=(img.shape[1] // 2, img.shape[0] // 2))

    cv2.putText(img, 'Stamp: {}.{:09d}'.format(img_msg.header.stamp.secs, img_msg.header.stamp.nsecs),
                (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow(name, img)
    # if random.random() < 0.05:
    #     cv2.imwrite('/tmp/sync_validator/{}.jpg'.format(time.time()), img)
    # if random.random() < 0.05:
    # cv2.imwrite('/tmp/sync_validator/{}.jpg'.format(time.time()), img)
    cv2.waitKey(1)


if __name__ == '__main__':
    device_id = 0
    rospy.init_node('sync_validator_{}'.format(device_id))
    rospy.loginfo('INIT...')
    bridge = CvBridge()

    sub_left = message_filters.Subscriber('/camera/left/image_raw', Image)
    sub_right = message_filters.Subscriber('/camera/right/image_raw', Image)
    # sub_left = message_filters.Subscriber('/camera/left/image_raw', Image)
    # sub_right = message_filters.Subscriber('/camera/right/image_raw', Image)
    ts = message_filters.TimeSynchronizer([sub_left, sub_right], 10)
    ts.registerCallback(cb_stereo)
    rospy.spin()

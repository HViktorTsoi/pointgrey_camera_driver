#!/usr/bin/env python2
# coding=utf-8
from __future__ import print_function, division, absolute_import

import rospy
from sensor_msgs.msg import TimeReference
import socket
import sys
import base64
import serial
import time
import thread
import datetime
import re

if __name__ == '__main__':
    rospy.init_node('clock_source')
    rospy.loginfo('Pointgery camera sync, clock source publisher started.')

    pub = rospy.Publisher('/imu/trigger_time', TimeReference, queue_size=1)
    # 向外部转发RMC同步消息
    serial_sync = serial.Serial(port='/dev/ttyUSB0', baudrate=460800, bytesize=8, parity='N', stopbits=1,
                                timeout=None, xonxoff=0, rtscts=0)

    while not rospy.is_shutdown():
        if serial_sync.isOpen():

            tic = time.time()
            stamp_return = serial_sync.readline()
            toc = time.time()

            stamp = TimeReference()
            stamp.header.frame_id = 'chronos'
            stamp.header.stamp = rospy.Time.now()

            # 获取下位机传输过来的时间戳
            try:
                secs, microsecs = [float(val) for val in stamp_return.split(':')]
                # print(secs, microsecs)
            except Exception as e:
                print(e, stamp_return)
                continue

            stamp.time_ref.secs = secs
            stamp.time_ref.nsecs = microsecs * 1e3

            pub.publish(stamp)

            if microsecs == 0:
                # 每秒的开始转发GPRMC
                pass
    #
    # while not rospy.is_shutdown():
    #     while True:
    #         stamp = TimeReference()
    #         stamp.header.frame_id = 'chronos'
    #         stamp.header.stamp = rospy.Time.now()
    #
    #         stamp.time_ref = stamp.header.stamp
    #
    #         pub.publish(stamp)
    #         rospy.sleep(1.0 / 30)

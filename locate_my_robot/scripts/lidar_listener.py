#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import absolute_import, division, print_function
import os
import sys
import numpy as np
import h5py
import rospy
import time

from sensor_msgs.msg import LaserScan

class LidarListener(object):
    def __init__(self):
        self.latest_scan_data = rospy.wait_for_message('/my_robot/laser/scan',LaserScan)
        self.lidar_sub = rospy.Subscriber('/my_robot/laser/scan',LaserScan,self.lidar_callback)
        

    def lidar_callback(self, data):
        self.latest_scan_data = data

    def get_latest_scan(self):
        return self.latest_scan_data

def main():
    rospy.init_node('lidar_listener', anonymous=True)
    listener = LidarListener()
    for ii in range(1):
        scan_data = listener.get_latest_scan()
        if scan_data != None:
            print("---")
            print("angle_min = %f, angle_max = %f" % (scan_data.angle_min, scan_data.angle_max))
            print("angle_increment = %f" % scan_data.angle_increment)
            print("num of points = %f" % len(scan_data.ranges))
            print("pts ", scan_data.ranges[340:460] )

            index_list = []
            distance_list = []
            for ii in range(0, len(scan_data.ranges) ):
                y_distance = abs(scan_data.ranges[ii] * np.sin(scan_data.angle_min + scan_data.angle_increment * ii))
                x_distance = abs(scan_data.ranges[ii] * np.cos(scan_data.angle_min + scan_data.angle_increment * ii))
                if y_distance <= 0.55:
                    index_list.append(ii)
                    distance_list.append(x_distance)
            #print(index_list)
            #print(distance_list)
            index_list = np.array(index_list)
            distance_list = np.array(distance_list)
            print(index_list[distance_list < 4])
            print(distance_list[distance_list < 4])

        time.sleep(0.1)

if __name__ == "__main__":
    main()

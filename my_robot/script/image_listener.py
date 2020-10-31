#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import absolute_import, division, print_function
import os
import sys
import numpy as np
import rospy

from sensor_msgs.msg import Image

class ImageListener(object):
    def __init__(self):
        self.latest_image = rospy.wait_for_message('/camera/rgb/image_raw',Image)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.camera_callback)

    def get_image(self):
        return self.latest_image

    def save_img(self,name):
        count = 0
        row_sum = 0
        col_sum = 0
        print(ord(max(self.latest_image.data)))
        for ii in range(int(len(self.latest_image.data) / 3)):
            r = ord(self.latest_image.data[ii * 3])
            g = ord(self.latest_image.data[ii * 3 + 1])
            b = ord(self.latest_image.data[ii * 3 + 2])
            if r == 255 and g == 255 and b == 255:
                count += 1
                col_sum += ii % 800
                row_sum += ii / 800
        print("count == ", count)
        print("(row,col) == (%f, %f) " % (float(row_sum / count), float(col_sum / count)))
        print("width == ",self.latest_image.width)
        print("height == ",self.latest_image.height)
        print("step == ",self.latest_image.step)
        print("encoding == ", self.latest_image.encoding)

    def camera_callback(self,data):
        self.latest_image = data

def main():
    rospy.init_node('image_listener', anonymous=True)
    listener = ImageListener()
    listener.save_img('0.jpg')

if __name__ == "__main__":
    main()




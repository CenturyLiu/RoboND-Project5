#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid

import numpy as np

import matplotlib.pyplot as plt

class Map_listener(object):
    def __init__(self,map_topic):
        self.map_topic = map_topic
        self.map = rospy.wait_for_message(self.map_topic,OccupancyGrid)
        self.map_subscriber =  rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map_listener)
        
    def get_map_listener(self,msg):
        self.map = msg
        
    def get_map(self):
        current_map = np.array(self.map.data)
        width = self.map.info.width
        height = self.map.info.height
        origin = self.map.info.origin
        resolution = self.map.info.resolution
        
        current_map = current_map.reshape((height,width)) # reshape the map
        
        # change the map value
        current_map[current_map > 0] = 1.0
        
        return width,height,origin,resolution,current_map
        
def plot_subplot(submap):
    for ii in range(124):
        for jj in range(124):
            if submap[ii][jj] == -1:
                plt.plot(ii,jj,'ok')
            elif submap[ii][jj] == 0:
                plt.plot(ii,jj,'xb')
            elif submap[ii][jj] > 100:
                plt.plot(ii,jj,'*r')
    plt.show()


if __name__ == "__main__":
    rospy.init_node('map_listener')
    map_listener = Map_listener('/map')
    width,height,origin,resolution,current_map = map_listener.get_map()
    print(origin)
    print(resolution)
    
    map_msg = map_listener.map
    real_map = np.array(map_msg.data).reshape((height,width))
    
    print(len(real_map[real_map > 0]))
    print(len(real_map[real_map == 0]))
    print(len(real_map[real_map == -1]))
    
    submap_list = []
    split_in_row = np.split(current_map, 16) # split the map into 16 parts
    for ii in range(16):
        split_in_col = np.hsplit(split_in_row[ii], 16)
        for submap in split_in_col:
            submap_list.append(submap)
            
    plot_subplot(submap_list[136])
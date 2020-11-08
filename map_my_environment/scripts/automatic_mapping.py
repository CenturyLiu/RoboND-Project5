#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import *

from map_listener import Map_listener
from map_split_detection import check_reacheable_unexplored_part
from odom_listener import OdomListener

import numpy as np
import copy

class submap_center(object):
    def __init__(self,grid_x,grid_y, index):
        self.grid_x = grid_x
        self.grid_y = grid_y
        self.index = index
        self.exploration_num = 0 # count how much time this submap has been explored

class Automatic_mapping(object):
    def __init__(self):
        # Get an action client for movebase
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        # create listener for map and odom
        self.map_listener = Map_listener("/map")
        self.odom_listener = OdomListener()
        
        # initialize the center of submaps in terms of map array
        self.number_of_submaps = 1024#256
        self.number_of_submap_row = 32#16
        self.number_of_submap_col = 32#16
        self.submap_grid_num = 62#124
        self.submap_center_list = []
        self.resolution = 0.0500000007451        
        self.x_offset = -50.0
        self.y_offset = -50.0
        
        # store the explored grid
        self.explored_grid = []
        
        # maximum exploration number for a submap
        self.max_exploration_num = 4
        
        # visualize the next exploration goal
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size = 10)
        
        self.rate = rospy.Rate(20) # 10hz
        
        self.min_unoccupied_tol = 64
        
        # split the submaps, visualize
        for ii in range(self.number_of_submap_row):
            for jj in range(self.number_of_submap_col):
                self.submap_center_list.append(submap_center(self.submap_grid_num / 2 + self.submap_grid_num * ii, self.submap_grid_num / 2 + self.submap_grid_num * jj, ii * self.number_of_submap_row + jj))
                # get the map coordinate of submap center
                grid_x = self.submap_center_list[-1].grid_x
                grid_y = self.submap_center_list[-1].grid_y
                map_x, map_y = self.grid_to_map(grid_x, grid_y)
                
                self.visualize_goal(map_x, map_y, scale = 0.5, marker_id = ii * self.number_of_submap_row + jj, r = 0.2, g = 0.2, b = 0.8)
    
    def visualize_goal(self, goal_x, goal_y, scale = 1.0, marker_id = 0, r = 0.8, g = 0.2, b = 0.2):
        marker = Marker()
        marker.id = marker_id
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CUBE
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = 0.1
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.5
 
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration() # target exists until next target is determined.
        self.marker_pub.publish(marker)
        self.rate.sleep()
    
    def map_to_grid(self,x,y):
        row = int((x - self.x_offset) / self.resolution)
        col = int((y - self.y_offset) / self.resolution)
        return row, col
        
    def grid_to_map(self,grid_x,grid_y):
        x = grid_x * self.resolution + self.x_offset
        y = grid_y * self.resolution + self.y_offset
        return x,y        
        
    def grid_to_submap(self, grid_x, grid_y):
        # get the submap index
        row = int(np.floor(float(grid_x) / self.submap_grid_num))
        col = int(np.floor(float(grid_y) / self.submap_grid_num))
        center_index = row * self.number_of_submap_col + col
        return center_index
    
    def map_to_submap(self, current_map):
        self.submap_list = []
        split_in_row = np.split(current_map, self.number_of_submap_row) # split the map into 16 parts
        for ii in range(self.number_of_submap_row):
            split_in_col = np.hsplit(split_in_row[ii], self.number_of_submap_col)
            for submap in split_in_col:
                self.submap_list.append(submap)
        
    def get_next_goal(self):
        # get the next goal for exploration
        # return the goal in (x,y) in terms of the map coordinate
        # while planning this, the robot should not move
        
        # get the robot odom
        current_pos = self.odom_listener.get_robot_xy()   
        
        # get the latest map
        width,height,origin,resolution,current_map = self.map_listener.get_map()
        
        # transform the odom into grid number
        grid_row, grid_col = self.map_to_grid(current_pos[0],current_pos[1])
        
        # get the submap corresponding to the current map
        submap_index = self.grid_to_submap(grid_row, grid_col)
        
        self.current_submap_center = copy.copy(self.submap_center_list[submap_index]) # deep copy the current center
        
        # sort the list of submap center
        self.submap_center_list.sort(key=self.center_sortkey)
        
        # split the current map
        self.map_to_submap(current_map)
        
        while True:
            for submap_center in self.submap_center_list:
                # if the grid has been explored enough times, don't explore again
                if submap_center.exploration_num >= self.max_exploration_num:
                    continue
                
                
                # get the submap            
                current_submap = self.submap_list[submap_center.index]
                
                
                # check whether there exists possibly reachable unexplored region
                reachable, unoccupied_center = check_reacheable_unexplored_part(current_submap, self.min_unoccupied_tol)
                
                # if reachable, find the map coordinate of the unoccupied center, and set as navigation goal
                if reachable:
                    unoccupied_grid_x = submap_center.grid_x - int(self.submap_grid_num / 2) + unoccupied_center[0]
                    unoccupied_grid_y = submap_center.grid_y - int(self.submap_grid_num / 2) + unoccupied_center[1]
                    # change the grid coordinate into map coordinate
                    goal_x,goal_y = self.grid_to_map(unoccupied_grid_x,unoccupied_grid_y)
                    print("Found next exploration target: (%f, %f)"%(goal_x,goal_y))
                    # increment the number of exploration
                    submap_center.exploration_num += 1
                    # visualize the goal
                    self.visualize_goal(goal_x,goal_y, scale = 1.0, marker_id = self.number_of_submaps)
                    return goal_x, goal_y
            
            # half the min_unoccupied_tol
            self.min_unoccupied_tol = self.min_unoccupied_tol / 2
            print("lower unoccupied tol to %d"%(self.min_unoccupied_tol))
            if self.min_unoccupied_tol == 0:
                break
            
        print("The map has been fully explored. Stop functioning")
        return None, None
        
    def center_sortkey(self,submap_center):
        x0 = self.current_submap_center.grid_x
        y0 = self.current_submap_center.grid_y
        x1 = submap_center.grid_x
        y1 = submap_center.grid_y
        return (x0-x1)**2 + (y0-y1)**2
    
        
    def get_to_goal(self,x,y):
        # set the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = 0
        self.goal.target_pose.pose.orientation.w = 0.7
        
        # call movebase to plan and move the robot to the goal
        self.client.send_goal(self.goal)
        
        self.client.wait_for_result()
        
        print(self.client.get_state)
        

if __name__ == "__main__":
    rospy.init_node('automatic_exploring_node')
    automatic_mapping = Automatic_mapping()
    #automatic_mapping.get_to_goal(2.0,0.0)
    
    print(automatic_mapping.submap_center_list[136].grid_x,automatic_mapping.submap_center_list[136].grid_y)    
    print(automatic_mapping.grid_to_map(automatic_mapping.submap_center_list[136].grid_x,automatic_mapping.submap_center_list[136].grid_y) )
    
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        # get the next exploration goal
        goal_x, goal_y = automatic_mapping.get_next_goal()
        
        # drive the robot towards the goal
        automatic_mapping.get_to_goal(goal_x, goal_y)
        
        # sleep for a while
        rate.sleep()
    
    
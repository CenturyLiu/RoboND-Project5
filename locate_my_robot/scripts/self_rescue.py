#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import time
import numpy as np

from geometry_msgs.msg import Twist
from lidar_listener import LidarListener

# class for resueing the robot
# when it's lost, i.e. covariance
# of the amcl_pose is large
class SelfRescueMode(object):
    def __init__(self):
        # velocity publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # lidar listener
        self.scan_listener = LidarListener()
        
        # angular and linear velocity in rescue mode
        self.angular_vel = 0.2 #rad/s
        self.linear_vel = 1.0 #m/s

        # threshold showing there exists obstacles
        self.stop_threshold = 1.5 # if any thing within this range emerge in the laser scan, stop robot
        self.move_threshold = 4.0 # if all points in the front are farther then this value, go straight forward

        # parameter storing the width of the robot
        self.robot_width = 1.5
        self.open_area_pts = 40 # at least 40 points exist in an "open area"
        #self.out_of_boundary_threshold = 5 # threshold for tolerating small number of points within move_threshold

        # paramter storing the current mode of the robot
        self.mode = "turning" # valid values: "turning", "forward"

        # define the variable to have velocity
        self.twist = Twist()

        # store the previous turning state
        self.previous_turn = None
        self.cumulative_turn_angle = 0.0 # store the cumulative turning angle
        self.view_coef = 0.5 # coefficient to determine the view for navigation detection
        self.view_coef_reset = 0.5 # backup copy of the coefficient to determine the view for navigation detection
        self.view_decay_rate = 0.5 # gradually narrow the view
        self.straight_count = 0
        self.restore_view_threshold = 10 # move straight 5 steps and then recover the view

    def do_nothing(self):
        print("why we are here?")

    def do_nothing1(self):
        print("why we are here?")

    def move_straight_forward(self, linear_speed = 0.0):
        '''
        Move the vehicle straight forward under constant speed

		---
		Parameter

		linear_speed : float
            the linear speed of the vehicle
        '''
        #twist = Twist()
        self.twist.linear.x = linear_speed
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)
        time.sleep(0.05)


    def counter_clockwise_turn(self, angle= 5.0):
        '''
		turn the robot counter clockwise
		with a small angle (only roughly since 
		neither does the robot have sensors like 
		imu in use, nor is the robot localized
		
		---
		Parameter

		angle : float
		    the small angle we are going to turn. Unit is in degree, 
		    angle can also be negative, which will lead to clockwise turning
        '''
        self.twist.linear.x = 0.0
        if angle != 0.0:
            self.twist.angular.z = angle / abs(angle) * self.angular_vel #self.angular_vel # 0.2 rad/s
        else:
            self.twist.angular.z = 0.0
        angle_rad = angle / 180.0 * np.pi 
        twist_time = abs(angle_rad / self.angular_vel)
        #print("angle_rad = %f, twist_time = %f" % (angle_rad,twist_time))
        # turn counter-clock wise
        self.pub.publish(self.twist)
        # sleep for sometime, so as to turn the robot
        time.sleep(twist_time)
        # stop the vehicle
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)
        time.sleep(0.1)


    

    def rescue(self):
        '''
        one step for rescueing the robot
        logic:
        
        check whether there exists any obstacle in the front
           of the robot

        if no obstacle exists:
            go straight forward
        else:
            if there exists open area to the left:
                turn counter-clock wise
            elif there exists open area to the right:
                turn clock-wise
            else:
                turn counter-clockwise     

        '''
        # get the latest scan data
        latest_scan = self.scan_listener.get_latest_scan()

        # check whether there exists any obstacle in the front
        middle_count = int(len(latest_scan.ranges) / 2)
        angle_min = latest_scan.angle_min
        angle_max = latest_scan.angle_max
        angle_increment = latest_scan.angle_increment
        
        no_obstacle = True

        front_distance_list = []
        front_distance_index_list = []

        for ii in range(int(middle_count * (1 - self.view_coef)),int(middle_count * (1 + self.view_coef))):
            x_distance = abs(latest_scan.ranges[ii] * np.cos(angle_min + angle_increment * ii))
            y_distance = abs(latest_scan.ranges[ii] * np.sin(angle_min + angle_increment * ii))
            if y_distance <= self.robot_width / 2: # check whether the point is in the front
                front_distance_list.append(x_distance)
                front_distance_index_list.append(ii)

        if self.mode == "turning":
            print("length == ", len(np.array(front_distance_list) < self.move_threshold))
            if min(front_distance_list) > self.move_threshold:
                # we have find an open area, go straight forward
                self.mode = "forward"  
                self.move_straight_forward(self.linear_vel)
                self.previous_turn = None # going straight forward, clear turning
                self.cumulative_turn_angle = 0.0
                self.straight_count = 0
                print("going straight forward!")
                return
            else:
                if self.cumulative_turn_angle > 360.0:
                    # turned around without finding open place, weaken the constraint on view
                    print("robot captured, weaken constraint")
                    self.view_coef *= self.view_decay_rate
                    self.cumulative_turn_angle = 0.0

            #    self.counter_clockwise_turn() # slowly turn counter-clockwise, try to find an open area
            #    return

        elif self.mode == "forward":
            self.previous_turn = None # going straight forward, clear turning
            if min(front_distance_list) > self.stop_threshold:
                # we are moving in an open area, keep going
                self.mode = "forward"
                self.move_straight_forward(self.linear_vel)
                self.straight_count += 1
                if self.straight_count > self.restore_view_threshold:
                    print("reset view")
                    self.view_coef = self.view_coef_reset # reset the view coefficient
                return
            else:
                # there exists obstacle in the front, stop
                self.mode = "turning"
                self.straight_count = 0
                self.cumulative_turn_angle = 0.0
                self.move_straight_forward(0.0) # stop robot

        # robot meets obstacle if coming here

        # check which area is occupied and don't turn into this direction
        front_distance_list = np.array(front_distance_list)
        front_distance_index_list = np.array(front_distance_index_list)
        small_distance_index = front_distance_index_list[front_distance_list <= self.stop_threshold]

        right_occupied = False
        left_occupied = False

        if len(small_distance_index) != 0:
            if min(small_distance_index) < middle_count * (1 - self.view_coef / 2):
                right_occupied = True
            if max(small_distance_index) > middle_count / (1 - self.view_coef / 2):
                left_occupied = True

        if right_occupied and not left_occupied:
            # turn left
            if not self.previous_turn == "right":
                print("right occupied, turn left")
                self.previous_turn = "left"
                self.counter_clockwise_turn(20.0)
                # update the cumulative turning angle
                self.cumulative_turn_angle += 20.0
                return

        if left_occupied and not right_occupied:
            # turn right
            if not self.previous_turn == "left": # avoid oscillating between turning left and right
                print("left occupied, turn right")
                self.previous_turn = "right"
                self.counter_clockwise_turn(-20.0)
                # update the cumulative turning angle
                self.cumulative_turn_angle -= 20.0
                return

        '''
        # check whether there exists an open area 
        exist_open_area = False
        center_of_open_area = None
        for ii in range(0,len(latest_scan.ranges) - self.open_area_pts):
            if min(latest_scan.ranges[ii:ii+self.open_area_pts]) > self.move_threshold + 1.0 and (ii < middle_count - 10 or ii > middle_count + 10):
                exist_open_area = True
                center_of_open_area = ii + self.open_area_pts / 2
                break

        if exist_open_area:
            print("Find open area, turn")
            self.counter_clockwise_turn(center_of_open_area * 0.25 - 90.0)
            self.mode = "turning"
            return
        else:
            # no open area exist, turn counter-clockwise for self-rescue
            print("Turn to find open area")
            self.mode = "turning"
            self.counter_clockwise_turn(45.0) # turn 45 degrees
            return
        '''
        # both left and right occupied, turn to find open area
        print("Turn to find open area")
        self.mode = "turning"
        if self.previous_turn != "right":
            #self.counter_clockwise_turn(15.0)
            self.counter_clockwise_turn(5.0)
            # update the cumulative turning angle
            self.cumulative_turn_angle += 5.0
        else:
            self.counter_clockwise_turn(45.0)
            # update the cumulative turning angle
            self.cumulative_turn_angle += 45.0
        self.previous_turn = "left"
        
        return
def main():
    rospy.init_node('self_rescue')
    self_rescuer = SelfRescueMode()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        self_rescuer.rescue()
        rate.sleep()

if __name__ == "__main__":
    main()

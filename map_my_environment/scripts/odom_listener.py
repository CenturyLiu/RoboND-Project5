#!/usr/bin/env python
from nav_msgs.msg import Odometry

import rospy

class OdomListener():
    def __init__(self):
        self.latest_odom = rospy.wait_for_message('/odom',Odometry)
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
    def odom_callback(self,data):
        self.latest_odom = data
    def get_robot_xy(self):
        return (self.latest_odom.pose.pose.position.x,self.latest_odom.pose.pose.position.x)

def main():
    rospy.init_node("odom_listener", anonymous = True)
    odom_listener = OdomListener()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print odom_listener.get_robot_xy()
        rate.sleep()

if  __name__ == "__main__":
    main()
#!/bin/sh
xterm -e " gazebo " &
sleep 5
xterm -e " source /opt/ros/kinetic/setup.bash; roscore" &
sleep 10
xterm -e " rosrun rviz rviz"

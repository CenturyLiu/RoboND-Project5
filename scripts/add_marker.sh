#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="/home/centuryliu/robotic_self_learning_ws/src/my_robot/worlds/open_classroom.world"" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:="/home/centuryliu/robotic_self_learning_ws/src/locate_my_robot/maps/slam_map.yaml"" &
sleep 10
xterm -e " roslaunch add_markers add_markers_only.launch" 

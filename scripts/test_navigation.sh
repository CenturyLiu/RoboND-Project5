#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="/home/centuryliu/robotic_self_learning_ws/src/my_robot/worlds/classroom.world"" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:="/home/centuryliu/robotic_self_learning_ws/src/my_robot/maps/classroom_map.yaml"" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" 

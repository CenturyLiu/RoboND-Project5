#!/bin/sh
xterm -e " roslaunch my_robot final_project_world.launch " &
sleep 5
xterm -e " roslaunch map_my_environment my_robot_mapping.launch" &
sleep 5
xterm -e " rosrun locate_my_robot self_rescue.py"

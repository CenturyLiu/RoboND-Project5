#!/bin/sh
xterm -e " roslaunch my_robot final_project_world.launch " &
sleep 5
xterm -e " roslaunch map_my_environment my_robot_mapping.launch" &
sleep 5
xterm -e " roslaunch pick_objects use_move_base.launch " &
sleep 5
xterm -e " roslaunch add_markers view_navigation_with_marker.launch" &
sleep 5
xterm -e " rosrun map_my_environment automatic_mapping.py"


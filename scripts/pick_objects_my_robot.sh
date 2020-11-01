#!/bin/sh
xterm -e " roslaunch my_robot final_project_world.launch " &
sleep 5
xterm -e " roslaunch locate_my_robot amcl_only.launch " &
sleep 5
xterm -e " roslaunch pick_objects use_move_base.launch " &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 10
xterm -e " rosrun pick_objects navigate_by_movebase"


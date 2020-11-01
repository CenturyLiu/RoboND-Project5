# RoboND-Project5
Implementation of project 5, Home-service Robot project Robotics Software Engineering Nanodegree by Udacity

## Introduction

This repository includes two similar implementation of the "Homeservice Robot project". One is using the turtlebot, the other is using a custom-build robot called [my_robot](https://github.com/CenturyLiu/RoboND-Project5/blob/main/my_robot/urdf/my_robot.xacro). Currently these two robots can map the gazebo environment with teleop_control or naive automatic exploration, and can navigate inside the different goals inside the environment. Later the function for my_robot may be updated, equipping it to catch and transfer objects.

![naive automatic exploration](https://github.com/CenturyLiu/RoboND-Project5/blob/main/naive_auto_mapping_demo.gif)
> naive automatic exploration

![myrobot navigate to pick up zone](https://github.com/CenturyLiu/RoboND-Project5/blob/main/navigate_to_pick_up.gif)
> myrobot navigate to pick up zone

![myrobot navigate to drop off zone](https://github.com/CenturyLiu/RoboND-Project5/blob/main/navigate_to_goal.gif)

> myrobot navigate to drop off zone

## Installation

This project is developed with Ubuntu 16.04 + ROS-kinetic
Download this project into your workspace, and download the kinetic version of the following official packages

- [gmapping](http://wiki.ros.org/gmapping)
- [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
- [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
- [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)

Also install xterm with

`sudo apt-get install xterm`

Then please follow the instructions in the next section to modify the path in several shell scipts, otherwise those scipts won't work properly.

## Path Issue in script files

In [test_slam.sh](https://github.com/CenturyLiu/RoboND-Project5/blob/main/scripts/test_slam.sh), a file path is included in line 2; In (test_navigation.sh)[https://github.com/CenturyLiu/RoboND-Project5/blob/main/scripts/test_navigation.sh], (home_service.sh)[https://github.com/CenturyLiu/RoboND-Project5/blob/main/scripts/home_service.sh], (pick_objects.sh)[https://github.com/CenturyLiu/RoboND-Project5/blob/main/scripts/pick_objects.sh],file paths are included in line 2 and line 4.
Please make sure to change the paths from 
    
    "/home/centuryliu/robotic_self_learning_ws/src/my_robot/worlds/open_classroom.world"
to

    "(path_to_your_workspace)/src/my_robot/worlds/open_classroom.world"
Otherwise the scripts won't work properly



## File structure

    ├──add_markers                    # add_markers C++ node
    │   ├── src/add_markers.cpp
    │   ├── rviz/nav_with_marker.rviz  # view_navigation.rviz + marker display
    |   ├── launch/add_markers.launch  # launch the add_markers node and display in rviz
    ├──locate_my_robot                 # package for localizing my_robot
    │   ├── launch/amcl_only.launch    # launch file for using amcl for my_robot with slam map
    │   ├── map
        │   ├──slam_map.pgm            # slam map pgm
        │   ├──slam_map.yaml           # slam map yaml. The map is created by using slam_gmapping, coordinates of the map are in line with gazebo
        │   ├──open_classroom_map.zip  # high resolution map created by using pgm_map_creator. Coordinate rotated pi/2 with respect to gazebo
    │   ├── scripts
        │   ├──my_robot_teleop.py      # teleop my_robot
        │   ├──self_rescue.py          # naive exploration implementation of my_robot, used in slam_auto_naive.sh
    ├──map_my_environment              # map the environment
    │   ├── launch/my_robot_mapping.launch # call slam_gmapping, args configured for my_robot
    ├──my_robot                        # package containing simulation environment and robot description
    │   ├── launch/final_project_world.launch # launch the world for this project and the my_robot
    │   ├── worlds/open_classroom.world # world created for my
    ├──pick_objects                    # pick_objects C++ node
    │   ├── launch/use_move_base.launch # launch file with args specified for my_robot to use move_base
    │   ├── src/pick_objects.cpp       # the pick_objects node
    ├── scripts                     # shell scripts files
    │   ├── add_marker.sh              # show marker to represent pick up and drop off
    │   ├── add_marker_my_robot.sh     # same for add_marker; for my_robot, same for all scripts with "_my_robot"
    │   ├── home_service.sh            # the main script for project
    │   ├── home_service_my_robot.sh
    │   ├── pick_objects.sh            # navigate turtlebot to pick up zone and drop off zone
    │   ├── pick_objects_my_robot.sh   
    │   ├── slam_auto_naive.sh         # naive implementation of automatic mapping using my_robot
    │   ├── slam_my_robot.sh           # teleop my_robot to map the environment
    │   ├── test_navigation.sh         # test navigation configuration of turtlebot    
    │   ├── test_slam.sh               # test slam configuration of turtlebot
    └──



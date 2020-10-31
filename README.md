# RoboND-Project5
Implementation of project 5, Robotics Software Engineering Nanodegree by Udacity


## Packages
    ├── map                          # map files
        │   ├── ...
        ├── scripts                   # shell scripts files
        │   ├── test_slam.sh
        │   ├── test_navigation.sh
        ├──rvizConfig                      # rviz configuration files
        │   ├── ...
        ├──pick_objects                    # pick_objects C++ node
        │   ├── src/pick_objects.cpp
        │   ├── ...
        ├──add_markers                     # add_marker C++ node
        │   ├── src/add_markers.cpp
        │   ├── ...
        └──

## Path Issue in script files

In [test_slam.sh](https://github.com/CenturyLiu/RoboND-Project5/blob/main/scripts/test_slam.sh), a file path is included in line 2; In (test_navigation.sh)[https://github.com/CenturyLiu/RoboND-Project5/blob/main/scripts/test_navigation.sh], file paths are included in line 2 and line 4.
Please make sure to change the paths from 
    
    "/home/centuryliu/robotic_self_learning_ws/src/my_robot/worlds/classroom.world"
to

    "(path_to_your_workspace)/src/my_robot/worlds/classroom.world"
Otherwise the scripts won't work properly

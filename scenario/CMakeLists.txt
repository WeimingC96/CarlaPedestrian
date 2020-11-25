cmake_minimum_required(VERSION 3.0.2)
project(scenario)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

roslaunch_add_file_check(launch IGNORE_UNSET_ARGS)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS src/carla_pedestrian_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

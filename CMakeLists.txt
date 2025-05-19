
cmake_minimum_required(VERSION 3.0.2)
project(kortex_3dpaint)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp 
                                        rospy 
                                        std_msgs 
                                        message_generation 
                                        actionlib_msgs 
                                        actionlib 
                                        kortex_driver 
                                        sensor_msgs 
                                        geometry_msgs 
                                        aruco_msgs
                                        aruco)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)

# Install

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS src/test/pick_and_place_moveit.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
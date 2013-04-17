cmake_minimum_required(VERSION 2.8.3)
project(vp6242_helping_hand)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    hg_controller_manager
    hg_realtime_tools 
    roscpp 
    denso_common 
    urdf 
    moveit_core moveit_ros_planning 
    moveit_ros_planning_interface 
    pluginlib)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES vp6242_helping_hand
#  CATKIN_DEPENDS hg_controller_manager roscpp
   CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(vp6242_helping_hand
#   src/${PROJECT_NAME}/vp6242_helping_hand.cpp
# )

## Declare a cpp executable
add_executable(vp6242_helping_hand_node src/vp6242_helping_hand_node.cpp)
add_executable(vp6242_dual_helping_hand_node src/vp6242_dual_helping_hand_node.cpp)
add_executable(vp6242_helping_hand_moveit src/vp6242_helping_hand_moveit.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(vp6242_helping_hand_node vp6242_helping_hand_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vp6242_helping_hand_node
  ${catkin_LIBRARIES}
  denso_common
  hg_controller_manager  
)

target_link_libraries(vp6242_dual_helping_hand_node
  ${catkin_LIBRARIES}
  denso_common
  hg_controller_manager  
)

target_link_libraries(vp6242_helping_hand_moveit
  ${catkin_LIBRARIES}    
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS vp6242_helping_hand vp6242_helping_hand_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vp6242_helping_hand.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

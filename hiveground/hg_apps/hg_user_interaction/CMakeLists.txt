cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#http://answers.ros.org/question/12790/ros-qt-creator-gui/
find_package(Qt4 REQUIRED)
# enable/disable some Qt features
set( QT_USE_QTGUI TRUE )
#set( QT_USE_QTOPENGL TRUE )
#set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

set(qt_srcs
    src/main.cpp
    src/gesture_detector.cpp
    src/gesture_hand_pushpull.cpp
    src/gesture_body_movement.cpp)
      
set(qt_hdrs
    include/${PROJECT_NAME}/main.h
    include/${PROJECT_NAME}/gesture_detector.h
    include/${PROJECT_NAME}/gesture_hand_pushpull.h
    include/${PROJECT_NAME}/gesture_body_movement.h)
      
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

QT4_WRAP_UI(uis_h src/main.ui)
# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(${PROJECT_NAME} ${uis_h} ${qt_srcs} ${qt_moc_srcs})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} qt_property_browser visual_editor)
rosbuild_link_boost(${PROJECT_NAME} thread signals)

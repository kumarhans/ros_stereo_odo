cmake_minimum_required(VERSION 2.8.3)
project(ros_stereo_odo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
  sensor_msgs
  tf2_ros
)


find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)



catkin_package( 
   #INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/gtsam ${CATKIN_DEVEL_PREFIX}/include/gtsam
   CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs image_transport sensor_msgs
)

 
 
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)



 
add_executable (stereo_odom_node
src/stereo_odometry.cpp
src/detect_features.cpp
src/rectify_images.cpp
src/point_util.cpp
src/optimize_pose.cpp
src/visualize.cpp
)

target_link_libraries(stereo_odom_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

 
 
 
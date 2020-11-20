cmake_minimum_required(VERSION 3.0.3)
project(pc_metrics)

set(CMAKE_CXX_FLAGS "-std=c++03")
 
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  roslaunch
  nav_msgs

)
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()
 

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable( cloud_metric src/cloud_metric)
target_link_libraries(cloud_metric ${catkin_LIBRARIES})
target_link_libraries (cloud_metric ${PCL_LIBRARIES})
 
 

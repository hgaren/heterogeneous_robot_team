cmake_minimum_required(VERSION 3.0.3)
project(hetero_waypoint)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  roslaunch

)

catkin_package()
roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(uav_waypoint_controller src/uav_waypoint_controller.cpp)
target_link_libraries(uav_waypoint_controller ${catkin_LIBRARIES})

add_executable(ugv_waypoint_controller src/ugv_waypoint_controller.cpp)
target_link_libraries(ugv_waypoint_controller ${catkin_LIBRARIES})
 
find_package(message_generation)
catkin_package(CATKIN_DEPENDS message_runtime)

install(DIRECTORY  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

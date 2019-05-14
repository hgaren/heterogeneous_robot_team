cmake_minimum_required(VERSION 3.0.3)
project(hetero_mapping)

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
roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(uav_mapping_real src/uav_mapping_real.cpp)
target_link_libraries(uav_mapping_real ${catkin_LIBRARIES})
target_link_libraries (uav_mapping_real ${PCL_LIBRARIES})

add_executable(cloud_merge_real src/cloud_merge_real.cpp)
target_link_libraries(cloud_merge_real ${catkin_LIBRARIES})
target_link_libraries (cloud_merge_real ${PCL_LIBRARIES})


add_executable(cloud_merge src/cloud_merge)
target_link_libraries(cloud_merge ${catkin_LIBRARIES})
target_link_libraries (cloud_merge  ${PCL_LIBRARIES})

 
install(DIRECTORY  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Heterogenous Team ROS-Gazebo Simulation Package

## Istanbul Technical University Laboratory
The purpose of this package is to simulate heterogenous robot team (an aerial and a ground vehicle) in Gazebo, achieve multi-map merging from different view point and apply a senario for exploration.

### Notes
This ROS package  also includes other packages. 
Before building this package, you should install these packages: hector_pose_estimatition , hector_uav_msg, hardware_interface, controller_interface, move_base, robot_localization, hector_gazebo_plugins, hector_sensor_description, gazebo_ros_control, message_to_tf, interactive_marker_twist_server,
twist_mux, dwa_local_planner,octomap_server , joint_State_controller, diff_drive_controller, velodyne_gazebo_plugin

If gazebo wont open, you also need to install "palm_tree" , "oak_tree", "construction_barel", "fire hydrant", "asphalt_plane", "dumpster", "jersey_barrier" models from Gazebo. 

For cmake , minimum requirement is 3.03 . If you need to upgrade the CMake, you can follow this instructions from  [link](https://askubuntu.com/questions/610291/how-to-install-cmake-3-2-on-ubuntu)

### Applications
Installs ROS packages from main server
```
sudo apt-get install ros-<distro>-<package-name>
```

Opens gazebo simulation, localization nodes, husky ugv (HUSKY) and hector uav (HECTOR QUADROTOR).

```
roslaunch hetero_demo outdoor_gazebo.launch 
```

Opens move base 2D navigation package for both robots.Also open the atittude controller for air vehicle

```
roslaunch hetero_navigation move_base_all.launch 
```

Python scripts for keyboard teleoperation UGV and UAV 
```
rosrun hetero_teleop ugv_keyboard_teleop.py
rosrun hetero_teleop uav_keyboard_teleop.py
```

Designs an exploration senario. The senario is GPS based bounded autonomous exploration. By writing *the latitudes and longitutes and number of the points* in the node for the area that wanted to bound, an algorithm  starts to send 2d coordinates to the move_base for both robots  in that closed area.

```
rosrun hetero_waypoint uav_waypoint_controller
rosrun hetero_waypoint ugv_waypoint_controller
```
or open only one waypoint launch file for both vehicle

```
roslaunch hetero_waypoint hetero_waypoint_nav.launch
```

Subscribes both cloud data and creates one merged map from two different viewpoints of 3D cloud.  
```
roslaunch hetero_mapping map_merging.launch
```
Gazebo Simulation Screenshot

![ScreenShot](/hetero_2.png)

Rviz Screenshot while accomplising senario
 
![ScreenShot](/hetero_1.png)



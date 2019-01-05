# *Heterogenous Team Gazebo Simulation Package
Istanbul Technical University Laboratory



Opens gazebo simulation, localizations, husky ugv and hector uav.

```
roslaunch hetero_demo outdoor_gazebo.launch 
```

Opens move base navigation package for both robot.

```
roslaunch hetero_navigation move_base_all.launch 
```

Sends GPS waypoints to the move base until points finish
```
rosrun hetero_waypoint uav_waypoint_controller
rosrun hetero_waypoint ugv_waypoint_controller

Subscribe both cloud data and create two different 3D map
```
roslaunch hetero_mapping octomap.launch
```

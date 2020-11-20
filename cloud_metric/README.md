# Point Cloud Metric Calculation and Graph

## Performance Metric
Metric values are coming from summing average distance from nearest points. If the value is lower, the noise and disturbance on point cloud is getting lower. 
(EPFL's point-to-point Performance Metric System)

## Occupancy Metric
Total number of filling points are divided to  pre-defined volume of an environment. The value is getting higher if more points are defined in environment. Resolution, surroundings, noise and disturbance are effect the filling point thus, it should have a meaning if it use  with performance metric values.

Below code subscribes UAV's, UGV's, Merged Map's cloud topic, finds performance and occupancy values according to given resolutions and writes txt files to plot 
```
rosrun pc_metrics cloud_metric
```

## Graphs

Below code can plot graph of merged map's or individual UAV's or individual UGV's performance and occupancy rate according to ready txt file values.

```
rosrun pc_metrics plot_all.py 
```
Below code can plot graph all resolutions that defined in txt files.
```
rosrun pc_metrics plot_resolutions.py 
```

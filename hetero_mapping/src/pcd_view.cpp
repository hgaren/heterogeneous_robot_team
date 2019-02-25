// CPP
#include <stdio.h>
#include <iostream>
#include <cmath>
// ROS
#include <ros/ros.h>
 
// PCL LIBRARY
 
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace pcl ;

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pcd_viewer");
  ros::NodeHandle n;
   ros::Rate rate(1);
  printf("Starting ... \n" );

 
     

  visualization::PCLVisualizer viewer("Cloud Viewer");
  pcl::PointCloud<pcl::PointXYZ>::Ptr body (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("/home/roblab/hetero_ws/src/hetero_mapping/src/pcd/v_h/ugv.pcd", cloud_blob);
  pcl::PointCloud<pcl::PointXYZ>::Ptr head (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::io::loadPCDFile ("/home/roblab/hetero_ws/src/hetero_mapping/src/pcd/v_h/uav.pcd",*head);
  
   uint8_t r=255;
  uint8_t g=0;
  uint8_t b=0;
  uint32_t rgb = (r << 16)| (g<<8) | b ;
 
 pcl::fromPCLPointCloud2 ( cloud_blob, *body);
for(size_t j=0; j<body->points.size(); j++){
    head->points[j].x = 1;
    head->points[j].y=1;
    head->points[j].z = 1;
    
 }
 



  viewer.addPointCloud(body,"body");
 // viewer.addPointCloud(point,"head");
  viewer.spin();
   
     
  
  return (0);
}
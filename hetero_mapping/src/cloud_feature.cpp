// CPP
#include <stdio.h>
#include <iostream>
#include <cmath>
// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
// PCL LIBRARY
#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/registration.h>

#include <pcl/keypoints/iss_3d.h>


#define PI 3.14159265
#define field 30
using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;

boost::shared_ptr<NodeHandle> node_; 
Subscriber pcSubscriber;
Subscriber pcSubscriber1;
Publisher pcPublisher;



pcl::PointCloud<pcl::PointXYZI>::Ptr ugv_cloud          (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr uav_cloud           (new pcl::PointCloud<pcl::PointXYZI>);

pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>());
search::KdTree<PointXYZI>::Ptr tree (new search::KdTree<PointXYZI> ());
sensor_msgs::PointCloud2 cloud_msg1;


bool pc_avaliable = false, pc2_avaliable = false;

void pcCallback_uav(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
  uav_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*laserCloudMsg, *uav_cloud);
  pc2_avaliable=true;
}

void pcCallback_ugv(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
  ugv_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*laserCloudMsg, *ugv_cloud);
  pc_avaliable=true;
}



void process(){
 
  double model_resolution =0.0058329;

  ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> iss_detector;
  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (6 * model_resolution);
  iss_detector.setNonMaxRadius (4 * model_resolution);
  iss_detector.setThreshold21 (0.975);
  iss_detector.setThreshold32 (0.975);
  iss_detector.setMinNeighbors (5);
  iss_detector.setNumberOfThreads (4);
  iss_detector.setInputCloud (ugv_cloud);
  iss_detector.compute (*keypoints);  
 
  toROSMsg(*keypoints, cloud_msg1);
  cloud_msg1.header.frame_id ="world";
  cloud_msg1.header.stamp = ros::Time::now();
  pcPublisher.publish(cloud_msg1);


}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "cloud_feature");
  node_.reset(new NodeHandle());
  pcSubscriber  = node_->subscribe("/uav/octomap_point_cloud_centers",        200, pcCallback_uav);
  pcSubscriber1 = node_->subscribe("/ugv/octomap_point_cloud_centers",   200, pcCallback_ugv);
  pcPublisher = node_->advertise<sensor_msgs::PointCloud2>("total_cloud",1,false);

  
  printf("Starting ... \n" );
  ros::Rate r(100);
  while(node_->ok()){

    

    if (pc_avaliable && pc2_avaliable)
      {
      process();
      
      }
      ros::spinOnce();
      r.sleep();
  }
  return (0);
}
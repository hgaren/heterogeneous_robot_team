// CPP
#include <stdio.h>
#include <iostream>
#include <cmath>
// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
// PCL LIBRARY
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/keypoints/iss_3d.h>
 
#include <pcl/io/pcd_io.h>
#define PI 3.14159265
#define field 30
using namespace std;
using namespace ros;

boost::shared_ptr<NodeHandle> node_; 
Subscriber pcSubscriber;
Subscriber pcSubscriber1;
Publisher pcPublisher;

Eigen::Matrix4f trans_final = Eigen::Matrix4f::Identity();
Eigen::Matrix4f trans_offset = Eigen::Matrix4f::Identity();
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target          (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source          (new pcl::PointCloud<pcl::PointXYZI>);


pcl::PointCloud<pcl::PointXYZI>::Ptr ugv_cloud          (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr ugv_cloud_trans         (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr ugv_cloud_final         (new pcl::PointCloud<pcl::PointXYZI>);


pcl::PointCloud<pcl::PointXYZI>::Ptr uav_cloud           (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZ>());
sensor_msgs::PointCloud2 cloud_msg1;


pcl::PointCloud<pcl::PointXYZI> total_cloud;
int do_once=1;
bool pc_avaliable = false, pc2_avaliable = false;
double highest=0;
double yaw=-field;
double pitch=-field+25;
double target_yaw=0;
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
  int match_score=0;
 
  while(yaw <field && node_->ok()){
  trans_offset(0,0)=cos(yaw* PI / 180.0 );
  trans_offset(0,1)=-sin(yaw* PI / 180.0 );
  trans_offset(1,0)=sin(yaw* PI / 180.0 );
  trans_offset(1,1)=cos(yaw* PI / 180.0 );
  trans_offset(2,2)=1;
  trans_offset(3,3)=1;
  pcl::transformPointCloud (*ugv_cloud, *ugv_cloud_trans, trans_offset);
 
  //std::cout <<"offset: "<< trans_offset << std::endl;
  //std::cout <<"ugv_cloud_count: "<< ugv_cloud_trans->points.size () << std::endl;
 // std::cout <<"uav_cloud_count: "<< uav_cloud->points.size () << std::endl;
  size_t size= ugv_cloud_trans->points.size ();

  if(size > uav_cloud->points.size ())
    size=uav_cloud->points.size ();
  double distance[size];

  int match_score=0;
  double threshold=0.5;
  cloud_target=ugv_cloud_trans;
  cloud_source=uav_cloud;
  for (size_t i = 0; i <size; ++i)
  {


  		distance[i]=sqrt(pow(cloud_target->points[i].x-cloud_source->points[i].x,2)+pow(cloud_target->points[i].y-cloud_source->points[i].y,2)+ pow(cloud_target->points[i].z-cloud_source->points[i].z,2));

        if(distance[i]<threshold)  
          match_score++;
     
  }
  sensor_msgs::PointCloud2 cloud_msg1;

  if(match_score>highest)
  {
     highest=match_score;
     ugv_cloud_final=ugv_cloud_trans;
 
     toROSMsg(*ugv_cloud_trans, cloud_msg1);
     cloud_msg1.header.frame_id ="world";
     cloud_msg1.header.stamp = ros::Time::now();
     pcPublisher.publish(cloud_msg1);
     target_yaw=yaw;

  }
    std::cout <<"yaw:"<<yaw<<"target_yaw:"<<target_yaw<<" matched:"<<match_score<<"highest"<< highest<<endl;

    yaw=yaw+0.5;
 }
  
  trans_offset(0,0)=cos(target_yaw* PI / 180.0 );
  trans_offset(0,1)=-sin(target_yaw* PI / 180.0 );
  trans_offset(1,0)=sin(target_yaw* PI / 180.0 );
  trans_offset(1,1)=cos(target_yaw* PI / 180.0 );
  trans_offset(2,2)=1;
  trans_offset(3,3)=1;
  pcl::transformPointCloud (*ugv_cloud, *ugv_cloud_final, trans_offset); 

  /*pcl::PointCloud<pcl::PointXYZI>::Ptr ugv_cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (ugv_cloud_final);
  sor.setLeafSize (0.2f, 0.2f, 0.2f);
  sor.filter (*ugv_cloud_filtered);
  	 */

  if(do_once==1){
	  toROSMsg(*ugv_cloud_final, cloud_msg1);
	  cloud_msg1.header.frame_id ="world";
	  cloud_msg1.header.stamp = ros::Time::now();
	  pcPublisher.publish(cloud_msg1);
	  do_once=0;
	} 

 


  std::cout <<"icp starting..."<< std::endl;
  pcl::PointCloud<pcl::PointXYZI> final_cloud;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp; // Generalized
  icp.setMaxCorrespondenceDistance(1);
  icp.setMaximumIterations (5);
  icp.setInputSource(ugv_cloud_final);
  icp.setInputTarget(uav_cloud);
  icp.align(final_cloud);
  trans_final = icp.getFinalTransformation();
  std::cout << "has converged:" << icp.hasConverged() <<" Score: " << icp.getFitnessScore() <<std::endl;
  std::cout <<"final: "<< trans_final << std::endl;
  pcl::transformPointCloud (*ugv_cloud_final, *ugv_cloud_final, trans_final);
  total_cloud=*ugv_cloud_final+*uav_cloud;
  toROSMsg(total_cloud, cloud_msg1);
  cloud_msg1.header.frame_id ="world";
  cloud_msg1.header.stamp = ros::Time::now();
  pcPublisher.publish(cloud_msg1);
    
  /*double model_resolution =0.0058329;

  pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

  iss_detector.setSalientRadius (6 * model_resolution);
  iss_detector.setNonMaxRadius (4 * model_resolution);
  iss_detector.setThreshold21 (0.975);
  iss_detector.setThreshold32 (0.975);
  iss_detector.setMinNeighbors (5);
  iss_detector.setNumberOfThreads (4);
  iss_detector.setInputCloud (*ugv_cloud_final);
  iss_detector.compute (*keypoints);  
  sensor_msgs::PointCloud2 cloud_msg2;
  toROSMsg(*keypoints, cloud_msg2);
  cloud_msg2.header.frame_id ="world";
  cloud_msg2.header.stamp = ros::Time::now();
  pcPublisher.publish(cloud_msg2);*/


}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "gicp_cloud_merge");
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
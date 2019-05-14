/* Two Map  Merging  Algorithm Which Have Diffirent Angle of Views in SIMULATIONS */
/*Garen Haddeler 
The Bachelor Project  
07.05.2019 */
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
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/keypoints/iss_3d.h>
 #include <tf2/LinearMath/Quaternion.h>
#include <pcl/io/pcd_io.h>
#define PI 3.14159265
#define field 12
using namespace std;
using namespace ros;

boost::shared_ptr<NodeHandle> node_; 
Subscriber pcSubscriber;
Subscriber pcSubscriber1;
Subscriber odomSubscriber;
Subscriber odomSubscriber1;
Publisher pcPublisher;

Eigen::Matrix4f t_world_uav = Eigen::Matrix4f::Identity();
Eigen::Matrix4f t_world_ugv = Eigen::Matrix4f::Identity();

Eigen::Matrix4f t_uav_world = Eigen::Matrix4f::Identity();

Eigen::Matrix4f t_ugv_world = Eigen::Matrix4f::Identity();
Eigen::Matrix4f t_offset = Eigen::Matrix4f::Identity();
Eigen::Matrix4f trans_final = Eigen::Matrix4f::Identity();

Eigen::Matrix4f Rt = Eigen::Matrix4f::Identity();


pcl::PointCloud<pcl::PointXYZ>::Ptr ugv_cloud          (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ugv_cloud_trans         (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ugv_cloud_final        (new pcl::PointCloud<pcl::PointXYZ>);



pcl::PointCloud<pcl::PointXYZ > total_cloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr uav_cloud           (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 cloud_msg1;
nav_msgs::Odometry odom_uav;
nav_msgs::Odometry odom_ugv;


bool pc_avaliable = false, pc2_avaliable = false;
double do_multiple=0;
double yaw=-field;
double target_yaw=0;
double qx=0;
double qy=0;
double qz=0;
double qw=0;

void pcCallback_uav(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
 uav_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::fromROSMsg(*laserCloudMsg, *uav_cloud);
 pc2_avaliable=true;
}

void pcCallback_ugv(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
 ugv_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::fromROSMsg(*laserCloudMsg, *ugv_cloud);
 pc_avaliable=true;
}

void odomCallback_uav( const nav_msgs::Odometry msg){
  odom_uav=msg;
}
void odomCallback_ugv( const nav_msgs::Odometry msg){
 odom_ugv=msg; 
 
}

void process(){
  
  //sets inital transformations
 if(do_multiple<1200){
  qx=odom_uav.pose.pose.orientation.x;
  qy=odom_uav.pose.pose.orientation.y;
  qz=odom_uav.pose.pose.orientation.z;
  qw=odom_uav.pose.pose.orientation.w;
  t_world_uav(0,0)=1 - 2*qy*qy - 2*qz*qz;
  t_world_uav(0,1)=2*qx*qy - 2*qz*qw;
  t_world_uav(0,2)=2*qx*qz + 2*qy*qw;
  t_world_uav(0,3)=odom_uav.pose.pose.position.x;


  t_world_uav(1,0)=2*qx*qy + 2*qz*qw;
  t_world_uav(1,1)=1 - 2*qx*qx - 2*qz*qz;
  t_world_uav(1,2)=2*qy*qz - 2*qx*qw;
  t_world_uav(1,3)=odom_uav.pose.pose.position.y;

  t_world_uav(2,0)=2*qx*qz - 2*qy*qw;

  t_world_uav(2,1)=2*qy*qz + 2*qx*qw;
  t_world_uav(2,2)= 1 - 2*qx*qx - 2*qy*qy;
  t_world_uav( 2,3)=0;

  t_world_uav(3,0)=0;
  t_world_uav(3,1)=0;
  t_world_uav(3,2)=0;
  t_world_uav(3,3)=1;


  Rt(0,0)=t_world_uav(0,0);
  Rt(0,1)=t_world_uav(1,0);
  Rt(0,2)=t_world_uav(2,0);
  
  Rt(1,0)=t_world_uav(0,1);
  Rt(1,1)=t_world_uav(1,1);
  Rt(1,2)=t_world_uav(1,2);
  
  Rt(2,0)=t_world_uav(0,2);
  Rt(2,1)=t_world_uav(1,2);
  Rt(2,2)=t_world_uav(2,2);

  t_uav_world(0,3)=0;//-(Rt(0,0)*t_world_uav(0,3)+Rt(0,1)*t_world_uav(1,3)+Rt(0,2)*t_world_uav(2,3));
  t_uav_world(1,3)=0;//-(Rt(1,0)*t_world_uav(0,3)+Rt(1,1)*t_world_uav(1,3)+Rt(1,2)*t_world_uav(2,3));
  t_uav_world(2,3)=0;//-(Rt(2,0)*t_world_uav(0,3)+Rt(2,1)*t_world_uav(1,3)+Rt(2,2)*t_world_uav(2,3));


  t_uav_world(0,0)=Rt(0,0);
  t_uav_world(0,1)=Rt(0,1);
  t_uav_world(0,2)=Rt(0,2) ;

  t_uav_world(1,0)=Rt(1,0);
  t_uav_world(1,1)=Rt(1,1);
  t_uav_world(1,2)=Rt(1,2); 

  t_uav_world(2,0)=Rt(2,0);
  t_uav_world(2,1)=Rt(2,1);
  t_uav_world(2,2)=Rt(2,2); 

  t_uav_world(3,0)=0;
  t_uav_world(3,1)=0;
  t_uav_world(3,2)=0;
  t_uav_world(3,3)=1;
 

  cout<<"t_uav_world:  "<<t_uav_world<<endl;
   
  t_offset=t_uav_world;
  
   
  do_multiple++;
  ugv_cloud_trans.reset(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::transformPointCloud (*ugv_cloud, *ugv_cloud_trans, t_offset);
  toROSMsg(*ugv_cloud_trans, cloud_msg1);
  cloud_msg1.header.frame_id ="world";
  cloud_msg1.header.stamp = ros::Time::now();
  pcPublisher.publish(cloud_msg1);
 }
    //finds  small transformations with using G-ICP. It generates map with smaller merging error 
 else if(do_multiple>=1200){
  pcl::transformPointCloud (*ugv_cloud, *ugv_cloud_trans, t_offset);
  std::cout <<"icp starting..."<< std::endl;
  pcl::PointCloud<pcl::PointXYZ> final_cloud;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // Generalized
  icp.setMaxCorrespondenceDistance(1);
  icp.setMaximumIterations (2);
  icp.setInputSource(ugv_cloud_trans);
  icp.setInputTarget(uav_cloud);
  icp.align(final_cloud);
  trans_final = icp.getFinalTransformation();
  pcl::transformPointCloud (*ugv_cloud_trans, *ugv_cloud_trans, trans_final);
  //sums UAV's and UGV's transformed cloud
  total_cloud=*ugv_cloud_trans+*uav_cloud;
  toROSMsg(total_cloud, cloud_msg1);

  cloud_msg1.header.frame_id ="world";
  cloud_msg1.header.stamp = ros::Time::now();
  pcPublisher.publish(cloud_msg1);
 }
}


int main (int argc, char** argv){
 ros::init (argc, argv, "simulation_cloud_merge");
 ros::NodeHandle n;
 pcSubscriber  = n.subscribe("/uav/octomap_point_cloud_centers",        200, pcCallback_uav);
 pcSubscriber1 = n.subscribe("/ugv/octomap_point_cloud_centers",   200, pcCallback_ugv);
 odomSubscriber = n.subscribe("/uav/odometry_filtered",   200, odomCallback_uav);
 odomSubscriber1 = n.subscribe("/odometry/filtered",   200, odomCallback_ugv);

 pcPublisher = n.advertise<sensor_msgs::PointCloud2>("total_cloud",1,false);

 
 printf("Starting ... \n" );
 ros::Rate r(100);
 while(n.ok()){
   
  if (pc_avaliable && pc2_avaliable){
    process(); 
   }
    ros::spinOnce();
    r.sleep();
  }
  return (0);
}
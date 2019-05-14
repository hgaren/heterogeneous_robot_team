/* UAV's Real-time Cloud Registiration Using ICP method */
/*Garen Haddeler 
The Bachelor Project  
07.05.2019 */
// CPP
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <math.h>
// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
// PCL LIBRARY
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/keypoints/iss_3d.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/io/pcd_io.h>


using namespace std;
using namespace ros;

boost::shared_ptr<NodeHandle> node_; 
Subscriber pcSubscriber;
Subscriber pcSubscriber1;
Subscriber pcSubscriber2;
Subscriber odomSubscriber;
Subscriber odomSubscriber1;
Publisher pcPublisher;
Publisher pcPublisher1;
Publisher pub_odom;

Eigen::Matrix4f trans_final = Eigen::Matrix4f::Identity();
Eigen::Matrix4f t_world_uav = Eigen::Matrix4f::Identity();
Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
pcl::PointCloud<pcl::PointXYZ> total_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud           (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud      (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 cloud_msg1;
sensor_msgs::PointCloud2 cloud_msg2;

nav_msgs::Odometry odom_uav;
nav_msgs::Odometry odom_new;

sensor_msgs::Imu imu_uav;
double qx=0;
double qy=0;
double qz=0;
double qw=0;

double counter=0;

bool pc_avaliable = false;
bool pc_avaliable1 = false;
bool do_once=true;
 
void pcCallback_uav(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
 cur_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::fromROSMsg(*laserCloudMsg, *cur_cloud);
  pc_avaliable=true;

 }

 void odomCallback_uav( const nav_msgs::Odometry msg){
  odom_uav=msg;
}
 void imuCallback_uav( const sensor_msgs::Imu msg){
  imu_uav=msg;
}


void process(){

  qx=imu_uav.orientation.x;
  qy=imu_uav.orientation.y;
  qz=imu_uav.orientation.z;
  qw=imu_uav.orientation.w;

  t_world_uav(0,0)=1 - 2*qy*qy - 2*qz*qz;
  t_world_uav(0,1)=2*qx*qy - 2*qz*qw;
  t_world_uav(0,2)=2*qx*qz + 2*qy*qw;
  t_world_uav(0,3)=odom_uav.pose.pose.position.x;;


  t_world_uav(1,0)=2*qx*qy + 2*qz*qw;
  t_world_uav(1,1)=1 - 2*qx*qx - 2*qz*qz;
  t_world_uav(1,2)=2*qy*qz - 2*qx*qw;
  t_world_uav(1,3)=odom_uav.pose.pose.position.y;

  t_world_uav(2,0)=2*qx*qz - 2*qy*qw;

  t_world_uav(2,1)=2*qy*qz + 2*qx*qw;
  t_world_uav(2,2)= 1 - 2*qx*qx - 2*qy*qy;
  t_world_uav( 2,3)=odom_uav.pose.pose.position.z;

  t_world_uav(3,0)=0;
  t_world_uav(3,1)=0;
  t_world_uav(3,2)=0;
  t_world_uav(3,3)=1;

  trans(0,0)=0;
  trans(0,1)=0;
  trans(0,2)=1;
  trans(0,3)=0;


  trans(1,0)=0;
  trans(1,1)=1;
  trans(1,2)=0;
  trans(1,3)=0;

  trans(2,0)=-1;
  trans(2,1)=0;
  trans(2,2)= 0;
  trans( 2,3)=0;

  trans(3,0)=0;
  trans(3,1)=0;
  trans(3,2)=0;
  trans(3,3)=1;
  
  double distance =0;

  /**for(size_t j=0; j<cur_cloud->size(); j++)
  {
    distance=sqrt(cur_cloud->points[j].x*cur_cloud->points[j].x+cur_cloud->points[j].y*cur_cloud->points[j].y+cur_cloud->points[j].z*cur_cloud->points[j].z);
    if(distance<1)
      {
         
        cur_cloud->points[j].x=100000;
        cur_cloud->points[j].y=100000;
        cur_cloud->points[j].z=100000;
      }
  }*/
   //for debug current cloud
   pcl::transformPointCloud (*cur_cloud, *cur_cloud, t_world_uav );
    toROSMsg(*cur_cloud, cloud_msg2);
        cloud_msg2.header.frame_id ="odom";
        cloud_msg2.header.stamp = ros::Time::now();
        pcPublisher1.publish(cloud_msg2);

      if(do_once)
    {
    old_cloud=cur_cloud;
    do_once=false;
    }
   
  std::cout <<"icp starting..."<< std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // Generalized
  pcl::PointCloud<pcl::PointXYZ> final_cloud;
  icp.setMaxCorrespondenceDistance(0.1);
  icp.setMaximumIterations (1);
  icp.setInputSource(cur_cloud);
  icp.setInputTarget(old_cloud);
  icp.align(final_cloud);
  double score=0;
  trans_final = icp.getFinalTransformation();
  score=icp.getFitnessScore();
  std::cout <<"Score is:"<<score<<std::endl;
  if(score>1 or (counter>=80 && counter<120) or (counter>=185) )
    cout<<"bad converge!!"<<endl;
  else{
   
      pcl::transformPointCloud (*cur_cloud, *cur_cloud, trans_final);
      total_cloud+=*cur_cloud;
      toROSMsg(total_cloud, cloud_msg1);
      cloud_msg1.header.frame_id ="uav_ref";
      cloud_msg1.header.stamp = ros::Time::now();
      pcPublisher.publish(cloud_msg1);
     }

  cout<<"counter is "<<counter<<endl;
  *old_cloud=*cur_cloud;
   counter++;
 pc_avaliable=false;
}


int main (int argc, char** argv)
{
 ros::init (argc, argv, "UAV_Cloud_Reg_ICP");
 ros::NodeHandle n;
 pcSubscriber  = n.subscribe("/velodyne_points_updated",        200, pcCallback_uav);
 
 pcPublisher1 = n.advertise<sensor_msgs::PointCloud2>("cur_cloud",1,false);
  pcSubscriber2  = n.subscribe("/imu/data",        200, imuCallback_uav);

 pcPublisher = n.advertise<sensor_msgs::PointCloud2>("total_cloud",1,false);
  odomSubscriber = n.subscribe("/odometry/filtered",   200, odomCallback_uav);

 printf("Starting ... \n" );
 ros::Rate r(100);
 while(n.ok()){
   
  if (pc_avaliable){
    process(); 
   }

    ros::spinOnce();
    r.sleep();
  }
  return (0);
}


/* Two Map  Merging  Algorithm Which Have Diffirent Angle of Views in REAL DATA */
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
Publisher pcPublisher2;

Eigen::Matrix4f t_offset = Eigen::Matrix4f::Identity();

Eigen::Matrix4f trans_final = Eigen::Matrix4f::Identity();

Eigen::Matrix4f trans_yaw = Eigen::Matrix4f::Identity();
Eigen::Matrix4f trans_roll= Eigen::Matrix4f::Identity();
Eigen::Matrix4f trans_pitch = Eigen::Matrix4f::Identity();

Eigen::Matrix4f  Rt = Eigen::Matrix4f::Identity();


pcl::PointCloud<pcl::PointXYZ>::Ptr ugv_cloud          (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ugv_cloud_trans         (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ugv_cloud_final        (new pcl::PointCloud<pcl::PointXYZ>);


pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud           (new pcl::PointCloud<pcl::PointXYZ>);


pcl::PointCloud<pcl::PointXYZ>::Ptr uav_cloud           (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 cloud_msg1;
sensor_msgs::PointCloud2 cloud_msg2;

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
 cloud_msg1.header.stamp = laserCloudMsg->header.stamp;
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
if(do_multiple<2){
 
    double yaw = -111*3.14/180;
    double roll = 90*3.14/180;    
	trans_yaw(0,0)=cos(yaw);
	trans_yaw(0,1)=-sin(yaw);
	trans_yaw(0,2)=0;
	trans_yaw(0,3)=0;


	trans_yaw(1,0)=sin(yaw);
	trans_yaw(1,1)=cos(yaw);
	trans_yaw(1,2)=0;
	trans_yaw(1,3)=0;

	trans_yaw(2,0)=0;
	trans_yaw(2,1)=0;
	trans_yaw(2,2)=1;
	trans_yaw( 2,3)=0;

	trans_yaw(3,0)=0;
	trans_yaw(3,1)=0;
	trans_yaw(3,2)=0;
	trans_yaw(3,3)=1;


   	trans_roll(0,0)=1;
	trans_roll(0,1)=0;
	trans_roll(0,2)=0;
	trans_roll(0,3)=0;


	trans_roll(1,0)=0;
	trans_roll(1,1)=cos(roll);
	trans_roll(1,2)=-sin(roll);
	trans_roll(1,3)=0;

	trans_roll(2,0)=0;
	trans_roll(2,1)=sin(roll);
	trans_roll(2,2)=cos(roll);
	trans_roll( 2,3)=0;

	trans_roll(3,0)=0;
	trans_roll(3,1)=0;
	trans_roll(3,2)=0;
	trans_roll(3,3)=1;
     
    t_offset(0,0)=1;
	t_offset(0,1)=0;
	t_offset(0,2)=0;
	t_offset(0,3)=26;

	t_offset(1,0)=0;
	t_offset(1,1)=1;
	t_offset(1,2)=0;
	t_offset(1,3)=-27;

	t_offset(2,0)=0;
	t_offset(2,1)=0;
	t_offset(2,2)=1;
	t_offset( 2,3)=0;

	t_offset(3,0)=0;
	t_offset(3,1)=0;
	t_offset(3,2)=0;
	t_offset(3,3)=1;

    
     
    cout<<do_multiple<<endl;
    do_multiple++;
    //sets initials by hand

    pcl::transformPointCloud (*ugv_cloud, *ugv_cloud, t_offset*trans_yaw*trans_roll);

    t_offset(0,0)=1;
	t_offset(0,1)=0;
	t_offset(0,2)=0;
	t_offset(0,3)=0;

	t_offset(1,0)=0;
	t_offset(1,1)=1;
	t_offset(1,2)=0;
	t_offset(1,3)=0;

	t_offset(2,0)=0;
	t_offset(2,1)=0;
	t_offset(2,2)=1;
	t_offset( 2,3)=32;

	t_offset(3,0)=0;
	t_offset(3,1)=0;
	t_offset(3,2)=0;
	t_offset(3,3)=1;

	pcl::transformPointCloud (*uav_cloud, *uav_cloud, t_offset);

    toROSMsg(*ugv_cloud, cloud_msg1);
    cloud_msg1.header.frame_id ="odom";
    pcPublisher.publish(cloud_msg1);
    //finds  small transformations with using G-ICP. It generates map with smaller merging error 
	std::cout <<"icp starting..."<< std::endl;
	pcl::PointCloud<pcl::PointXYZ> final_cloud;
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // Generalized
	icp.setMaxCorrespondenceDistance(2);
	icp.setMaximumIterations (1);
	icp.setInputSource(ugv_cloud);
	icp.setInputTarget(uav_cloud);
	icp.align(final_cloud);
	trans_final = icp.getFinalTransformation();
	pcl::transformPointCloud (*ugv_cloud, *ugv_cloud, trans_final);
    cout<<"d "<<trans_final<<endl;
    double score=icp.getFitnessScore();
    std::cout <<"Score is:"<<score<<std::endl;

    //sums UAV's and UGV's transformed cloud
	*total_cloud=*ugv_cloud +*uav_cloud;
	toROSMsg(*total_cloud, cloud_msg2);

	cloud_msg2.header.frame_id ="odom";
	cloud_msg2.header.stamp = ros::Time::now();
	pcPublisher2.publish(cloud_msg2);
   
   
    pc2_avaliable=false;
    pc_avaliable=false;
}
else
{
    //due to high computational work loads in ICP, after two times, automaticly sums two map by latest transformation.
    double yaw = -111*3.14/180;
    double roll = 90*3.14/180;
    
	trans_yaw(0,0)=cos(yaw);
	trans_yaw(0,1)=-sin(yaw);
	trans_yaw(0,2)=0;
	trans_yaw(0,3)=0;


	trans_yaw(1,0)=sin(yaw);
	trans_yaw(1,1)=cos(yaw);
	trans_yaw(1,2)=0;
	trans_yaw(1,3)=0;

	trans_yaw(2,0)=0;
	trans_yaw(2,1)=0;
	trans_yaw(2,2)=1;
	trans_yaw( 2,3)=0;

	trans_yaw(3,0)=0;
	trans_yaw(3,1)=0;
	trans_yaw(3,2)=0;
	trans_yaw(3,3)=1;


   	trans_roll(0,0)=1;
	trans_roll(0,1)=0;
	trans_roll(0,2)=0;
	trans_roll(0,3)=0;


	trans_roll(1,0)=0;
	trans_roll(1,1)=cos(roll);
	trans_roll(1,2)=-sin(roll);
	trans_roll(1,3)=0;

	trans_roll(2,0)=0;
	trans_roll(2,1)=sin(roll);
	trans_roll(2,2)=cos(roll);
	trans_roll( 2,3)=0;

	trans_roll(3,0)=0;
	trans_roll(3,1)=0;
	trans_roll(3,2)=0;
	trans_roll(3,3)=1;

     
    t_offset(0,0)=1;
	t_offset(0,1)=0;
	t_offset(0,2)=0;
	t_offset(0,3)=26;

	t_offset(1,0)=0;
	t_offset(1,1)=1;
	t_offset(1,2)=0;
	t_offset(1,3)=-27;

	t_offset(2,0)=0;
	t_offset(2,1)=0;
	t_offset(2,2)=1;
	t_offset( 2,3)=0;

	t_offset(3,0)=0;
	t_offset(3,1)=0;
	t_offset(3,2)=0;
	t_offset(3,3)=1;

 
    pcl::PointCloud<pcl::PointXYZ> uav_cloud_dif;
    pcl::transformPointCloud (*ugv_cloud, *ugv_cloud, t_offset*trans_yaw*trans_roll);

    t_offset(0,0)=1;
	t_offset(0,1)=0;
	t_offset(0,2)=0;
	t_offset(0,3)=0;

	t_offset(1,0)=0;
	t_offset(1,1)=1;
	t_offset(1,2)=0;
	t_offset(1,3)=0;

	t_offset(2,0)=0;
	t_offset(2,1)=0;
	t_offset(2,2)=1;
	t_offset( 2,3)=32;

	t_offset(3,0)=0;
	t_offset(3,1)=0;
	t_offset(3,2)=0;
	t_offset(3,3)=1;

	pcl::transformPointCloud (*uav_cloud, uav_cloud_dif, t_offset);

    pcl::transformPointCloud (*ugv_cloud, *ugv_cloud, trans_final);
    
	*total_cloud=*ugv_cloud+uav_cloud_dif;
	toROSMsg(*total_cloud, cloud_msg2);

	cloud_msg2.header.frame_id ="odom";
	cloud_msg2.header.stamp = ros::Time::now();
	pcPublisher2.publish(cloud_msg2);
    cout<<do_multiple<<endl;
    do_multiple++;
    
    
    pc_avaliable=false; 
}
}

int main (int argc, char** argv){
 ros::init (argc, argv, "real_time_cloud_merge");
 ros::NodeHandle n;
 pcSubscriber  = n.subscribe("/uav/total_cloud",        200, pcCallback_uav);
 pcSubscriber1 = n.subscribe("/ugv/total_cloud",   200, pcCallback_ugv);
 odomSubscriber = n.subscribe("/uav/odometry/filtered_v1",   200, odomCallback_uav);
 odomSubscriber1 = n.subscribe("/ugv/odometry/filtered_v1",   200, odomCallback_ugv);

 pcPublisher = n.advertise<sensor_msgs::PointCloud2>("initialised_cloud",1,false);

 pcPublisher2 = n.advertise<sensor_msgs::PointCloud2>("merged_cloud",1,false);
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

// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
// PCL LIBRARY
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
 
#include <pcl/common/common.h>


#include <fstream>
#include <iostream>
#include <vector>
#include <ctime>
#include <string>

#define angle_horizontal_resolution 0.4
#define angle_horizontal_view 30
#define SIZE 900000
#define resolution 0.2

pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud          (new pcl::PointCloud<pcl::PointXYZ>);



pcl::PointCloud<pcl::PointXYZ>::Ptr ugv_cloud          (new pcl::PointCloud<pcl::PointXYZ>);



pcl::PointCloud<pcl::PointXYZ>::Ptr uav_cloud          (new pcl::PointCloud<pcl::PointXYZ>);

 
bool pc_avaliable = false, pc_avaliable1 = false;
double uav_count=1;
int uav_number=1;
double uav_all_distance=0;
double a,b,c;
double uav_all_av_distance[SIZE]={0};
double uav_occupancy_volume[SIZE]={0};



double ugv_count=1;
int ugv_number=1;
double ugv_all_distance=0;
double ugv_all_av_distance[SIZE]={0};
double ugv_occupancy_volume[SIZE]={0};

double total_count=1;
int total_number=1;
double total_all_distance=0;
double total_all_av_distance[SIZE]={0};
double total_occupancy_volume[SIZE]={0};


long double maxPointVolume=(1/resolution)*(1/resolution)*(1/resolution)*70*70*7;   //(1/resolution)*(angle_horizontal_view/angle_horizontal_resolution)*16*70*70*7;
void pcCallback_uav(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
  uav_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*laserCloudMsg, *uav_cloud);
  pc_avaliable=true;
}
void pcCallback_ugv(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
  ugv_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*laserCloudMsg, *ugv_cloud);
  pc_avaliable1=true;
}
void pcCallback_total(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
  total_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*laserCloudMsg, *total_cloud);
}


void process(int type, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
if(type==1){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered         (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  a=maxPoint.x - minPoint.x;
  b= maxPoint.y - minPoint.y;
  c= maxPoint.z - minPoint.z;
 // maxPointVolume=8000000;   //(angle_horizontal/angle_horizontal_resolution)*a*b*c;
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
    
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (1.0f, 1.0f, 1.0f);
  sor.filter (*cloud_filtered);
  size_t A_points = cloud->size();  
  size_t B_points = cloud_filtered->size();  
  uav_occupancy_volume[uav_number]=100*A_points/maxPointVolume;
     
  //std::cout<<"Occupancy in Volume:"<<occupancy_volume[number] <<std::endl;
  //std::cerr << "PointCloud after filtering: " << B_points<< "without filtering data "<<A_points<<std::endl;
  pcl::PointXYZ searchPoint;
  double distance = 0;
  for(size_t j=0; j<B_points; j++){
  	searchPoint.x = cloud_filtered->points[j].x;
  	searchPoint.y = cloud_filtered->points[j].y;
  	searchPoint.z = cloud_filtered->points[j].z;

     // K nearest neighbor search

  int K = 10;
  double sum=0;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

    
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){  
          sum += pointNKNSquaredDistance[i];}
    		}
             distance +=sum/K;
    }
          
  std::cout<<"distance  is:"<< distance/B_points<< std::endl;
  uav_all_distance+=(distance/B_points);
  uav_all_av_distance[uav_number]=uav_all_distance/uav_count;
  std::cout<<"all av  distance is: "<<uav_all_av_distance[uav_number]<< std::endl;
  uav_number++;
  uav_count++;
    
 
  std::ofstream myfile;
  myfile.open ("/home/roblab/hetero_ws/src/hetero_mapping/src/uav_distance_av.txt");
  for(int i=0; i<uav_number; i++){
       
    myfile <<uav_all_av_distance[i]<<",";
      
    }
    myfile.close();


    myfile.open ("/home/roblab/hetero_ws/src/hetero_mapping/src/uav_occupancy_volume.txt");
   for(int i=0; i<uav_number; i++){
       
      myfile <<uav_occupancy_volume[i]<<",";
      
    }
    myfile.close();
  
  }

  if(type==2){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered         (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  a=maxPoint.x - minPoint.x;
  b= maxPoint.y - minPoint.y;
  c= maxPoint.z - minPoint.z;
 // maxPointVolume=8000000; //(angle_horizontal/angle_horizontal_resolution)*a*b*c;
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
    
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (1.0f, 1.0f, 1.0f);
  sor.filter (*cloud_filtered);
  size_t A_points = cloud->size();  
  size_t B_points = cloud_filtered->size();  
  ugv_occupancy_volume[ugv_number]=100*A_points/maxPointVolume;
     
  //std::cout<<"Occupancy in Volume:"<<occupancy_volume[number] <<std::endl;
  //std::cerr << "PointCloud after filtering: " << B_points<< "without filtering data "<<A_points<<std::endl;
  pcl::PointXYZ searchPoint;
  double distance = 0;
  for(size_t j=0; j<B_points; j++){
    searchPoint.x = cloud_filtered->points[j].x;
    searchPoint.y = cloud_filtered->points[j].y;
    searchPoint.z = cloud_filtered->points[j].z;

     // K nearest neighbor search

  int K = 10;
  double sum=0;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

    
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){  
          sum += pointNKNSquaredDistance[i];}
        }
             distance +=sum/K;
    }
          
  std::cout<<"distance  is:"<< distance/B_points<< std::endl;
  ugv_all_distance+=(distance/B_points);
  ugv_all_av_distance[ugv_number]=ugv_all_distance/ugv_count;
  std::cout<<"all av  distance is: "<<ugv_all_av_distance[ugv_number]<< std::endl;
  ugv_number++;
  ugv_count++;
    
 
  std::ofstream myfile;
  myfile.open ("/home/roblab/hetero_ws/src/hetero_mapping/src/ugv_distance_av.txt");
  for(int i=0; i<ugv_number; i++){
       
    myfile <<ugv_all_av_distance[i]<<",";
      
    }
    myfile.close();


    myfile.open ("/home/roblab/hetero_ws/src/hetero_mapping/src/ugv_occupancy_volume.txt");
   for(int i=0; i<ugv_number; i++){
       
      myfile <<ugv_occupancy_volume[i]<<",";
      
    }
    myfile.close();
  
  }

    if(type==3){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered         (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  a=maxPoint.x - minPoint.x;
  b= maxPoint.y - minPoint.y;
  c= maxPoint.z - minPoint.z;
  //maxPointVolume=8000000 ;//(angle_horizontal/angle_horizontal_resolution)*a*b*c;
  std::cout<<"a:" <<a<<"b:"<<b<<"c:"<<c<<std::endl;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
    
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (1.0f, 1.0f, 1.0f);
  sor.filter (*cloud_filtered);
  size_t A_points = cloud->size();  
  size_t B_points = cloud_filtered->size();  
  total_occupancy_volume[ total_number]=100*A_points/maxPointVolume;
     
  //std::cout<<"Occupancy in Volume:"<<occupancy_volume[number] <<std::endl;
  //std::cerr << "PointCloud after filtering: " << B_points<< "without filtering data "<<A_points<<std::endl;
  pcl::PointXYZ searchPoint;
  double distance = 0;
  for(size_t j=0; j<B_points; j++){
    searchPoint.x = cloud_filtered->points[j].x;
    searchPoint.y = cloud_filtered->points[j].y;
    searchPoint.z = cloud_filtered->points[j].z;

     // K nearest neighbor search

  int K = 10;
  double sum=0;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

    
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){  
          sum += pointNKNSquaredDistance[i];}
        }
             distance +=sum/K;
    }
          
  std::cout<<"distance  is:"<< distance/B_points<< std::endl;
  total_all_distance+=(distance/B_points);
  total_all_av_distance[total_number]=total_all_distance/total_count;
  std::cout<<"all av  distance is: "<<total_all_av_distance[total_number]<< std::endl;
  total_number++;
  total_count++;
    
 
  std::ofstream myfile;
  myfile.open ("/home/roblab/hetero_ws/src/hetero_mapping/src/total_distance_av.txt");
  for(int i=0; i<total_number; i++){
       
    myfile <<total_all_av_distance[i]<<",";
      
    }
    myfile.close();


    myfile.open ("/home/roblab/hetero_ws/src/hetero_mapping/src/total_occupancy_volume.txt");
   for(int i=0; i<total_number; i++){
       
      myfile <<total_occupancy_volume[i]<<",";
      
    }
    myfile.close();
  
  }




}



int main (int argc, char** argv)
{
  ros::init (argc, argv, "cloud_filter");
  ros::NodeHandle n;
  ros::Subscriber pcSubscriber  = n.subscribe("/uav/octomap_point_cloud_centers", 200, pcCallback_uav);
  ros::Subscriber pcSubscriber1  = n.subscribe("/ugv/octomap_point_cloud_centers", 200, pcCallback_ugv);
  ros::Subscriber pcSubscriber2  = n.subscribe("/total_cloud", 200, pcCallback_total);

  printf("Starting ... \n" );
  ros::Rate r(1);
  while(n.ok()){
   
  if (pc_avaliable && pc_avaliable1){
    process(1,uav_cloud); 
    process(2,ugv_cloud); 
    process(3,total_cloud); 

    
  }

    ros::spinOnce();
    r.sleep();
}


  return 0;
}

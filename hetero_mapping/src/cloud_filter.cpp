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


#define angle_horizontal_resolution 0.2
#define angle_horizontal 360
#define SIZE 100000


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud          (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered         (new pcl::PointCloud<pcl::PointXYZ>);
bool pc_avaliable = false;
double count=1;
int number=1;
double all_distance=0;
double a,b,c;
double all_av_distance[SIZE]={0};
double occupancy_volume[SIZE]={0};

long double maxPointVolume;
void pcCallback_uav(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*laserCloudMsg, *cloud);
  pc_avaliable=true;
}
void process(){

pcl::PointXYZ minPoint, maxPoint;
pcl::getMinMax3D(*cloud, minPoint, maxPoint);
a=maxPoint.x - minPoint.x;
b= maxPoint.y - minPoint.y;
c= maxPoint.z - minPoint.z;
maxPointVolume=(angle_horizontal/angle_horizontal_resolution)*a*b*c;
std::cout<<"Volume:"<<a*b*c<<std::endl;
std::cout<<"Max point in Volume:"<<maxPointVolume<<std::endl;


pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (cloud);
  
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (1.0f, 1.0f, 1.0f);
  sor.filter (*cloud_filtered);
  size_t A_points = cloud->size();  
  size_t B_points = cloud_filtered->size();  
  occupancy_volume[number]=100*A_points/maxPointVolume;
   
  std::cout<<"Occupancy in Volume:"<<occupancy_volume[number] <<std::endl;
  std::cerr << "PointCloud after filtering: " << B_points<< "without filtering data "<<A_points<<std::endl;
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

  		/*std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;*/

  		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  		{
   		 for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
    	        /*std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;*/
                sum += pointNKNSquaredDistance[i];}
  		}

            distance +=sum/K;
       //     std::cout<<"avarage is "<<sum/K<< std::endl;
  }
        
        std::cout<<"distance  is:"<< distance/B_points<< std::endl;
        all_distance+=(distance/B_points);
        all_av_distance[number]=all_distance/count;
        std::cout<<"all av  distance is: "<<all_av_distance[number]<< std::endl;
        number++;
        count++;
   
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "cloud_filter");
  ros::NodeHandle n;
  ros::Subscriber pcSubscriber  = n.subscribe("/total_cloud", 200, pcCallback_uav);

  printf("Starting ... \n" );
  ros::Rate r(1);
  while(n.ok()){
   
  if (pc_avaliable){
    process(); 
   }
    std::ofstream myfile;
    myfile.open ("/home/roblab/hetero_ws/src/hetero_mapping/src/distance_av.txt");
   for(int i=0; i<number; i++){
   	   
  		myfile <<all_av_distance[i]<<",";
  		
  	}
    myfile.close();


    myfile.open ("/home/roblab/hetero_ws/src/hetero_mapping/src/occupancy_volume.txt");
   for(int i=0; i<number; i++){
   	   
  		myfile <<occupancy_volume[i]<<",";
  		
  	}
    myfile.close();

    ros::spinOnce();
    r.sleep();
  }


  return 0;
}

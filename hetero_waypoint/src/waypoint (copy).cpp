#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <sstream>

// initialize variables

#define waypoint_number     6
#define coefficient         2.5 // if you decrease inner poly becomes more smaller
#define polygon_number      2   //TO DO;  if need to create more than two
#define time_out            60 //120 second waits for sending goal again

geometry_msgs::PointStamped map_goal, UTM_goal;
geometry_msgs::PoseStamped map_pose, center_pose;
geometry_msgs::PolygonStamped polyStamp, polyStamp1;
geometry_msgs::PointStamped UTM_point, map_point;

// TO DO , Read from text
double waypoints[waypoint_number][2]={ {41.1053558192,29.0235311762},{41.1053331777, 29.0237283253},{41.1052168137,29.023746603},
{41.1051661528,29.0238162789}, {41.1050842003,29.0237917683},{41.1051709292,29.0235311272}};

double newgoals[waypoint_number][2]; // new created goals
double goals[waypoint_number*polygon_number][2]={0}; // current and new created goals


float center_x=0;  // centers
float center_y=0;  // centers


std::string result_move_base;
std::string utm_zone;
geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("odom", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            
        }
    }
    return map_point_output;
}
geometry_msgs::PoseStamped  findCenter()
{
    float total_x=0;
    float total_y=0;
    for(int i=0; i< waypoint_number; i++)
    {
        total_x += goals[i][0];
        total_y += goals[i][1];
    }
    //Average numbers in waypoints
    center_x=total_x/waypoint_number;
    center_y=total_y/waypoint_number;

  // ROS_INFO(" center of plant x:  %f  y: %f  ", center[min_j][0], center[0][min_j] );
    ROS_INFO(" center of plant x:  %f  y: %f  ", center_x,center_y );


    center_pose.header.frame_id = "odom";
    center_pose.header.stamp = ros::Time::now();
    center_pose.pose.position.x = center_x;
    center_pose.pose.position.y = center_y;
    center_pose.pose.position.z = 0;
    center_pose.pose.orientation.x = 0;
    center_pose.pose.orientation.y = 0;
    center_pose.pose.orientation.z = 0;
    center_pose.pose.orientation.w= 1;

    return center_pose;
}
void createNewgoals()
 {
    double slope[waypoint_number];
    double dist_to_center[waypoint_number];
    for ( int i=0; i< waypoint_number ; i++ )
    {      
        //calculates distance from goals to the center
        dist_to_center[i] = sqrt (pow((goals[i][1]-center_y),2)+pow((goals[i][0]-center_x),2));
  
    }
      for(int i=0; i<waypoint_number; i++)
    {  
        //calculates slope from goals to the center
        slope[i]= atan2((goals[i][1]-center_y),(goals[i][0]-center_x));
        //uses slope and distance to the center to create new goals
        newgoals[i][0]= goals[i][0]-cos(slope[i])*(dist_to_center[i]/coefficient) ;
        newgoals[i][1]= goals[i][1]-sin(slope[i])*(dist_to_center[i]/coefficient );
        //ROS_INFO(" new goals  x: %f, y: %f  ",newgoals[i][0],newgoals[i][1]);
    }


 }
void createPolygon(ros::NodeHandle n)
{   
    ros::Publisher pub_poly=n.advertise<geometry_msgs::PolygonStamped>("/uav/waypoint/polygon",10);
    ros::Publisher pub_poly1=n.advertise<geometry_msgs::PolygonStamped>("/uav/waypoint/polygon_inner",10);
    ros::Publisher pub_center = n.advertise<geometry_msgs::PoseStamped>("/uav/waypoint/center", 1000);
    
    //given waypoints are translated to the polygon 

    for(int i=0; i<waypoint_number; i++)
    {
        UTM_goal = latLongtoUTM(waypoints[i][0],waypoints[i][1]);
        map_goal = UTMtoMapPoint(UTM_goal);
        goals[i][0]=map_goal.point.x;
        goals[i][1]=map_goal.point.y;
        geometry_msgs::Point32 tl;
        tl.x=goals[i][0];
        tl.y=goals[i][1];
        polyStamp.polygon.points.reserve(10); // ensure that there will be space for at least 10 points
        polyStamp.polygon.points.push_back(tl);
        ROS_INFO("%f and %f", polyStamp.polygon.points[i].x,goals[i][1] );
    } 
    //send the center
    pub_center.publish(findCenter());
    
    //creates new waypoints according to the center
    createNewgoals();
    
    //created new waypoints are translated to the polygon 
    for(int i=0; i<waypoint_number; i++)
    {
        
        geometry_msgs::Point32 tl;
        tl.x=newgoals[i][0];
        tl.y=newgoals[i][1];
        polyStamp1.polygon.points.reserve(10); // ensure that there will be space for at least 10 points
        polyStamp1.polygon.points.push_back(tl);
        goals[i+waypoint_number][0]=newgoals[i][0];
        goals[i+waypoint_number][1]=newgoals[i][1];
    } 

        polyStamp.header.frame_id="world";
        polyStamp.header.stamp=ros::Time::now();
        polyStamp1.header.frame_id="world";
        polyStamp1.header.stamp=ros::Time::now();
        //sends to Rviz, ten times to catch the packet
    for(int i=0; i<10 ;i++)
        {
            pub_poly.publish(polyStamp);
            pub_poly1.publish(polyStamp1);
        }
        ROS_INFO("Polygons are published");

}


geometry_msgs::PoseStamped   buildGoal(float x, float y)
{
    geometry_msgs::PoseStamped Pose;
    //Specify what frame we want the goal to be published in
    Pose.header.frame_id = "odom";
    Pose.header.stamp = ros::Time::now();
    // Specify x and y goal
    Pose.pose.position.x=x;
    Pose.pose.position.y=y;
    // towards on front    
    Pose.pose.orientation.x = 0.0;
    Pose.pose.orientation.y = 0.0;
    Pose.pose.orientation.z = 0.0;
    Pose.pose.orientation.w = 1.0;
    return Pose;
}

void movebaseCallback (move_base_msgs::MoveBaseActionResult result)
{ 
    result_move_base=result.status.text;
}

bool checkSucces()
  { 
    ros::spinOnce();
   
    if( result_move_base=="Goal reached.")
       { ROS_INFO("succes"  );
        return true; 
       }
    else 
        return false;      
  }

int main(int argc, char** argv)
{   
    //initiate node called waypoint_controller
    ros::init(argc, argv, "uav_waypoint_controller"); 
    ros::NodeHandle n;
    //sends goals to UAV-move base
    ros::Publisher pub_uav=n.advertise<geometry_msgs::PoseStamped>("/uav/move_base_simple/goal",10);
    ros::Rate rate(1); 
    ROS_INFO("Initiated waypoint_controller node");

    int n_goal=0;
    bool is_finished=false;
    // creates   inner and outter polygons
    createPolygon(n);

    while(ros::ok() )
    {  
 
        result_move_base=" ";
        bool check= false;
        ros::spinOnce();
        //if n_goal is smaller than max n goal, this line publish goals to the move base
        if(n_goal<waypoint_number*polygon_number) 
            {
                ROS_INFO("Sending goal to : [%d] x: %f y: %f ", n_goal,goals[n_goal][0],goals[n_goal][1]);
                pub_uav.publish(buildGoal(goals[n_goal][0],goals[n_goal][1]))  ;
               
            }
        //if n_goal reaches its limit than stops the code.
        else
            {
            is_finished=true;
            ROS_INFO("Congrats All Goals Are Done!!");
            break;
            }

        ros::Subscriber sub2 = n.subscribe("/uav/move_base/result", 10, movebaseCallback);
        double start =ros::Time::now().toSec();
        double secs=0;
        //this loop checks if goal is reached or mission is finished, if not it will wait.
        while(!check && ros::ok() && !is_finished )
        {    
            ros::spinOnce();
            check= checkSucces();
            secs=ros::Time::now().toSec()-start;
            //if seconds that waiting move base to reachs its point exceed its time than send again. 
            if(secs> time_out)
            { 
              ROS_INFO(" **Time Out** Sending goal to : [%d] x: %f y: %f ", n_goal,goals[n_goal][0],goals[n_goal][1]);
              pub_uav.publish(buildGoal(goals[n_goal][0],goals[n_goal][1]))  ;
            }
            rate.sleep();
        }
        n_goal++; // if goal reached , increase by one
        ros::spinOnce();
        rate.sleep(); 
    }
    return 0;
}

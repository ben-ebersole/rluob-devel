#include <iostream>
#include <string>
#include <math.h>
#include <algorithm>
#include <iterator>
#include <limits>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Quaternion.h"

//#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

using namespace std;
moveit::planning_interface::MoveGroup *robot;
geometry_msgs::PoseStamped can_start;
geometry_msgs::PoseStamped can_finish;

void callback_add (const geometry_msgs::PoseStamped input)
{
	can_start = input;
	
	/*tf::TransformListener listener;
  
  try
  {
      listener.transformPose("/base_link", input, can_start);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }*/
}

void callback_rem (const geometry_msgs::PoseStamped input)
{ 
	can_finish = input;
	/*tf::TransformListener listener;
    
	try
  {
      listener.transformPose("/base_link", input, can_finish);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }*/
}

void wait()
{
  std::cout << "Press ENTER to repeat pick/place";
  std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_place_node");
  ros::NodeHandle n;
  
  ros::Subscriber sub_add = n.subscribe("/objects/com/added_object", 1, callback_add);
	ros::Subscriber sub_rem = n.subscribe("/objects/com/removed_object", 1, callback_rem);
	
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  robot = new moveit::planning_interface::MoveGroup("sia10");
  robot->setStartStateToCurrentState();
  robot->setPlannerId("RRTConnectkConfigDefault");

  robot->setPoseReferenceFrame("/base_link");
  robot->setPlannerId("RRTkConfigDefault");
  
  robot->setPlanningTime(30.0);
  
  robot->setNamedTarget("home");
	robot->move();
	
	//can_start.header.frame_id = "/base_link";
	//can_finish.header.frame_id = "/base_link";
	
	/*for ( int i = 0; i == 10; i++)
	{
		ros::spinOnce();		
	}*/
	  
  while (ros::ok())
  {
  	tf::TransformListener listener;
    bool fail;
    
    while (fail == true)
    {
    	fail = false;
    	ros::spinOnce();
			try
			{
		    listener.transformPose("/base_link", can_start, can_start);
		    listener.transformPose("/base_link", can_finish, can_finish);
			}
			catch (tf::TransformException &ex)
			{
		  	ROS_ERROR("%s",ex.what());
		  	ros::Duration(1.0).sleep();
		  	fail = true;
			}
  	}
  	
  	robot->setPoseTarget(can_start);
		cout << "Moving to initial can position..." << endl;
		cout << can_start << endl;
		robot->move();
		
		robot->setPoseTarget(can_finish);
		cout << "Moving to final can position..." << endl;
		cout << can_finish << endl;
		robot->move();
		
		robot->setNamedTarget("home");
		robot->move();

  	wait();
  }  
  return 0;
}

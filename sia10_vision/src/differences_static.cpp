#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <numeric>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include "change_detector/cloud_functions.cpp"
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <algorithm>
#include <eigen3/Eigen/src/Geometry/Transform.h>

using namespace std;
typedef pcl::PointXYZ PointT;

std::string pcdDirectory;
bool needPointCloud = false;
ros::NodeHandle *nh;
pcl::PointCloud<PointT>::Ptr current_cloud (new pcl::PointCloud<PointT>);
boost::shared_ptr< tf::TransformListener> tf_listener;

string intToString(int i){
    stringstream ss;
    ss << i; 
    return ss.str(); 
}

bool fileExists(string filename)
{
  std::ifstream ifile(filename.c_str());
  return ifile;
}

/*-------------------------------------------------------
 * Function:    cloud_callback
 * 
 * Callback target function from /camera/depth/points subscription. 
 * Function modifies global variable 'current_cloud' to a pcl::PointCloud<PointT>::Ptr representation
 * of the most up to date cloud available from the ROS topic
 * 
 * @input:      sensor_msgs::PointCloud2ConstPtr& from node callback
 *  
 * ---------------------------------------------------------*/

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{   
  //Check if we even need to do anything with the incoming point cloud
  if (needPointCloud == true )
  {  
      pcl::fromROSMsg(*input,*current_cloud);
      std::vector<int> indicies;
      pcl::removeNaNFromPointCloud(*current_cloud, *current_cloud, indicies);
      needPointCloud = false;
  } 
}

/*-------------------------------------------------------
 *  Function:   getPointCloudWait
 * 
 * Waits for a new point cloud to be recieved from cloud_callback(). 
 * On return global pcl::PointCloud<PointT>::Ptr current_cloud will be updated.
 * 
 * ---------------------------------------------------------*/

void getPointCloudWait(){
  //Set the global flag and wait around untill we have a new cloud
  needPointCloud = true;
  while (needPointCloud == true) {
    //std::cout << "Waiting for new cloud" << std::endl;
    ros::Duration(1).sleep();
    ros::spinOnce();
  }
}

/*-------------------------------------------------------
 *  Function:   wait
 * 
 * Waits for user input to continue 
 * 
 * ---------------------------------------------------------*/

void wait()
{
  std::cout << "Press ENTER to repeat scan...";
  std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
}

void sleepok(int t, ros::NodeHandle &nh)
{
  if (nh.ok())
    sleep(t);
}

int main(int argc, char** argv)
{
		pcdDirectory = ros::package::getPath("sia10_vision");
		pcdDirectory = pcdDirectory.append("/pcd_data/");
		ros::init(argc, argv, "change_detector");
		nh = new ros::NodeHandle;
		//ros::Subscriber sub = nh->subscribe("/camera/depth/points", 1, cloud_callback);    
		pub = nh->advertise<sensor_msgs::PointCloud2>("/camera/depth/points_fixed", 5, true);
		pub_original = nh->advertise<sensor_msgs::PointCloud2>("/camera/depth/original_cloud", 5, true);
		pub_diff_removed = nh->advertise<sensor_msgs::PointCloud2>("/camera/depth/diff_removed", 5, true);
		pub_diff_added = nh->advertise<sensor_msgs::PointCloud2>("/camera/depth/diff_added", 5, true);
		 
		tf_listener.reset(new tf::TransformListener);
		sleepok(1, *nh);         
		
	while (ros::ok())
	{                              
		int i = -1;
		std::string fileName = "";
		do{
		  i++;
		  fileName = pcdDirectory;
		  fileName = fileName.append("snapshot").append(intToString(i)).append(".pcd");
		}while(fileExists(fileName));
		                                  
		//std::cout << "Waiting for cloud . . . "; std::cout.flush();
		//getPointCloudWait();
		//std::cout << "Complete" << std::endl; std::cout.flush();
		                                  
		/*if(i == 0){
		  pcl::io::savePCDFileASCII (fileName.c_str(), *current_cloud);
		  std::cout << "No matching pcd file could be found, creating initial scan to " << fileName  << std::endl;
		  sensor_msgs::PointCloud2 output;
		  current_cloud = downSampleByVoxel(current_cloud);
		  pcl::toROSMsg(*current_cloud, output);
		  output.header.frame_id = "/map";
		  pub.publish (output);*/
		                                  
		//}else{                                   
		  i--;  //If you want to use the last point cloud collected each time
		  // i=0;    //If you want to use the first point cloud collected
		  fileName = pcdDirectory;
		  fileName = fileName.append("snapshot").append(intToString(i)).append(".pcd");
		  pcl::io::loadPCDFile<PointT> (fileName.c_str(), *current_cloud);
		  
		  i--;
		  fileName = pcdDirectory;
		  fileName = fileName.append("snapshot").append(intToString(i)).append(".pcd");
		  pcl::PointCloud<PointT>::Ptr original_cloud (new pcl::PointCloud<PointT>);
		  pcl::io::loadPCDFile<PointT> (fileName.c_str(), *original_cloud);
		  
		  std::cout << "requesting difference map for file " <<  fileName << std::endl;
		  int k = compareRawClouds(current_cloud,original_cloud);
		  /*if(k > 0){
		  	i++;
		  	fileName = pcdDirectory;
		  	fileName = fileName.append("snapshot").append(intToString(i)).append(".pcd");
		  	std::cout << "Changes detected, saving new scan to " << fileName  << std::endl; 
		  	pcl::io::savePCDFileASCII (fileName.c_str(), *current_cloud);
		  }*/
		//}
		                                  
		//Give ROS a ton of chances to actually publish the clouds
		std::cout << "Spinning ROS";

		for(int j=0;j<10;j++){
		  ros::Duration(0.1).sleep();
		  ros::spinOnce();
		  std::cout << " ."; std::cout.flush();
		}
		std::cout << std::endl;
		
		wait(); 
  }
  return 0;
}

/*
 * scan_to_ptcloud_node.cpp
 *
 *  Created on: Oct 31, 2021
 *      Author: hankm
 */


#include <ros/ros.h>
#include <ros/console.h>

#include <scan_to_ptcloud.hpp>

using namespace scan2cloud;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan2ptcloud");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::WallTime start_, end_;
  if(0) //argc !=3)
  {
	ROS_ERROR("usage: %s <filepath>\n", argv[0]);
	exit(-1);
  }

  Scan2PointCloud oScan2PtCloud((private_nh, nh)) ;
  ros::spinOnce();
  ROS_WARN("scan2ptcloud initialized \n");
//  oScan2PtCloud.initialize();

  while( ros::ok() )
  {
	  try{
		  ros::spinOnce();
	  }
	  catch(std::runtime_error& e)
	  {
		ROS_ERROR("scan 2 ptcloud exception: %s", e.what());
		return -1;
	  }
  }

  ROS_WARN(" Shutting down scan2ptcloud process \n");
  ros::shutdown();

  return 0;
}



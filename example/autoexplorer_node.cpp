/*
 * frontier_detector_node.cpp
 *
 *  Created on: Mar 18, 2021
 *      Author: hankm
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <octomap_server/mapframedata.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
//#include <signal.h>

#include "frontier_detector_sms.hpp"
#include "frontier_detector_dms.hpp"

using namespace autoexplorer;

enum SLAM_ID{GMAPPING=0, CARTOGRAPHER, SLAM_TOOLBOX};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frontier_detector");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");

  ros::WallTime start_, end_;
  if(0) //argc !=3)
  {
	ROS_ERROR("usage: %s <slam_method>\n", argv[0]);
	exit(-1);
  }

//  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
//    ROS_ERROR("usage: %s \n",argv[0]);
//    exit(-1);
//  }

  ROS_INFO("args: %s %s %s\n", argv[0], argv[1], argv[2]);

  string slam_method(argv[1]);
  int numthreads = atoi(argv[2]);
  static map<string, int> slamid;
  slamid["gmapping"] = SLAM_ID::GMAPPING;
  slamid["cartographer"] = SLAM_ID::CARTOGRAPHER;
  slamid["slam_toolbox"] = SLAM_ID::SLAM_TOOLBOX;

  numthreads = MIN( omp_get_num_threads(), numthreads );
  ROS_INFO("slamid: %s %d \n", slam_method.c_str(), slamid[slam_method.c_str()]);
//  exit(-1);

  switch( slamid[slam_method.c_str()] )
  {
  	  case GMAPPING:
  	  {
		  ROS_INFO("Initializing frontier_detector_sms \n");
		  FrontierDetectorSMS front_detector_sms(private_nh, nh);
		  ros::spinOnce();
		  front_detector_sms.initmotion();
		  front_detector_sms.SetInitMotionCompleted();
		  while( !front_detector_sms.isDone() && ros::ok() )
		  {
			  try{
				  ros::spinOnce();
			  }
			  catch(std::runtime_error& e)
			  {
				ROS_ERROR("frontier_detector exception: %s", e.what());
				return -1;
			  }
		  }
		  front_detector_sms.publishDone();
		  break;
  	  }

  	  case CARTOGRAPHER:
  	  case SLAM_TOOLBOX:
  	  {
		  ROS_INFO("Initializing frontier_detector_dms \n");
		  FrontierDetectorDMS front_detector_dms(private_nh, nh);
		  front_detector_dms.SetNumThreads(numthreads);
		  ros::spinOnce();
		  front_detector_dms.initmotion();
		  front_detector_dms.SetInitMotionCompleted();
		  while( !front_detector_dms.isDone() && ros::ok() )
		  {
			  try{
				  ros::spinOnce();
			  }
			  catch(std::runtime_error& e)
			  {
				ROS_ERROR("frontier_detector exception: %s", e.what());
				return -1;
			  }
		  }
		  //front_detector_dms.publishDone();
		  front_detector_dms.publishDoneExploration();
		  break;
  	  }
  	  default: ROS_ERROR("Invalid slam method \n");
  }

  ROS_WARN(" Sutting down frontier detection process \n");
  //ros::shutdown(); lets shutdown from outside

  return 0;
}




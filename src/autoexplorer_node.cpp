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

#include "frontier_detector.hpp"
//#include <signal.h>

using namespace frontier_detector;

int main(int argc, char** argv){
  ros::init(argc, argv, "frontier_detector");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");

//  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//     ros::console::notifyLoggerLevelsChanged();

  ros::WallTime start_, end_;
  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("something wrong with your input argument \n");
    exit(-1);
  }

  FrontierDetector front_detector(private_nh, nh);
  ros::spinOnce();

  //do init motion
  front_detector.initmotion();

//ROS_WARN("frontier_detector initialized \n");

  while( !front_detector.isDone() && ros::ok() )
  {
	  try{
		//ros::spin();
		  ros::spinOnce();
	  }
	  catch(std::runtime_error& e)
	  {
		ROS_ERROR("frontier_detector exception: %s", e.what());
		return -1;
	  }
  }

  front_detector.publishDone();
  ROS_WARN(" Sutting down frontier detection process \n");
  ros::shutdown();

  return 0;
}



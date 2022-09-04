/*********************************************************************
* Software License Agreement (XXX License)
*
*  Copyright (c) 2022, Ewha Graphics Lab
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************

 *  Created on: Apr, 2022
 *      Author: Kyungmin Han (hankm@ewha.ac.kr)
*/


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include "../include/frontier_detector_sms.hpp_"
#include "frontier_detector_dms.hpp"

using namespace autoexplorer;

enum SLAM_ID{GMAPPING=0, CARTOGRAPHER, SLAM_TOOLBOX, GT_MAPPING};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frontier_detector");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");

  ros::WallTime start_, end_;

  ROS_INFO("args: %s %s %s\n", argv[0], argv[1], argv[2]);

  string slam_method(argv[1]);
  int numthreads = atoi(argv[2]);
  static map<string, int> slamid;
  slamid["gmapping"] = SLAM_ID::GMAPPING;
  slamid["cartographer"] = SLAM_ID::CARTOGRAPHER;
  slamid["slam_toolbox"] = SLAM_ID::SLAM_TOOLBOX;
  slamid["gtmapping"]	= SLAM_ID::GT_MAPPING;

  int totnumthreads = omp_get_max_threads() ;
  ROS_INFO("max num threads, input num threads: %d %d \n", totnumthreads, numthreads);
  numthreads = MIN( totnumthreads, numthreads );
  ROS_INFO("slamid: %s %d \n", slam_method.c_str(), slamid[slam_method.c_str()]);

  switch( slamid[slam_method.c_str()] )
  {
  	  case GMAPPING:
  	  {
		  ROS_ERROR("Autoexplorer doesn't support GMAPPING yet. It will be ready soon. \n");
		  return -1;
//		  FrontierDetectorSMS front_detector_sms(private_nh, nh);
//		  ros::spinOnce();
//		  front_detector_sms.initmotion();
//		  front_detector_sms.SetInitMotionCompleted();
//		  while( !front_detector_sms.isDone() && ros::ok() )
//		  {
//			  try{
//				  ros::spinOnce();
//			  }
//			  catch(std::runtime_error& e)
//			  {
//				ROS_ERROR("frontier_detector exception: %s", e.what());
//				return -1;
//			  }
//		  }
//		  front_detector_sms.publishDone();
//		  break;
  	  }

  	  case CARTOGRAPHER:
  	  {
		  ROS_ERROR("Autoexplorer is not compatible with CARTOGRAPHER \n");
		  return -1;
  	  }
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

  	  case GT_MAPPING:
  	  {
		  ROS_INFO("Initializing frontier_detector_dms \n");
		  FrontierDetectorDMS front_detector_dms(private_nh, nh);

		  ROS_INFO("Setting num of thread %d \n", numthreads);
		  front_detector_dms.SetNumThreads(numthreads);
		  ros::spinOnce();
		  //front_detector_dms.initmotion();
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

  ROS_WARN(" Sutting down the exploration task \n");
  //ros::shutdown(); lets shutdown from outside

  return 0;
}




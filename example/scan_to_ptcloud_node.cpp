/*********************************************************************
Copyright 2022 The Ewha Womans University.
All Rights Reserved.

Permission to use, copy, modify OR distribute this software and its
documentation for educational, research and non-profit purposes, without
fee, and without a written agreement is hereby granted, provided that the
above copyright notice and the following three paragraphs appear in all
copies.

IN NO EVENT SHALL THE EWHA WOMANS UNIVERSITY BE
LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE EWHA WOMANS UNIVERSITY
BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

THE EWHA WOMANS UNIVERSITY SPECIFICALLY DISCLAIM ANY
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE EWHA WOMANS UNIVERSITY
HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
UPDATES, ENHANCEMENTS, OR MODIFICATIONS.


The authors may be contacted via:


Mail:        Y. J. Kim, Kyung Min Han
             Computer Graphics Lab                       
             Department of Computer Science and Engineering
             Ewha Womans University
             11-1 Daehyun-Dong Seodaemun-gu, Seoul, Korea 120-750


Phone:       +82-2-3277-6798


EMail:       kimy@ewha.ac.kr
             hankm@ewha.ac.kr
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



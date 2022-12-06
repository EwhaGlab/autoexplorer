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

#ifndef INCLUDE_SCAN2PTCLOUD_SCAN_TO_PTCLOUD_HPP_
#define INCLUDE_SCAN2PTCLOUD_SCAN_TO_PTCLOUD_HPP_


#include <ros/console.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "std_msgs/Header.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"

//#include "tf2_ros/transform_listener.h"
//#include "tf2_ros/message_filter.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_listener.h"

#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "laser_geometry/laser_geometry.h"


namespace scan2cloud
{

using namespace std;

class Scan2PointCloud
{
public:
	Scan2PointCloud(std::ofstream* poutofs, std::ofstream* pofsinfo){};
	Scan2PointCloud(const std::string& strfile_ofs, const std::string& strinfo_ofs);
	Scan2PointCloud(const ros::NodeHandle private_nh_ = ros::NodeHandle("~"), const ros::NodeHandle &nh_ = ros::NodeHandle()) ;
	void initialize();

	virtual ~Scan2PointCloud();

	void scanCallback( const sensor_msgs::LaserScan::ConstPtr& scan_in );

private:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	ros::Subscriber m_laserScanSub ;
	ros::Publisher  m_ptCloudPub;

	std::string m_mapFrameId ;
	std::string m_laserlinkFrameId ;
	std::string m_baseFrameId ;

	tf::TransformListener m_tfListener;

	laser_geometry::LaserProjection m_projector;
	std::ofstream m_outfilestream;
	std::ofstream m_ofsinfo ;

//	std::ofstream* mp_outfilestream;
//	std::ofstream* mp_ofsinfo ;

	int m_nScanCnt ;
//	message_filters::Subscriber<geometry_msgs::Point>
};

}

#endif /* INCLUDE_SCAN2PTCLOUD_SCAN_TO_PTCLOUD_HPP_ */

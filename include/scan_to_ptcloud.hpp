/*
 * scan_to_ptcloud.hpp
 *
 *  Created on: Oct 31, 2021
 *      Author: hankm
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

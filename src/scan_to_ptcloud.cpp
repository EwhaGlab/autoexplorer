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

//#ifndef SRC_SCAN_TO_PTCLOUD_CPP_
//#define SRC_SCAN_TO_PTCLOUD_CPP_

#include <scan_to_ptcloud.hpp>

namespace scan2cloud
{

//Scan2PointCloud::Scan2PointCloud(std::ofstream* poutofs, std::ofstream* pofsinfo):
//mp_outfilestream(poutofs),
//mp_ofsinfo(pofsinfo),
//m_nScanCnt(0)
//{
//	//m_tfListener = TransformListener(m_tfBuffer);
//
//	m_ptCloudPub		= m_nh.advertise<sensor_msgs::PointCloud2>("scan_ptcloud",1);
//	m_laserScanSub  	= m_nh.subscribe("base_scan", 1, &Scan2PointCloud::scanCallback, this); // kmHan
//};

Scan2PointCloud::Scan2PointCloud(const std::string& strfile_ofs, const std::string& strinfo_ofs):
m_nScanCnt(0)
{
	//m_tfListener = TransformListener(m_tfBuffer);

	m_outfilestream.open(strfile_ofs);
	m_ofsinfo.open(strinfo_ofs) ;

	m_ptCloudPub		= m_nh.advertise<sensor_msgs::PointCloud2>("scan_ptcloud",1);
	m_laserScanSub  	= m_nh.subscribe("base_scan", 1, &Scan2PointCloud::scanCallback, this); // kmHan
};

Scan2PointCloud::Scan2PointCloud(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):
m_nh_private(private_nh_),
m_nh(nh_),
m_baseFrameId("base_link")
{
	m_nScanCnt = 0;
	ROS_INFO("instantiating the Scan2PointCloud node \n");
	m_ptCloudPub		= m_nh.advertise<sensor_msgs::PointCloud2>("scan_ptcloud",1);
	m_laserScanSub  	= m_nh.subscribe("scan", 1, &Scan2PointCloud::scanCallback, this); // kmHan

//	if(!m_nh.hasParam("base_frame_id"))
//		ROS_ERROR("No param named 'base_frame_id' \n");

	//ros::param::get("~base_frame_id", m_baseFrameId);
	m_nh_private.param("scan2ptcloud/base_frame_id", m_baseFrameId, m_baseFrameId);

	ROS_WARN("@scan2ptcloud baseframe id: %s\n", m_baseFrameId.c_str() );
};


//void Scan2PointCloud::initialize()
//{
//	m_nScanCnt = 0;
//	ROS_INFO("instantiating the Scan2PointCloud node \n");
//	m_ptCloudPub		= m_nh.advertise<sensor_msgs::PointCloud2>("scan_ptcloud",1);
//	m_laserScanSub  	= m_nh.subscribe("scan", 1, &Scan2PointCloud::scanCallback, this); // kmHan
//};


Scan2PointCloud::~Scan2PointCloud()
{
	//m_outfilestream.close();
};


void Scan2PointCloud::scanCallback( const sensor_msgs::LaserScan::ConstPtr& scan_in )
{

//	if(!m_tfListener.waitForTransform(
//		scan_in->header.frame_id,
//		"/base_link",
//		scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
//		ros::Duration(1.0))){
//	 return;
//	}


	tf::StampedTransform transform;
    try{
    	m_tfListener.lookupTransform(m_baseFrameId, scan_in->header.frame_id,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

	sensor_msgs::PointCloud cloud;
	m_projector.transformLaserScanToPointCloud(scan_in->header.frame_id,*scan_in, cloud, m_tfListener);

	//ROS_INFO("cloud size: %ld \n", cloud.points.size());

	sensor_msgs::PointCloud2 cloud2;
	sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
	m_ptCloudPub.publish(cloud2);

  // Do something with cloud.
};



}

//#endif /* SRC_SCAN_TO_PTCLOUD_CPP_ */

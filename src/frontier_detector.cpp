/*
 * frontier_detector.cpp
 *
 *  Created on: Sep 29, 2021
 *      Author: hankm
 */


#include "frontier_detector.hpp"


namespace frontier_detector
{

FrontierDetector::FrontierDetector():
m_worldFrameId("map"), m_baseFrameId("base_link"),
m_globalcostmap_rows(0), m_globalcostmap_cols(0), m_eRobotState(ROBOT_STATE::ROBOT_IS_NOT_MOVING),
m_move_client("move_base", true),
m_fRobotRadius(0.3), isdone(false), m_nroi_origx(0), m_nroi_origy(0), m_nrows(0), m_ncols(0)
{}

FrontierDetector::~FrontierDetector(){}

cv::Point2f FrontierDetector::img2gridmap( cv::Point img_pt ){};
cv::Point FrontierDetector::gridmap2img( cv::Point2f grid_pt ){};

void FrontierDetector::mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_ERROR("this shouldn't be called \n");
}
vector<cv::Point> FrontierDetector::eliminateSupriousFrontiers( nav_msgs::OccupancyGrid &costmapData, vector<cv::Point> frontierCandidates, int winsize = 25)
{}

//void accessFrontierPoint( ){}


}


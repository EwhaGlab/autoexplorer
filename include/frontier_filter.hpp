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


#ifndef INCLUDE_FRONTIER_FILTER_HPP_
#define INCLUDE_FRONTIER_FILTER_HPP_

#include <fstream>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "octomap_server/mapframedata.h"

#include <ros/ros.h>
#include <ros/console.h>
#include "nav_msgs/OccupancyGrid.h"

#include "frontier_point.hpp"

using namespace std;
namespace autoexplorer
{

class FrontierFilter
{
public:

	FrontierFilter();

	FrontierFilter(
			int ncostmap_roi_size, int ngridmap_roi_size, std::string str_debugpath, int nNumPyrDownSample,
			float fgridmap_conf_thr, float fcostmap_conf_thr, int noccupancy_thr, int nlethal_cost_thr,
			int nGlobalMapWidth, int nGlobalMapHeight,
			float fGMResolution
	);
	virtual ~FrontierFilter();

	cv::Point2f gridmap2world( cv::Point img_pt_gm  )
	{
		float fpx = static_cast<float>(img_pt_gm.x) - m_nGlobalMapCentX;
		float fpy = static_cast<float>(img_pt_gm.y) - m_nGlobalMapCentY;

	//	ROS_INFO("%f %f %f %d %d\n", fpx, fResolution, fXstart, m_nScale, m_nNumPyrDownSample );
	//	ROS_INFO("%f %f %f %d \n", fpy, fResolution, fYstart, m_nScale );
		float fgx = ( fpx * m_fGMResolution ); // + fXstart ) ;
		float fgy = ( fpy * m_fGMResolution ); // - fYstart ) ;

		return cv::Point2f( fgx, fgy );
	}
	//
	cv::Point world2gridmap( cv::Point2f grid_pt)
	{
		float fx = grid_pt.x / m_fGMResolution + m_nGlobalMapCentX;
		float fy = grid_pt.y / m_fGMResolution + m_nGlobalMapCentY;

		int nx = static_cast<int>( fx );
		int ny = static_cast<int>( fy );

		return cv::Point2f( nx, ny );
	}

	void measureCostmapConfidence( const nav_msgs::OccupancyGrid& costmapData, std::vector<FrontierPoint>& voFrontierCandidates ) ;
	void measureGridmapConfidence( const nav_msgs::OccupancyGrid& gridmapData, std::vector<FrontierPoint>& voFrontierCandidates ) ;
	void computeReachability( const set<pointset, pointset>& unreachable_frontiers, std::vector<FrontierPoint>& voFrontierCandidates ) ;

	inline float GetCostmapConf() const
	{
		return m_fcostmap_conf_thr ;
	}

	inline float GetGridmapConf()
	{
		return m_fgridmap_conf_thr ;
	}

private:

	int m_ncostmap_roi_size ;
	int m_ngridmap_roi_size ;

	std::string m_str_debugpath;
	float m_fgridmap_conf_thr;
	float m_fcostmap_conf_thr;
	int m_noccupancy_thr;
	int m_nlethal_cost_thr;
	int m_nGlobalMapWidth ;
	int m_nGlobalMapHeight;
	int m_nGlobalMapCentX;
	int m_nGlobalMapCentY;
	float m_fGMResolution;
	int m_nScale;

};

}

#endif /* INCLUDE_FRONTIER_FILTER_HPP_ */

/*********************************************************************
Copyright 2022 The Ewha Womans University.
All Rights Reserved.
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
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
			float fGMResolution,  float funreachable_decision_bound
	);
	virtual ~FrontierFilter();

	cv::Point2f gridmap2world( cv::Point img_pt_gm  )
	{
		float fpx = static_cast<float>(img_pt_gm.x) - mn_globalmap_centx;
		float fpy = static_cast<float>(img_pt_gm.y) - mn_globalmap_centy;

	//	ROS_INFO("%f %f %f %d %d\n", fpx, fResolution, fXstart, m_nScale, m_nNumPyrDownSample );
	//	ROS_INFO("%f %f %f %d \n", fpy, fResolution, fYstart, m_nScale );
		float fgx = ( fpx * mf_gmresolution ); // + fXstart ) ;
		float fgy = ( fpy * mf_gmresolution ); // - fYstart ) ;

		return cv::Point2f( fgx, fgy );
	}
	//
	cv::Point world2gridmap( cv::Point2f grid_pt)
	{
		float fx = grid_pt.x / mf_gmresolution + mn_globalmap_centx;
		float fy = grid_pt.y / mf_gmresolution + mn_globalmap_centy;

		int nx = static_cast<int>( fx );
		int ny = static_cast<int>( fy );

		return cv::Point2f( nx, ny );
	}

	inline bool frontier_sanity_check( int nx, int ny, int nwidth, const std::vector<signed char>& cmdata )
	{
		// 0	1	 2
		// 3		 5
		// 6	7	 8

		int i0 = nwidth * (ny - 1) 	+	nx - 1 ;
		int i1 = nwidth * (ny - 1) 	+	nx 		;
		int i2 = nwidth * (ny - 1) 	+	nx + 1	;
		int i3 = nwidth * ny			+	nx - 1 ;
		int i5 = nwidth * ny			+	nx + 1 ;
		int i6 = nwidth * (ny + 1)	+	nx - 1 ;
		int i7 = nwidth * (ny + 1)	+	nx		;
		int i8 = nwidth * (ny + 1)	+	nx + 1 ;

		//ROS_INFO("width i0 val : %d %d %d\n", nwidth, i0, gmdata[i0] );
		if( cmdata[i0] > mn_lethal_cost_thr || cmdata[i1] > mn_lethal_cost_thr || cmdata[i2] > mn_lethal_cost_thr || cmdata[i3] > mn_lethal_cost_thr ||
			cmdata[i5] > mn_lethal_cost_thr || cmdata[i6] > mn_lethal_cost_thr || cmdata[i7] > mn_lethal_cost_thr || cmdata[i8] > mn_lethal_cost_thr )
		{
			return false;
		}

		if( cmdata[i0] < 0 || cmdata[i1] < 0 || cmdata[i2] < 0 || cmdata[i3] < 0 ||  cmdata[i5] < 0 || cmdata[i6] < 0 || cmdata[i7] < 0 || cmdata[i8] < 0 )
		{
			return true ;
		}
		else
		{
			return false ;
		}
	}

	void measureCostmapConfidence( const nav_msgs::OccupancyGrid& costmapData, std::vector<FrontierPoint>& voFrontierCandidates ) ;
	void measureGridmapConfidence( const nav_msgs::OccupancyGrid& gridmapData, std::vector<FrontierPoint>& voFrontierCandidates ) ;
	void computeReachability( const set<pointset>& unreachable_frontiers, std::vector<FrontierPoint>& voFrontierCandidates ) ;
	bool isReachable( const set<pointset>& unreachable_frontiers, const float& fxcand, const float& fycand ) ;

	inline float GetCostmapConf() const
	{
		return mf_costmap_conf_thr ;
	}

	inline float GetGridmapConf()
	{
		return mf_gridmap_conf_thr ;
	}

	void SetUnreachableDistThr( const float& fdist )
	{
		mf_unreachable_dist_thr = fdist ;
	}

private:

	int mn_costmap_roi_size ;
	int mn_gridmap_roi_size ;

	std::string mstr_debugpath;
	float mf_gridmap_conf_thr;
	float mf_costmap_conf_thr;
	float mf_unreachable_dist_thr ;

	int mn_occupancy_thr;
	int mn_lethal_cost_thr;
	int mn_globalmap_width ;
	int mn_globalmap_height;
	int mn_globalmap_centx;
	int mn_globalmap_centy;
	float mf_gmresolution;
	int mn_scale;

};

}

#endif /* INCLUDE_FRONTIER_FILTER_HPP_ */

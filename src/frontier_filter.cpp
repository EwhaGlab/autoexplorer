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


#include "frontier_filter.hpp"

namespace autoexplorer
{

FrontierFilter::FrontierFilter(){};

FrontierFilter::FrontierFilter(
		int ncostmap_roi_size, int ngridmap_roi_size, std::string str_debugpath, int nNumPyrDownSample,
		float fgridmap_conf_thr, float fcosmap_conf_thr, int noccupancy_thr, int nlethal_cost_thr,
		int nGlobalMapWidth, int nGlobalMapHeight, float fGMResolution ):
m_ncostmap_roi_size(ncostmap_roi_size), m_ngridmap_roi_size(ngridmap_roi_size), m_str_debugpath(str_debugpath),
m_fcostmap_conf_thr(fcosmap_conf_thr), m_fgridmap_conf_thr(fgridmap_conf_thr), m_noccupancy_thr(noccupancy_thr), m_nlethal_cost_thr(nlethal_cost_thr),
m_nGlobalMapWidth(nGlobalMapWidth), m_nGlobalMapHeight(nGlobalMapHeight), m_fGMResolution(fGMResolution)
{
	m_nGlobalMapCentX = nGlobalMapWidth  / 2 ;
	m_nGlobalMapCentY = nGlobalMapHeight / 2 ;
	m_nScale		  = pow(2, nNumPyrDownSample) ;
}
FrontierFilter::~FrontierFilter()
{
}

void FrontierFilter::measureCostmapConfidence( const nav_msgs::OccupancyGrid& costmapData, std::vector<FrontierPoint>& voFrontierCandidates )
{
//ROS_INFO("eliminating suprious frontiers in the costmap \n");
static int cmapidx = 0;

	float fXstart=costmapData.info.origin.position.x; // world coordinate in the costmap
	float fYstart=costmapData.info.origin.position.y; // world coordinate in the costmap
	float resolution = costmapData.info.resolution ;

	int width= static_cast<int>(costmapData.info.width) ;
	int height= static_cast<int>(costmapData.info.height) ;

	std::vector<signed char> Data=costmapData.data;

//ROS_INFO("origin in costmap: %f %f\n", fXstart, fXstart );

#ifdef FD_DEBUG_MODE
	m_nglobalcostmapidx++;

	char tmp0[200], tmp1[200], tmp2[200];
	sprintf(tmp0, "%s/front_in_costmap%05d.txt", m_str_debugpath.c_str(), m_nglobalcostmapidx) ;
	sprintf(tmp1, "%s/costmap%05d.txt", m_str_debugpath.c_str(), m_nglobalcostmapidx) ;
	sprintf(tmp2, "%s/gridmap%05d.txt", m_str_debugpath.c_str(), m_nglobalcostmapidx) ;

	string str_front_in_costmap(tmp0);
	string str_costmap_file(tmp1);
	string str_gridmap_file(tmp2);

	//ROS_INFO("saving %s\n", str_front_in_costmap.c_str());

	ofstream ofs_incostmap(str_front_in_costmap) ;
	ofstream ofs_costmap(str_costmap_file) ;
//	ofstream ofs_gridmap(str_gridmap_file) ;

	for(int ridx = 0; ridx < height; ridx++)
	{
		for(int cidx = 0; cidx < width; cidx++)
		{
			int cost = static_cast<int>( Data[ridx * width + cidx] ) ;
			ofs_costmap << cost << " ";
//			ofs_gridmap << m_gridmap.data[ ridx * width + cidx ] << " ";
	//		costmap.data[ridx * width + cidx] = cost ;
//			if(cost < 0 )
//				ofs_costmap << 127 << " ";
//			else if(cost == 0)
//				ofs_costmap << 0 << " ";
//			else
//				ofs_costmap << cost * 2 + 50 << " ";
		}
		ofs_costmap << endl;
//		ofs_gridmap << endl;
	}
	ofs_costmap.close();
//	ofs_gridmap.close();
#endif

	for( size_t idx =0; idx < voFrontierCandidates.size(); idx++) // frontiers in image coord
	{
		cv::Point frontier_in_gridmap = voFrontierCandidates[idx].GetCorrectedGridmapPosition() ;

		//returns grid value at "Xp" location
		//map data:  100 occupied      -1 unknown       0 free
		int py_c = frontier_in_gridmap.y ;
		int px_c = frontier_in_gridmap.x ;

		//int py_c=  floor( (frontier_in_world.y - fYstart ) / resolution  ) ; // px in costmap
		//int px_c=  floor( (frontier_in_world.x - fXstart ) / resolution  ) ;
		int8_t cost ;
		int32_t ncost = 0;

		int sx = MAX(px_c - m_ncostmap_roi_size, 0);
		int ex = MIN(px_c + m_ncostmap_roi_size, width) ;
		int sy = MAX(py_c - m_ncostmap_roi_size, 0);
		int ey = MIN(py_c + m_ncostmap_roi_size, height) ;
//ROS_INFO("cm test window: %d %d %d %d \n", sx, ex, sy, ey);
		//ofs_fpc << px_c << " " << py_c << endl;

#ifdef FD_DEBUG_MODE
#endif

		cv::Mat roi = cv::Mat::zeros(ey - sy + 1, ex - sx + 1, CV_8S);

		int cellcnt = 0;
		int totcost = 0;
		for( int ridx =sy; ridx < ey; ridx++)
		{
			for( int cidx=sx; cidx < ex; cidx++)
			{
				//int dataidx = px_c + cidx + (py_c + ridx) * width ;
				int dataidx = ridx * width + cidx ;
//ROS_INFO("ind rix cidx %d %d %d ", idx, ridx, cidx);
				cost = Data[dataidx] ; // orig 0 ~ 254 --> mapped to 0 ~ 100
				if(cost >= 0 )// m_nlethal_cost_thr) //LEATHAL_COST_THR ) // unknown (-1)
				{
					//ncost++;
					totcost += static_cast<int>(cost);
				}
#ifdef FD_DEBUG_MODE
	ofs_fptroi << cost << " ";
#endif
				//roi.data[ ridx * width + cidx ] = cost ;
				cellcnt++;
			}

#ifdef FD_DEBUG_MODE
	ofs_fptroi << endl;
#endif

		}
		float fcost = static_cast<float>(totcost) / ( static_cast<float>( cellcnt ) * 100  );
		float fcm_conf = 1.f - fcost ;
		voFrontierCandidates[idx].SetCostmapConfidence(fcm_conf);

#ifdef FD_DEBUG_MODE
		ofs_fptroi.close();
		ofs_incostmap.close();
#endif

	}

}

void FrontierFilter::measureGridmapConfidence( const nav_msgs::OccupancyGrid& gridmapData, std::vector<FrontierPoint>& voFrontierCandidates )
{

	float fXstart=gridmapData.info.origin.position.x; // world coordinate in the costmap
	float fYstart=gridmapData.info.origin.position.y; // world coordinate in the costmap
	float resolution = gridmapData.info.resolution ;

	int width= static_cast<int>(gridmapData.info.width) ;
	int height= static_cast<int>(gridmapData.info.height) ;

	std::vector<signed char> Data=gridmapData.data;
//ROS_INFO("origin in gridmap: %f %f\n", fYstart, fXstart );

	for( size_t idx =0; idx < voFrontierCandidates.size(); idx++) // frontiers in image coord
	{
		cv::Point frontier_in_gm = voFrontierCandidates[idx].GetCorrectedGridmapPosition() ;
		//returns grid value at "Xp" location
		//map data:  100 occupied      -1 unknown       0 free
		int px_g = frontier_in_gm.x ;
		int py_g = frontier_in_gm.y ;

		int8_t cost ;
		int32_t ncost = 0;

		int sx = MAX(px_g - m_ngridmap_roi_size, 0);
		int ex = MIN(px_g + m_ngridmap_roi_size, width) ;
		int sy = MAX(py_g - m_ngridmap_roi_size, 0);
		int ey = MIN(py_g + m_ngridmap_roi_size, height) ;

//ROS_INFO(" idx px py %u %d %d\n", idx, px_c, py_c);
		cv::Mat roi = cv::Mat::zeros(ey - sy + 1, ex - sx + 1, CV_8S);

		int cellcnt = 0;
		int unkncnt = 0;
		for( int ridx =sy; ridx < ey; ridx++)
		{
			for( int cidx=sx; cidx < ex; cidx++)
			{
				int dataidx = ridx * width + cidx ;
//ROS_INFO("ind rix cidx %d %d %d ", idx, ridx, cidx);
				cost = Data[dataidx] ; // orig 0 ~ 254 --> mapped to 0 ~ 100
				if(cost < 0 )// m_nlethal_cost_thr) //LEATHAL_COST_THR ) // unknown (-1)
				{
					unkncnt++;
				}
				//roi.data[ ridx * width + cidx ] = cost ;
				cellcnt++;
			}
		}
		float fscore = static_cast<float>(unkncnt) / ( static_cast<float>( cellcnt ) );
		float fconfd = 1.f - std::abs( fscore - 0.5 )/0.5 ;  // fscore = 0 ~ 1  best: 0.5
		voFrontierCandidates[idx].SetGridmapConfidence( fconfd );

#ifdef FD_DEBUG_MODE
		ofs_fptroi.close();
		ofs_incostmap.close();
#endif

	}

}

void FrontierFilter::computeReachability( const set<pointset, pointset>& unreachable_frontiers, std::vector<FrontierPoint>& voFrontierCandidates )
{

// classify unreachable points

	if( !unreachable_frontiers.empty() )
	{
		vector<cv::Point> validFrontiers ;
		cv::Point pt_img;
		for (size_t i=0; i < voFrontierCandidates.size(); i++ )
		{
			FrontierPoint pt = voFrontierCandidates[i];
			if( !pt.isConfidentFrontierPoint() )
				continue ;

			cv::Point2f frontier_in_world = pt.GetCorrectedWorldPosition() ;
			float fx = frontier_in_world.x ;
			float fy = frontier_in_world.y ;
			for (const auto & di : unreachable_frontiers)
			{
				float fdist = std::sqrt( (fx - di.d[0]) * (fx - di.d[0]) + (fy - di.d[1]) * (fy - di.d[1]) ) ;
				if(fdist < 0.2)
				{
					ROS_WARN("(%f %f) and (%f %f) are the same? unreachable point? \n",
								fx, fy, di.d[0], di.d[1]);
					voFrontierCandidates[i].SetReachability(false);
				}
			}
		}
	}

}



}

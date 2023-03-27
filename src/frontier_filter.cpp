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


#include "frontier_filter.hpp"

namespace autoexplorer
{

FrontierFilter::FrontierFilter(){};

FrontierFilter::FrontierFilter(
		int ncostmap_roi_size, int ngridmap_roi_size, std::string str_debugpath, int nNumPyrDownSample,
		float fgridmap_conf_thr, float fcosmap_conf_thr, int noccupancy_thr, int nlethal_cost_thr,
		int nGlobalMapWidth, int nGlobalMapHeight, float fGMResolution, float funreachable_decision_bound ):
mn_costmap_roi_size(ncostmap_roi_size), mn_gridmap_roi_size(ngridmap_roi_size), mstr_debugpath(str_debugpath),
mf_costmap_conf_thr(fcosmap_conf_thr), mf_gridmap_conf_thr(fgridmap_conf_thr), mn_occupancy_thr(noccupancy_thr), mn_lethal_cost_thr(nlethal_cost_thr),
mn_globalmap_width(nGlobalMapWidth), mn_globalmap_height(nGlobalMapHeight), mf_gmresolution(fGMResolution),
mf_unreachable_dist_thr(funreachable_decision_bound)
{
	mn_globalmap_centx = nGlobalMapWidth  / 2 ;
	mn_globalmap_centy = nGlobalMapHeight / 2 ;
	mn_scale		  = pow(2, nNumPyrDownSample) ;
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
		}
		ofs_costmap << endl;
	}
	ofs_costmap.close();
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

		int sx = MAX(px_c - mn_costmap_roi_size, 0);
		int ex = MIN(px_c + mn_costmap_roi_size, width) ;
		int sy = MAX(py_c - mn_costmap_roi_size, 0);
		int ey = MIN(py_c + mn_costmap_roi_size, height) ;
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
				cost = Data[dataidx] ; // 0 ~ 254 --> mapped to 0 ~ 100
				//if(cost >= 0 )// m_nlethal_cost_thr) //LEATHAL_COST_THR ) // unknown (-1)
				if(cost >  mn_lethal_cost_thr) //LEATHAL_COST_THR ) // unknown (-1)
				{
					//ncost++;
					//totcost += static_cast<int>(cost);
					totcost += 1;
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
		float fcost = static_cast<float>(totcost) / static_cast<float>( cellcnt )  ;
		float fcm_conf = 1.f - std::sqrt( fcost ) ;
		voFrontierCandidates[idx].SetCostmapConfidence(fcm_conf);

		ROS_DEBUG_NAMED("autoexplorer","pt cm conf: %d %d %f (%d/%d)\n", px_c, py_c, fcm_conf, totcost, cellcnt);
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

		int sx = MAX(px_g - mn_gridmap_roi_size, 0);
		int ex = MIN(px_g + mn_gridmap_roi_size, width) ;
		int sy = MAX(py_g - mn_gridmap_roi_size, 0);
		int ey = MIN(py_g + mn_gridmap_roi_size, height) ;

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

void FrontierFilter::computeReachability( const set<pointset>& unreachable_frontiers, std::vector<FrontierPoint>& voFrontierCandidates )
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
				float fdist = std::sqrt( (fx - di.p[0]) * (fx - di.p[0]) + (fy - di.p[1]) * (fy - di.p[1]) ) ;
				if(fdist < mf_unreachable_dist_thr)
				{
//					ROS_WARN("(%f %f) is an identical point of (%f %f) which is an unreachable pt \n",
//								fx, fy, di.d[0], di.d[1]);
					voFrontierCandidates[i].SetReachability(false);
					voFrontierCandidates[i].SetFrontierFlag(false);
				}
			}
		}
	}
}

bool FrontierFilter::isReachable( const set<pointset>& unreachable_frontiers, const float& fxcand, const float& fycand )
{
	for (const auto & di : unreachable_frontiers)
	{
		float fdist = std::sqrt( (fxcand - di.p[0]) * (fxcand - di.p[0]) + (fycand - di.p[1]) * (fycand - di.p[1]) ) ;
		if(fdist < mf_unreachable_dist_thr)
			return false;
	}

	return true;
}


}
